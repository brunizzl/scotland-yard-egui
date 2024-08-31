use super::*;

/// BitVec-like structure, but specialized for use in [`RobberStratQueue`].
struct BitQueue {
    /// can't grow, acts like bitset
    data: Vec<usize>,
    /// logical maximum number of entries
    capacity: usize,

    /// current index in [`Self::data`]
    data_iter: usize,
    /// copy of [`Self::data`] at [`Self::data_iter`], except just inserted items are missing
    data_curr: usize,
}

/// number of bits contained in single value of usize
const BITS: usize = usize::BITS as usize;

impl BitQueue {
    /// fails if not enough memory is available
    pub fn new(length: usize) -> Option<Self> {
        let mut data = Vec::new();
        // guarantee data is nonempty to simplify access
        let data_len = (length + BITS - 1) / BITS;
        let data_cap = usize::max(data_len, 1);
        data.try_reserve_exact(data_cap).ok()?;
        data.resize(data_len, usize::MAX);
        data.resize(data_cap, 0);

        let last_count = length % BITS;
        if last_count != 0 {
            let last_val = (1usize << last_count) - 1;
            data[data_len - 1] = last_val;
        }
        debug_assert_eq!(
            data.iter().map(|x| x.count_ones() as usize).sum::<usize>(),
            length
        );

        let data_curr = data[0];
        Some(Self {
            data,
            capacity: length,
            data_iter: 0,
            data_curr,
        })
    }

    /// [`Self::pop_next_one`] only walks forwards by itself.
    /// this method resets the internal iteration state to again search entries from the start.
    pub fn wrap_back(&mut self) {
        self.data_iter = 0;
        self.data_curr = self.data[0];
    }

    pub fn pop_next_one(&mut self) -> Option<usize> {
        let next_off = 'find_next: loop {
            let next_off = self.data_curr.trailing_zeros() as usize;
            if next_off < BITS {
                break 'find_next next_off;
            }
            self.data_iter += 1;
            if self.data_iter == self.data.len() {
                return None;
            }
            self.data_curr = self.data[self.data_iter];
        };

        let mask = 1usize << next_off;
        self.data[self.data_iter] ^= mask;
        self.data_curr ^= mask;
        debug_assert!(self.data_iter * BITS + next_off < self.capacity);
        Some(self.data_iter * BITS + next_off)
    }

    pub fn set(&mut self, pos: usize) -> bool {
        debug_assert!(pos < self.capacity);
        let data_i = pos / BITS;
        let int_i = pos % BITS;
        let mask = 1usize << int_i;
        let old = self.data[data_i] & mask != 0;
        self.data[data_i] |= mask;
        old
    }
}

/// this is special, as we know exactly what entries could possibly be enqueued.
/// also: initially, all entries must be present.
/// we thus don't actually use a queue, but store one bit for each
/// [`CompactCops`] configuration held in [`CopConfigurations`].
///
/// we then remember what the last unqueued [`CompactCops`] value was
/// and search the next one to unqueue in the following bits.
/// if all bits are visited, we start anew at bit 0, until no bits are set.
pub struct RobberStratQueue {
    /// stores which values are enqueued
    bit_queues: Vec<queues::BitQueue>,
    /// maps each of [`Self::bit_queues`] to their representative vertex
    firsts: Vec<usize>,

    /// iteration state: index in both [`Self::bit_queues`] and [`Self::firsts`]
    first_index: usize,
    /// iteration state: how many items are currently enqueued
    length: usize,
    /// statistics: how many times have we checked each bit of [`Self::bit_queues`]
    rounds_complete: usize,
}

impl RobberStratQueue {
    /// fails if not enough memory is available
    pub fn new(cop_moves: &CopConfigurations) -> Option<Self> {
        let mut bit_queues = Vec::new();
        let mut firsts = Vec::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let queue = queues::BitQueue::new(indices.len())?;
            bit_queues.push(queue);
            firsts.push(fst_index);
        }

        Some(Self {
            bit_queues,
            firsts,
            first_index: 0,
            rounds_complete: 0,
            length: cop_moves.nr_configurations(),
        })
    }

    pub fn push(&mut self, entry: CompactCopsIndex) {
        let first = self.firsts.iter().position(|&f| f == entry.fst_index).unwrap();
        if !self.bit_queues[first].set(entry.rest_index) {
            self.length += 1;
        }
    }

    pub fn pop(&mut self) -> Option<CompactCopsIndex> {
        if self.length == 0 {
            return None;
        }
        loop {
            let curr_queue = &mut self.bit_queues[self.first_index];
            if let Some(rest_index) = curr_queue.pop_next_one() {
                self.length -= 1;
                let fst_index = self.firsts[self.first_index];
                return Some(CompactCopsIndex { fst_index, rest_index });
            }
            curr_queue.wrap_back();
            self.first_index += 1;
            if self.first_index == self.firsts.len() {
                self.first_index = 0;
                self.rounds_complete += 1;
            }
        }
    }

    pub fn len(&self) -> usize {
        self.length
    }

    pub fn rounds_complete(&self) -> usize {
        self.rounds_complete
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InQueue {
    No,
    Yes,
    NoAndAtMax,
    YesAndAtMax,
}

pub struct CopStratQueue {
    queue: VecDeque<CompactCopsIndex>,
    status: BTreeMap<usize, Vec<InQueue>>,
    curr_max_nr_moves: UTime,
}

impl CopStratQueue {
    pub fn new(cop_moves: &CopConfigurations) -> Option<Self> {
        let mut contained = BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len();

            let mut data = Vec::new();
            data.try_reserve(nr_entries).ok()?;
            data.resize(nr_entries, InQueue::Yes);
            let old = contained.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        let mut queue = VecDeque::new();
        queue.try_reserve(cop_moves.nr_configurations()).ok()?;
        queue.extend(cop_moves.all_positions());
        Some(Self {
            queue,
            status: contained,
            curr_max_nr_moves: 1,
        })
    }

    fn status_mut_of(&mut self, entry: CompactCopsIndex) -> &mut InQueue {
        &mut self.status.get_mut(&entry.fst_index).unwrap()[entry.rest_index]
    }

    pub fn push(&mut self, entry: CompactCopsIndex) {
        let entry_status = self.status_mut_of(entry);

        let enqueue = match *entry_status {
            InQueue::No => {
                *entry_status = InQueue::Yes;
                true
            },
            InQueue::NoAndAtMax => {
                *entry_status = InQueue::YesAndAtMax;
                true
            },
            _ => false,
        };
        if enqueue {
            self.queue.push_back(entry);
        }
    }

    pub fn mark_as_at_max(&mut self, entry: CompactCopsIndex) {
        let entry_status = self.status_mut_of(entry);
        match *entry_status {
            InQueue::No => {
                *entry_status = InQueue::NoAndAtMax;
            },
            InQueue::Yes => {
                *entry_status = InQueue::YesAndAtMax;
            },
            _ => {},
        }
    }

    fn pop_and_mark_popped(&mut self) -> Option<CompactCopsIndex> {
        let res = self.queue.pop_front();
        if let Some(entry) = res {
            let entry_status = self.status_mut_of(entry);
            match *entry_status {
                InQueue::Yes => *entry_status = InQueue::No,
                InQueue::YesAndAtMax => *entry_status = InQueue::NoAndAtMax,
                _ => panic!(),
            }
        }
        res
    }

    pub fn pop(&mut self) -> Option<CompactCopsIndex> {
        if let Some(entry) = self.pop_and_mark_popped() {
            return Some(entry);
        }
        self.curr_max_nr_moves += 1;
        for (&fst_index, part) in &mut self.status {
            for (rest_index, entry_status) in part.iter_mut().enumerate() {
                if *entry_status == InQueue::NoAndAtMax {
                    *entry_status = InQueue::Yes;
                    self.queue.push_back(CompactCopsIndex { fst_index, rest_index });
                }
                if *entry_status == InQueue::YesAndAtMax {
                    *entry_status = InQueue::Yes;
                }
            }
        }
        self.pop_and_mark_popped()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }

    pub fn curr_max(&self) -> UTime {
        self.curr_max_nr_moves
    }
}

#[cfg(test)]
pub mod test {
    use super::*;

    #[test]
    fn bit_queue() {
        for length in 0..(2 * BITS) {
            let mut q = BitQueue::new(length).unwrap();
            {
                let mut i = 0;
                while q.pop_next_one().is_some() {
                    i += 1;
                }
                assert_eq!(length, i);
            }

            let is = [0, 1, 2, 4, 8, 16, 20, 30, 40, 50, 77, 127];
            for &i in is.iter().take_while(|&&i| i < length) {
                assert_eq!(false, q.set(i));
            }
            q.wrap_back();
            for &i in is.iter().take_while(|&&i| i < length) {
                assert_eq!(Some(i), q.pop_next_one());
            }
            assert_eq!(None, q.pop_next_one());
            q.wrap_back();
            assert_eq!(None, q.pop_next_one());
        }
    }

    #[test]
    fn robber_vs_cop_queue() {
        let cop_moves = CopConfigurations {
            nr_cops: 3,
            nr_map_vertices: 100,
            configurations: BTreeMap::from_iter([
                (0, (0..100).collect_vec()),
                (10, (10..100).collect_vec()),
                (90, (90..100).collect_vec()),
            ]),
        };
        let mut robber_queue = RobberStratQueue::new(&cop_moves).unwrap();
        let mut cops_queue = CopStratQueue::new(&cop_moves).unwrap();
        assert_eq!(robber_queue.len(), cops_queue.len());
        loop {
            let r_next = robber_queue.pop();
            let c_next = cops_queue.pop();
            assert_eq!(r_next, c_next);
            if r_next.is_none() {
                break;
            }
        }
    }
}
