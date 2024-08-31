use super::*;

struct BitQueue {
    data: Vec<usize>,
    capacity: usize,

    data_iter: usize,
    data_curr: usize,
}

const BITS: usize = usize::BITS as usize;

impl BitQueue {
    pub fn new(length: usize) -> Option<Self> {
        let mut data = Vec::new();
        let data_len = (length + BITS - 1) / BITS;
        data.try_reserve_exact(data_len).ok()?;
        data.resize(data_len, usize::MAX);

        let last_count = length % BITS;
        if last_count != 0 {
            let last_val = (1usize << last_count) - 1;
            data[data_len - 1] = last_val;
        }
        debug_assert_eq!(
            data.iter().map(|x| x.count_ones() as usize).sum::<usize>(),
            length
        );

        let data_curr = *data.first().unwrap_or(&0);
        Some(Self {
            data,
            capacity: length,
            data_iter: 0,
            data_curr,
        })
    }

    pub fn wrap_back(&mut self) {
        self.data_iter = 0;
        self.data_curr = *self.data.first().unwrap_or(&0);
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

pub struct RobberStratQueue {
    bit_queues: Vec<queues::BitQueue>,
    firsts: Vec<usize>,
    first_index: usize,
    length: usize,
    rounds_complete: usize,
}

impl RobberStratQueue {
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
            data.resize(nr_entries, InQueue::No);
            let old = contained.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        let mut queue = VecDeque::new();
        queue.try_reserve(cop_moves.nr_configurations()).ok()?;
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
