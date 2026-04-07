//! this module does the bruteforce computation part for [`crate::graph::planar3d::fog`].
//! there are no longer two parties (cops vs robber), but just a single cleaning party.
//! we say a graph is _m-visibility-k-clenable_, if `k` cleaners with visibility `m` have
//! a strategy to remove all fog. should such a strategy exist, it will be found here.

use std::collections::{BinaryHeap, TryReserveError, VecDeque};

use bitvec::boxed::BitBox;
use itertools::{Itertools, izip};

use crate::graph::EdgeList;

use super::RawCops as RawCleaners;
use super::{MAX_COPS, Rules, thread_manager};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PackedCleaners(usize);

impl PackedCleaners {
    pub fn from_raw(nr_vertices: usize, positions: &[usize]) -> Self {
        debug_assert!(positions.is_sorted());
        let mut result = 0;
        for &cleaner in positions.iter().rev() {
            result *= nr_vertices;
            debug_assert!(cleaner < nr_vertices);
            result += cleaner;
        }
        Self(result)
    }

    pub fn into_raw(mut self, nr_cleaners: usize, nr_vertices: usize) -> RawCleaners {
        let mut raw = RawCleaners::uninit(nr_cleaners);
        for i in 0..nr_cleaners {
            raw[i] = self.0 % nr_vertices;
            self.0 /= nr_vertices;
        }
        debug_assert!(raw.is_sorted());
        raw
    }
}

/// this has the same size as just the [`bitvec::boxed::BitBox`] variant on it's own,
/// because the pointer pointing to the `BitBox` storage is guaranteed to never be null.
/// (and because this pointer is a fat pointer, e.g. also carries the length of the held storage.)
/// note: i just hope that the branch predictor is able to overcome any run-time overhead
/// caused by the match of `self` in each non-static method.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Fog {
    Smol(usize),
    Big(BitBox),
}

impl Fog {
    /// number of bits of a usize
    const MAX_SMOL: usize = usize::BITS as usize;

    fn new_filled(nr_vertices: usize) -> Self {
        match nr_vertices {
            Self::MAX_SMOL => Self::Smol(usize::MAX),
            0..Self::MAX_SMOL => Self::Smol((1 << nr_vertices) - 1),
            _ => {
                let mut data = vec![usize::MAX; nr_vertices.div_ceil(Self::MAX_SMOL)];
                let nr_in_last_int = nr_vertices % Self::MAX_SMOL;
                if nr_in_last_int > 0 {
                    *data.last_mut().unwrap() = (1 << nr_in_last_int) - 1;
                }
                let raw_box = Box::from(data);
                Self::Big(BitBox::from_boxed_slice(raw_box))
            },
        }
    }

    /// returns fog state where all vertices not in visibility range of `cleaners` are foggy.
    /// `visible` maps each vertex to the set of all vertices visible from that vertex.
    fn new_initial(cleaners: &[usize], visible: &EdgeList) -> Self {
        let mut result = Self::new_filled(visible.nr_vertices());
        for &cleaner in cleaners {
            for vis in visible.neighbors_of(cleaner) {
                result.mark_cleaned_at(vis);
            }
        }
        result
    }

    fn count_foggy(&self) -> usize {
        match self {
            Self::Smol(data) => data.count_ones() as usize,
            Self::Big(data) => data.count_ones(),
        }
    }

    #[allow(dead_code)]
    fn find_first_cleaned(&self) -> usize {
        match self {
            Self::Smol(data) => data.trailing_ones() as usize,
            Self::Big(data) => data.first_zero().unwrap_or(data.len()),
        }
    }

    fn mark_foggy_at(&mut self, v: usize) {
        match self {
            Self::Smol(data) => *data |= 1usize << v,
            Self::Big(data) => data.set(v, true),
        }
    }

    fn mark_cleaned_at(&mut self, v: usize) {
        match self {
            Self::Smol(data) => *data &= !(1usize << v),
            Self::Big(data) => data.set(v, false),
        }
    }

    fn is_foggy_at(&self, v: usize) -> bool {
        match self {
            Self::Smol(data) => (*data & (1usize << v)) != 0,
            Self::Big(data) => data[v],
        }
    }

    fn as_slice(&self) -> &bitvec::slice::BitSlice {
        match self {
            Self::Smol(data) => bitvec::slice::BitSlice::from_element(data),
            Self::Big(data) => data.as_bitslice(),
        }
    }

    fn iter_foggy(&self) -> impl Iterator<Item = usize> {
        self.as_slice().iter_ones()
    }

    /// returns [`std::cmp::Ordering::Equal`] if `self == other`,
    /// [`std::cmp::Ordering::Less`] if self is a subset of other and
    /// [`std::cmp::Ordering::Greater`] if self is a suberset of other.
    /// should none of these cases hold, [`None`] is returned.
    fn subset_ord(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering::{Equal, Greater, Less};
        let smol_ord = |a: usize, b: usize| {
            if a == b {
                Some(Equal)
            } else if a | b == a {
                Some(Greater)
            } else if a | b == b {
                Some(Less)
            } else {
                None
            }
        };
        match (self, other) {
            (&Self::Smol(a), &Self::Smol(b)) => smol_ord(a, b),
            (Self::Big(a), Self::Big(b)) => {
                let mut curr_ord = Equal;
                for (&a_part, &b_part) in izip!(a.as_raw_slice(), b.as_raw_slice()) {
                    let part_ord = smol_ord(a_part, b_part)?;
                    match (curr_ord, part_ord) {
                        (Equal, _) => curr_ord = part_ord,
                        (Less, Greater) | (Greater, Less) => return None,
                        _ => {},
                    }
                }
                Some(curr_ord)
            },
            _ => panic!("We can only compare fog states of the same underlying graph."),
        }
    }

    fn is_subset_of(&self, other: &Self) -> bool {
        self.subset_ord(other).is_some_and(|ord| ord.is_le())
    }

    /// assumes `self` to be the current fog state.
    /// computes the new fog state assuming cleaners moved from their current positions (unknown to us)
    /// to `new_positions`. parameter `visible` maps each vertex to the vertices vivible from there.
    fn compute_step(&self, edges: &EdgeList, visible: &EdgeList, new_positions: &[usize]) -> Self {
        let mut fog_after_step = self.clone();
        let mut out_of_reach = Fog::new_filled(edges.nr_vertices());
        for &cleaner in new_positions {
            for vis in visible.neighbors_of(cleaner) {
                out_of_reach.mark_cleaned_at(vis);
                fog_after_step.mark_cleaned_at(vis);
            }
        }
        let mut next_fog = fog_after_step.clone();
        for (v, neighs) in izip!(0.., edges.neighbors()) {
            if fog_after_step.is_foggy_at(v) {
                for neigh in neighs {
                    if out_of_reach.is_foggy_at(neigh) {
                        next_fog.mark_foggy_at(neigh);
                    }
                }
            }
        }
        next_fog
    }
}

/// convinience struct to have everything needed to perform a fog step computation.
struct FogStepComputation<'a> {
    edges: &'a EdgeList,
    visible: &'a EdgeList,
    fog_speed: isize,
    distances: Vec<isize>,
    queue: VecDeque<usize>,
}

impl<'a> FogStepComputation<'a> {
    fn new(edges: &'a EdgeList, visible: &'a EdgeList, fog_speed: isize) -> Self {
        debug_assert_eq!(edges.nr_vertices(), visible.nr_vertices());
        let distances = vec![0isize; edges.nr_vertices()];
        let queue = VecDeque::new();
        Self {
            edges,
            visible,
            fog_speed,
            distances,
            queue,
        }
    }

    fn compute_step(&mut self, curr_fog: &Fog, new_positions: &[usize]) -> Fog {
        debug_assert!(self.queue.is_empty());

        let mut fog_after_step = curr_fog.clone();
        let mut out_of_reach = Fog::new_filled(self.edges.nr_vertices());
        for &cleaner in new_positions {
            for vis in self.visible.neighbors_of(cleaner) {
                out_of_reach.mark_cleaned_at(vis);
                fog_after_step.mark_cleaned_at(vis);
            }
        }

        self.distances.fill(isize::MAX);
        for fog_v in fog_after_step.iter_foggy() {
            self.distances[fog_v] = 0;
            self.queue.push_back(fog_v);
        }
        let select = |v: usize, dists: &[isize], new_dist: isize| {
            new_dist <= self.fog_speed && out_of_reach.is_foggy_at(v) && dists[v] > new_dist
        };
        self.edges
            .calc_distances_to_with(&mut self.distances, select, &mut self.queue);

        let mut next_fog = fog_after_step.clone();
        for (v, &dist) in izip!(0.., &self.distances) {
            if dist != isize::MAX {
                next_fog.mark_foggy_at(v);
            }
        }

        if self.fog_speed == 1 {
            debug_assert_eq!(
                next_fog,
                curr_fog.compute_step(self.edges, self.visible, new_positions)
            );
        }

        next_fog
    }
}

struct FogState {
    fog: Fog,
    prev: PackedCleaners,
}

/// this a a game state that has no information where it comes from.
/// -> only useful in combination whith [`FogState`]
#[derive(PartialEq, Eq)]
struct QueueEntry {
    fog: Fog,
    cleaners: PackedCleaners,
}

impl std::cmp::Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // note: this is function is called when we are stored in a max-heap.
        // -> values that are larger should be better to investigate further.
        // -> smaller number of foggy vertices is good to prioritize.
        let nr_foggy = other.fog.count_foggy().cmp(&self.fog.count_foggy());
        let fog = other.fog.cmp(&self.fog);
        let cleaners = other.cleaners.cmp(&self.cleaners);

        // primarily oder by nr_foggy -> this speeds up the algorithm (if a solution exists).
        nr_foggy.then(fog).then(cleaners)
    }
}
impl std::cmp::PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// for a given vertex `v`, the returned "neighbors" of `v` are
/// the vertices with distance `<= max_dist` in `edges`.
/// note: this means `v` always "neighbors" itself.
fn compute_near_vertices(edges: &EdgeList, max_dist: usize) -> EdgeList {
    let max_dist = max_dist.min(edges.nr_vertices());
    let visible_others = edges.pow(max_dist);
    let all_visible = izip!(0..edges.nr_vertices(), visible_others.neighbors())
        .map(|(v, vis)| std::iter::once(v).chain(vis));

    let max_dregree = visible_others.max_degree() + 1;
    EdgeList::from_iter(all_visible, max_dregree)
}

pub struct FogSolution {
    /// nonempty <=> cleanable.
    pub sequence: Vec<PackedCleaners>,
    pub nr_vertices: usize,
    pub nr_cleaners: usize,
    pub visibility: usize,
    pub fog_speed: isize,
    /// if graph is cleanable with [`Self::sequence`],
    /// is it also cleanable with the reverse of [`Self::sequence`]?
    pub works_in_reverse: bool,
    /// assuming graph is cleanable, is [`Self::sequence`] a shortest possible sequence?
    pub is_best_solution: bool,
}

impl FogSolution {
    pub fn is_cleanable(&self) -> bool {
        !self.sequence.is_empty()
    }

    pub fn unpack(&self, packed: PackedCleaners) -> RawCleaners {
        packed.into_raw(self.nr_cleaners, self.nr_vertices)
    }

    pub fn iter_unpacked(&self) -> impl ExactSizeIterator<Item = RawCleaners> {
        assert!(self.is_cleanable());
        self.sequence.iter().map(|&cs| self.unpack(cs))
    }
}

trait Queue<T> {
    fn push(&mut self, new: T);
    fn pop(&mut self) -> Option<T>;
    fn try_reserve(&mut self, additional: usize) -> Result<(), TryReserveError>;
    fn len(&self) -> usize;
    const IS_FIFO: bool;
}
impl<T> Queue<T> for VecDeque<T> {
    fn push(&mut self, new: T) {
        self.push_back(new);
    }
    fn pop(&mut self) -> Option<T> {
        self.pop_front()
    }
    fn try_reserve(&mut self, additional: usize) -> Result<(), TryReserveError> {
        VecDeque::try_reserve(self, additional)
    }
    fn len(&self) -> usize {
        VecDeque::len(self)
    }
    const IS_FIFO: bool = true;
}
impl<T: Ord> Queue<T> for BinaryHeap<T> {
    fn push(&mut self, new: T) {
        BinaryHeap::push(self, new);
    }
    fn pop(&mut self) -> Option<T> {
        BinaryHeap::pop(self)
    }
    fn try_reserve(&mut self, additional: usize) -> Result<(), TryReserveError> {
        BinaryHeap::try_reserve(self, additional)
    }
    fn len(&self) -> usize {
        BinaryHeap::len(self)
    }
    const IS_FIFO: bool = false;
}

/// unlike for the other bruteforce algorithms, no care is taken to ensure that out-of-memory errors are caught.
/// this is because this algorithm is continuously allocating small chunks. first, this is very anoying to track
/// and second not even guaranteed to be recoverable if something goes wrong anyway.
fn compute_fog_strategy<R: Rules, Q: Queue<QueueEntry> + Default>(
    rules: R,
    visibility: usize,
    nr_cleaners: usize,
    fog_speed: isize,
    edges: EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<FogSolution, String> {
    // if the queue keeps the order we inserted entries in,
    // we do a breath first search, so game states reached in fewer steps are handled first.
    // otherwise the priorisation is dependent on however the queue decides to do it.
    // in that other case, we can thus not guarantee to find a solution with the fewest possible steps.
    let find_best = Q::IS_FIFO;

    if nr_cleaners > MAX_COPS {
        let msg = format!("Rechnung kann für höchstens {MAX_COPS} Cleaner durchgeführt werden.");
        return Err(msg);
    }
    let nr_vertices = edges.nr_vertices();
    if nr_vertices.checked_pow(nr_cleaners as u32).is_none() {
        return Err("Cleanerpositionen passen nicht in usize.".to_string());
    }
    if nr_vertices == 0 {
        return Err("Graph darf nicht leer sein.".to_string());
    }
    if !find_best && !edges.is_connected() {
        return Err("Graph muss zusammenhängend sein".to_string());
    }

    let mut sol = FogSolution {
        sequence: Vec::new(),
        nr_vertices,
        nr_cleaners,
        visibility,
        fog_speed,
        works_in_reverse: false,
        is_best_solution: find_best,
    };

    manager.update("initialisiere Variablen")?;
    // `visible.neighbors_of(v)` are all vertices in sight of a cleaner standing at vertex `v`.
    let visible = compute_near_vertices(&edges, visibility);

    let mut fog_step_data = FogStepComputation::new(&edges, &visible, fog_speed);

    // stores for each cleaner state, which interesting fog states are possible to reach.
    type States = std::collections::HashMap<PackedCleaners, Vec<FogState>>;
    let mut states = States::new();
    // stores each interesting game state with unexplored moves.
    let mut queue = Q::default();

    // enters an initial state into both states and queue.
    let add_initial = |unpacked: &[usize], states: &mut States, queue: &mut Q| {
        let cleaners = PackedCleaners::from_raw(nr_vertices, unpacked);
        let fog = Fog::new_initial(unpacked, &visible);
        queue.push(QueueEntry { fog: fog.clone(), cleaners });
        states.insert(cleaners, vec![FogState { fog, prev: cleaners }]);
    };
    if find_best {
        // we want to find the best solution (e.g. shortest move sequence),
        // so we actually need to consider every possible initial state.
        let sym = crate::graph::NoSymmetry::new(nr_vertices);
        let positions = super::CopConfigurations::new(&edges, &sym, nr_cleaners, manager)?;
        let nr_initial_states = positions.nr_configurations();
        if states.try_reserve(nr_initial_states).is_err() {
            return Err("nicht genug Speicher für States.".to_string());
        }
        if queue.try_reserve(nr_initial_states).is_err() {
            return Err("nicht genug Speicher für Queue.".to_string());
        }
        for (i, position_index) in izip!(0.., positions.all_positions()) {
            if i % 4096 == 0 {
                let percent = 100.0 * (i as f32) / (nr_initial_states as f32);
                manager.update(format!("initialisiere Cleanerstates: {percent:.2}%"))?;
            }
            let initial_unpacked = positions.eager_unpack(position_index).sorted();
            add_initial(&initial_unpacked, &mut states, &mut queue);
        }
    } else {
        // we want to find a solution quickly and the graph is connected
        // -> it suffices to start with any arbitrary cleaner state and the other (needed ones)
        // are discovered in the main loop.
        let initial_unpacked = RawCleaners::from_iter(std::iter::repeat_n(0, nr_cleaners));
        add_initial(&initial_unpacked, &mut states, &mut queue);
    }

    // information for manager
    let mut most_cleaned_so_far = 0;
    let mut time_until_log_refresh: usize = 1;

    // the loop returns the first found cleaner state, that has an empty fog state associated with it.
    let final_packed_cleaners = loop {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            let len = queue.len();
            manager.update(format!(
                "berechne Reinigungsstrategie: {len} in Queue, \
                max {most_cleaned_so_far}/{nr_vertices} Knoten sauber"
            ))?;
            time_until_log_refresh = 1_000;
        }

        let Some(curr) = queue.pop() else {
            debug_assert!(!sol.is_cleanable());
            return Ok(sol);
        };
        let curr_nr_foggy = curr.fog.count_foggy();
        most_cleaned_so_far = most_cleaned_so_far.max(nr_vertices - curr_nr_foggy);
        if curr_nr_foggy == 0 {
            break curr.cleaners;
        }

        let curr_cleaners = sol.unpack(curr.cleaners);
        let all_raw_next_cleaners = rules.raw_cop_moves_from(&edges, curr_cleaners);

        for next_cleaners in all_raw_next_cleaners.map(RawCleaners::sorted) {
            let next_fog = fog_step_data.compute_step(&curr.fog, &next_cleaners);
            let next_packed_cleaners = PackedCleaners::from_raw(nr_vertices, &next_cleaners);

            debug_assert!(!find_best || states.contains_key(&next_packed_cleaners));
            // don't actually initialize the fog states, because this is done below anyway.
            let next_states = states.entry(next_packed_cleaners).or_default();
            // next_fog is only interesting if we don't already know that we can reach
            // a fog state with (at least) all the vertices cleaned in next_fog also cleaned.
            if next_states.iter().any(|old| old.fog.is_subset_of(&next_fog)) {
                continue;
            }
            // the new state is pushed to the back. -> if we compute the shortest solution,
            //   next_states is ordered by in how many rounds the states can be reached.
            next_states.push(FogState {
                fog: next_fog.clone(),
                prev: curr.cleaners,
            });
            queue.push(QueueEntry {
                fog: next_fog,
                cleaners: next_packed_cleaners,
            });
        }
    };
    drop(queue);

    manager.update("schreibe Lösung")?;
    let mut reversed_sequence = vec![final_packed_cleaners];
    let mut curr_packed_cleaners = final_packed_cleaners;
    let is_cleaned = |state: &&FogState| state.fog.count_foggy() == 0;
    let mut curr_state = states[&final_packed_cleaners].iter().find(is_cleaned).unwrap();
    loop {
        let prev_packed_cleaners = curr_state.prev;
        let curr_cleaners = sol.unpack(curr_packed_cleaners);
        if curr_state.fog == Fog::new_initial(&curr_cleaners, &visible) {
            debug_assert!(!find_best || curr_packed_cleaners == prev_packed_cleaners);
            break;
        }
        assert_ne!(curr_packed_cleaners, prev_packed_cleaners);
        reversed_sequence.push(prev_packed_cleaners);

        let is_prev_fog = |state: &&FogState| {
            fog_step_data.compute_step(&state.fog, &curr_cleaners) == curr_state.fog
        };
        // note that if we find a best solution,
        // the fog states of a fixed cleaner state are stored in chronological order.
        // we thus take the earliest state with consistent fog.
        let prev_state = states[&prev_packed_cleaners].iter().find(is_prev_fog).unwrap();

        curr_packed_cleaners = prev_packed_cleaners;
        curr_state = prev_state;
    }

    sol.sequence = reversed_sequence;
    sol.works_in_reverse = verify_sequence(&rules, &sol, &edges).is_ok();
    // actually store sequence in correct order
    sol.sequence.reverse();
    verify_sequence(&rules, &sol, &edges)?;

    Ok(sol)
}

/// returns the strategy with the shortest number of moves, should it exist.
pub fn compute_best_fog_strategy<R: Rules>(
    rules: R,
    visibility: usize,
    nr_cleaners: usize,
    fog_speed: isize,
    edges: EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<FogSolution, String> {
    type Q = VecDeque<QueueEntry>;
    compute_fog_strategy::<R, Q>(rules, visibility, nr_cleaners, fog_speed, edges, manager)
}

/// if a strategy exists, an unspecified working strategy is returned.
pub fn compute_any_fog_strategy<R: Rules>(
    rules: R,
    visibility: usize,
    nr_cleaners: usize,
    fog_speed: isize,
    edges: EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<FogSolution, String> {
    type Q = BinaryHeap<QueueEntry>;
    compute_fog_strategy::<R, Q>(rules, visibility, nr_cleaners, fog_speed, edges, manager)
}

/// tries to walk the solution and fails if something fishy happens while doing so.
fn verify_sequence<R: Rules>(rules: &R, sol: &FogSolution, edges: &EdgeList) -> Result<(), String> {
    if !sol.is_cleanable() {
        return Err("Strategie existiert nicht.".to_string());
    };
    assert_eq!(sol.nr_vertices, edges.nr_vertices());
    let visible = compute_near_vertices(edges, sol.visibility);
    let fog_spread = compute_near_vertices(edges, sol.fog_speed as usize);

    let mut fog_step_data = FogStepComputation::new(edges, &visible, sol.fog_speed);

    let mut sequence = sol.iter_unpacked();
    let mut prev_cleaners = sequence.next().unwrap();
    let mut prev_fog = Fog::new_initial(&prev_cleaners, &visible);
    for curr_cleaners in sequence {
        if !rules
            .raw_cop_moves_from(edges, prev_cleaners)
            .map(RawCleaners::sorted)
            .contains(&curr_cleaners)
        {
            return Err(format!(
                "Komme nicht von {prev_cleaners:?} zu {curr_cleaners:?} in einer Runde.",
            ));
        }
        let curr_fog = fog_step_data.compute_step(&prev_fog, &curr_cleaners);
        for v in 0..sol.nr_vertices {
            let v_close_to_fog = fog_spread.neighbors_of(v).any(|n| prev_fog.is_foggy_at(n));
            if curr_fog.is_foggy_at(v) && !v_close_to_fog {
                return Err("Nebel breitet sich zu schnell aus.".to_string());
            }
            let v_visible = curr_cleaners.iter().any(|&c| visible.neighbors_of(c).contains(&v));
            if prev_fog.is_foggy_at(v) && !curr_fog.is_foggy_at(v) && !v_visible {
                return Err("Nebel verschwindet von alleine.".to_string());
            }
        }
        prev_fog = curr_fog;
        prev_cleaners = curr_cleaners;
    }

    let still_foggy = (0..sol.nr_vertices)
        .filter(|&v| prev_fog.is_foggy_at(v))
        .collect_vec();
    if !still_foggy.is_empty() {
        return Err(format!("Diese Knoten {still_foggy:?} sind noch nebelig."));
    }
    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn fog_basics() {
        assert_eq!(std::mem::size_of::<Fog>(), std::mem::size_of::<BitBox>());

        let _ = Fog::new_filled(0);
        for nr_vertices in [10, 32, 41, 64, 500, 1024, 2000] {
            let full_fog = Fog::new_filled(nr_vertices);
            assert_eq!(nr_vertices, full_fog.count_foggy() as usize);
            assert_eq!(nr_vertices, full_fog.find_first_cleaned());

            let mut fog = full_fog.clone();

            fog.mark_cleaned_at(5);
            assert_eq!(fog.find_first_cleaned(), 5);
            fog.mark_cleaned_at(3);
            assert_eq!(fog.find_first_cleaned(), 3);
            assert_eq!(fog.count_foggy() as usize, nr_vertices - 2);

            assert_eq!(fog.subset_ord(&full_fog), Some(std::cmp::Ordering::Less));
            assert_eq!(full_fog.subset_ord(&fog), Some(std::cmp::Ordering::Greater));
            assert!(!full_fog.is_subset_of(&fog));
            assert!(fog.is_subset_of(&full_fog));

            fog.mark_foggy_at(3);
            assert_eq!(fog.find_first_cleaned(), 5);
            fog.mark_foggy_at(5);
            assert_eq!(fog.find_first_cleaned(), nr_vertices);
            assert_eq!(fog.subset_ord(&full_fog), Some(std::cmp::Ordering::Equal));
        }
    }

    fn compute_best(
        edges: EdgeList,
        vis: usize,
        speed: isize,
        nr_cleaners: usize,
    ) -> Result<FogSolution, String> {
        let rules = super::super::GeneralEagerCops(u32::MAX);
        let (_, manager) = thread_manager::build_managers();
        let any =
            compute_any_fog_strategy(rules, vis, nr_cleaners, speed, edges.clone(), &manager)?;
        let best =
            compute_best_fog_strategy(rules, vis, nr_cleaners, speed, edges.clone(), &manager)?;
        assert_eq!(any.is_cleanable(), best.is_cleanable());
        assert!(best.sequence.len() <= any.sequence.len());
        Ok(best)
    }

    fn general_cleaning_number(edges: EdgeList, vis: usize, speed: isize) -> Result<usize, String> {
        for nr_cleaners in 1.. {
            if compute_best(edges.clone(), vis, speed, nr_cleaners)?.is_cleanable() {
                return Ok(nr_cleaners);
            }
        }
        unreachable!()
    }

    #[test]
    fn test_cleaning_numbers() {
        let circle = |n: usize| -> EdgeList {
            let edges = (0..n).map(|v| [(v + n - 1) % n, (v + n + 1) % n].into_iter());
            EdgeList::from_iter(edges, 2)
        };
        let complete = |n: usize| -> EdgeList {
            let edges = (0..n).map(|v| (0..v).chain((v + 1)..n));
            EdgeList::from_iter(edges, n - 1)
        };
        let cleaning_number = |edges, vis| general_cleaning_number(edges, vis, 1);

        assert_eq!(cleaning_number(circle(5), 0), Ok(2));
        assert_eq!(cleaning_number(circle(5), 1), Ok(2));
        assert_eq!(cleaning_number(circle(5), 2), Ok(1));
        assert_eq!(cleaning_number(circle(25), 2), Ok(2));
        assert_eq!(cleaning_number(circle(25), 11), Ok(2));
        assert_eq!(cleaning_number(circle(25), 12), Ok(1));
        assert_eq!(cleaning_number(complete(4), 0), Ok(2));
        assert_eq!(cleaning_number(complete(5), 0), Ok(3));
        assert_eq!(cleaning_number(complete(6), 0), Ok(3));
        assert_eq!(cleaning_number(complete(10), 1), Ok(1));

        // example was found by alexander. this exposed a (now fixed) bug
        let complete_with_rings = |n: usize| -> EdgeList {
            let mut edges = complete(n);
            let ring_a = (0..(n + 1)).map(|_| edges.add_vertex()).collect_vec();
            let ring_b = (0..(n + 1)).map(|_| edges.add_vertex()).collect_vec();
            for ring in [&ring_a, &ring_b] {
                edges.add_path_edges(ring.iter().copied().chain(Some(ring[0])));
            }
            for (i, va, vb) in izip!(0.., ring_a, ring_b) {
                edges.add_edge(va, i % n);
                edges.add_edge(vb, (i + n / 2) % n);
            }
            edges
        };
        for n in 6..12 {
            let strat = compute_best(complete_with_rings(n), 1, 1, 1).unwrap();
            assert!(strat.is_cleanable());
            assert!(strat.sequence.len() >= 6 * (n - 2));
            assert!(strat.sequence.len() <= 6 * n);
        }

        use crate::graph::{planar3d::Embedding3D, shape::Shape};
        let from_shape = |shape, res| Embedding3D::new_map_from(&shape, res).into_edges();
        assert_eq!(cleaning_number(from_shape(Shape::TriangGrid, 4), 1), Ok(1));
        // one cleaner guards a fixed edge, the other cleaners walk the six remaining vertices in parallel.
        assert_eq!(cleaning_number(from_shape(Shape::Cube, 0), 0), Ok(3));
        // two cleaners at opposing sides already see most vertices without moving
        assert_eq!(cleaning_number(from_shape(Shape::Icosahedron, 1), 2), Ok(2));

        // cleaning numbers for higher fog speeds (all these are independent of speed)
        assert_eq!(general_cleaning_number(circle(5), 0, 2), Ok(2));
        assert_eq!(general_cleaning_number(circle(5), 0, 100), Ok(2));
        assert_eq!(general_cleaning_number(circle(5), 1, 2), Ok(2));
        assert_eq!(general_cleaning_number(circle(5), 1, 100), Ok(2));
        assert_eq!(general_cleaning_number(circle(5), 2, 2), Ok(1));
        assert_eq!(general_cleaning_number(circle(5), 2, 100), Ok(1));
        assert_eq!(general_cleaning_number(circle(25), 2, 2), Ok(2));
        assert_eq!(general_cleaning_number(circle(25), 2, 100), Ok(2));
        // increasing speed increases cleaning number
        assert_eq!(
            general_cleaning_number(complete_with_rings(10), 1, 2),
            Ok(2)
        );
    }
}
