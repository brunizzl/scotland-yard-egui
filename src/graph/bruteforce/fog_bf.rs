//! this module does the bruteforce computation part for [`crate::graph::planar3d::fog`].
//! there are no longer two parties (cops vs robber), but just a single cleaning party.
//! we say a graph is _m-visibility-k-clenable_, if `k` cleaners with visibility `m` have
//! a strategy to remove all fog. should such a strategy exist, it will be found here.

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
    fn new_filled(nr_vertices: usize) -> Self {
        const MAX_BITS_PER_INT: usize = usize::MAX.count_ones() as usize;
        match nr_vertices {
            MAX_BITS_PER_INT => Self::Smol(usize::MAX),
            0..MAX_BITS_PER_INT => Self::Smol((1 << nr_vertices) - 1),
            _ => {
                let mut data = vec![usize::MAX; nr_vertices.div_ceil(MAX_BITS_PER_INT)];
                let nr_in_last_int = nr_vertices % MAX_BITS_PER_INT;
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
                result.mark_cleaned(vis);
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

    fn count_cleaned(&self, nr_vertices: usize) -> usize {
        nr_vertices - self.count_foggy()
    }

    #[allow(dead_code)]
    fn first_cleaned(&self) -> usize {
        match self {
            Self::Smol(data) => data.trailing_ones() as usize,
            Self::Big(data) => data.first_zero().unwrap_or(data.len()),
        }
    }

    fn mark_foggy(&mut self, v: usize) {
        match self {
            Self::Smol(data) => *data |= 1usize << v,
            Self::Big(data) => data.set(v, true),
        }
    }

    fn mark_cleaned(&mut self, v: usize) {
        match self {
            Self::Smol(data) => *data &= !(1usize << v),
            Self::Big(data) => data.set(v, false),
        }
    }

    fn is_foggy(&self, v: usize) -> bool {
        match self {
            Self::Smol(data) => (*data & (1usize << v)) != 0,
            Self::Big(data) => data[v],
        }
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
                out_of_reach.mark_cleaned(vis);
                fog_after_step.mark_cleaned(vis);
            }
        }
        let mut next_fog = fog_after_step.clone();
        for (v, neighs) in izip!(0.., edges.neighbors()) {
            if fog_after_step.is_foggy(v) {
                for neigh in neighs {
                    if out_of_reach.is_foggy(neigh) {
                        next_fog.mark_foggy(neigh);
                    }
                }
            }
        }
        next_fog
    }
}

#[derive(Debug, Clone)]
struct FogState {
    fog: Fog,
    prev: PackedCleaners,
}

/// this is meant to be inserted into a max-heap.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct QueueEntry {
    /// this is the first held data,
    /// so that an entry with more cleaned vertices is larger, e.g. is removed from the max-heap earlier.
    /// (this is also the reason why we count the number of cleaned, not the number of foggy vertices)
    nr_cleaned: usize,
    cleaners: PackedCleaners,
}

/// for a given vertex `v`, the returned "neighbors" of `v` are
/// the vertices with distance `<= visibility` in `edges`.
/// note: this means `v` always "neighbors" itself.
fn compute_visible(edges: &EdgeList, visibility: usize) -> EdgeList {
    let visible_others = edges.pow(visibility);
    let all_visible = izip!(0..edges.nr_vertices(), visible_others.neighbors())
        .map(|(v, vis)| std::iter::once(v).chain(vis));

    let max_dregree = visible_others.max_degree() + 1;
    EdgeList::from_iter(all_visible, max_dregree)
}

pub struct CleaningSequence {
    pub sequence: Vec<PackedCleaners>,
    pub nr_vertices: usize,
    pub nr_cleaners: usize,
    pub visibility: usize,
}

pub enum FogSolution {
    Cleanable(CleaningSequence),
    NotCleanable,
}

/// unlike for the other bruteforce algorithms, no care is taken to ensure that out-of-memory errors are caught.
/// this is because this algorithm is continuously allocating small chunks. first, this is very anoying to track
/// and second not even guaranteed to be recoverable if something goes wrong anyway.
pub fn compute_cleaning_strategy<R: Rules>(
    rules: R,
    visibility: usize,
    nr_cleaners: usize,
    edges: EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<FogSolution, String> {
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
    if !edges.is_connected() {
        return Err("Graph muss zusammenhängend sein".to_string());
    }

    manager.update("initialisiere Variablen")?;
    let visible = compute_visible(&edges, visibility);

    // stores for each cleaner state, which interesting fog states are possible to reach
    let mut states = std::collections::HashMap::new();
    let mut queue = std::collections::BinaryHeap::new();
    {
        let initial_cleaners = RawCleaners::from_iter(std::iter::repeat_n(0, nr_cleaners));
        let compact = PackedCleaners::from_raw(nr_vertices, &initial_cleaners);
        let fog = Fog::new_initial(&initial_cleaners, &visible);
        let nr_cleaned = fog.count_cleaned(nr_vertices);
        states.insert(compact, vec![FogState { fog, prev: compact }]);
        queue.push(QueueEntry { nr_cleaned, cleaners: compact });
    }

    // because a queue entry only stores how many vertices are foggy,
    // and because it may be that the same cleaner state holds multiple
    // fog states with the same number of cleaned vertices,
    // we don't always know which fog state is the exact one entered into the queue.
    // solution: do the update for every such fog state.
    // for borrow rules reasons, we need to clone these fog states into this variable.
    let mut current_fogs = Vec::new();

    let mut most_cleaned_so_far = 0;
    let mut time_until_log_refresh: usize = 1;

    let final_compact_cleaners = loop {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            manager.update(format!(
                "berechne Reinigungsstrategie: {} in Queue, max {}/{} Knoten sauber",
                queue.len(),
                most_cleaned_so_far,
                nr_vertices,
            ))?;
            time_until_log_refresh = 10_000;
        }

        let Some(curr) = queue.pop() else {
            return Ok(FogSolution::NotCleanable);
        };
        debug_assert!(curr.nr_cleaned <= nr_vertices);
        most_cleaned_so_far = most_cleaned_so_far.max(curr.nr_cleaned);
        if curr.nr_cleaned == nr_vertices {
            break curr.cleaners;
        }

        debug_assert!(current_fogs.is_empty());
        for state in states[&curr.cleaners].iter() {
            if state.fog.count_cleaned(nr_vertices) == curr.nr_cleaned {
                current_fogs.push(state.fog.clone());
            }
        }
        let curr_cleaners = curr.cleaners.into_raw(nr_cleaners, nr_vertices);
        for curr_fog in current_fogs.drain(..) {
            for mut next_cleaners in rules.raw_cop_moves_from(&edges, curr_cleaners) {
                next_cleaners.sort();
                let next_fog = curr_fog.compute_step(&edges, &visible, &next_cleaners);
                let next_compact_cleaners = PackedCleaners::from_raw(nr_vertices, &next_cleaners);
                // don't actually initialize the fog states, because this is done below anyway.
                let next_states = states.entry(next_compact_cleaners).or_insert_with(Vec::new);
                // next_fog is only interesting if we don't already know that we can reach
                // a fog state with (at least) all the vertices cleaned in next_fog also cleaned.
                if next_states.iter().any(|state| state.fog.is_subset_of(&next_fog)) {
                    continue;
                }
                let next_nr_cleared = next_fog.count_cleaned(nr_vertices);
                next_states.push(FogState {
                    fog: next_fog,
                    prev: curr.cleaners,
                });
                queue.push(QueueEntry {
                    nr_cleaned: next_nr_cleared,
                    cleaners: next_compact_cleaners,
                });
            }
        }
    };
    drop(queue);

    manager.update("schreibe Lösung")?;
    let mut cleaning_sequence = vec![final_compact_cleaners];
    let mut curr_compact_cleaners = final_compact_cleaners;
    let is_cleaned = |state: &&FogState| state.fog.count_cleaned(nr_vertices) == nr_vertices;
    let mut curr_state = states[&final_compact_cleaners].iter().find(is_cleaned).unwrap();
    loop {
        let curr_cleaners = curr_compact_cleaners.into_raw(nr_cleaners, nr_vertices);
        if curr_state.fog == Fog::new_initial(&curr_cleaners, &visible) {
            break;
        }
        assert_ne!(curr_compact_cleaners, curr_state.prev);
        let prev_states = &states[&curr_state.prev];
        let is_prev = |state: &&FogState| {
            state.fog.compute_step(&edges, &visible, &curr_cleaners) == curr_state.fog
        };
        let prev_state = prev_states.iter().find(is_prev).unwrap();
        cleaning_sequence.push(curr_state.prev);
        curr_compact_cleaners = curr_state.prev;
        curr_state = prev_state;
    }

    cleaning_sequence.reverse();
    let seq = CleaningSequence {
        sequence: cleaning_sequence,
        nr_vertices,
        nr_cleaners,
        visibility,
    };
    debug_assert!(verify_solution(rules, &seq, &edges).is_ok());
    Ok(FogSolution::Cleanable(seq))
}

/// tries to walk the solution and fails if something fishy happens while doing so.
fn verify_solution<R: Rules>(
    rules: R,
    seq: &CleaningSequence,
    edges: &EdgeList,
) -> Result<(), String> {
    assert_eq!(seq.nr_vertices, edges.nr_vertices());
    let visible = compute_visible(edges, seq.visibility);

    let mut prev_cleaners = seq.sequence[0].into_raw(seq.nr_cleaners, seq.nr_vertices);
    let mut prev_fog = Fog::new_initial(&prev_cleaners, &visible);
    for &curr_compact in &seq.sequence[1..] {
        let curr_cleaners = curr_compact.into_raw(seq.nr_cleaners, seq.nr_vertices);
        if !rules.raw_cop_moves_from(edges, prev_cleaners).any(|mut step| {
            step.sort();
            step == curr_cleaners
        }) {
            return Err(format!(
                "Cannot walk from {:?} to {:?} in a single round.",
                &prev_cleaners[..],
                &curr_cleaners[..]
            ));
        }
        let curr_fog = prev_fog.compute_step(edges, &visible, &curr_cleaners);
        for v in 0..seq.nr_vertices {
            if curr_fog.is_foggy(v) && edges.neighbors_of(v).all(|n| !prev_fog.is_foggy(n)) {
                return Err("fog spread faster than one unit per step".to_string());
            }
            let v_visible = curr_cleaners.iter().any(|&c| visible.neighbors_of(c).contains(&v));
            if prev_fog.is_foggy(v) && !curr_fog.is_foggy(v) && !v_visible {
                return Err(format!("vertex {v} became fog-free while not visible"));
            }
        }
        prev_fog = curr_fog;
        prev_cleaners = curr_cleaners;
    }

    let still_foggy = (0..seq.nr_vertices).filter(|&v| prev_fog.is_foggy(v)).collect_vec();
    if !still_foggy.is_empty() {
        return Err(format!("these vertices {still_foggy:?} are not cleaned."));
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
            assert_eq!(nr_vertices, full_fog.count_foggy());
            assert_eq!(nr_vertices, full_fog.first_cleaned());

            let mut fog = full_fog.clone();

            fog.mark_cleaned(5);
            assert_eq!(fog.first_cleaned(), 5);
            fog.mark_cleaned(3);
            assert_eq!(fog.first_cleaned(), 3);
            assert_eq!(fog.count_foggy(), nr_vertices - 2);

            assert_eq!(fog.subset_ord(&full_fog), Some(std::cmp::Ordering::Less));
            assert_eq!(full_fog.subset_ord(&fog), Some(std::cmp::Ordering::Greater));
            assert!(!full_fog.is_subset_of(&fog));
            assert!(fog.is_subset_of(&full_fog));

            fog.mark_foggy(3);
            assert_eq!(fog.first_cleaned(), 5);
            fog.mark_foggy(5);
            assert_eq!(fog.first_cleaned(), nr_vertices);
            assert_eq!(fog.subset_ord(&full_fog), Some(std::cmp::Ordering::Equal));
        }
    }

    fn cleaning_number(edges: EdgeList, visibility: usize) -> Result<usize, String> {
        let rules = crate::graph::bruteforce::rules::GeneralEagerCops(u32::MAX);
        let (_, manager) = thread_manager::build_managers();
        for nr_cleaners in 1.. {
            let result =
                compute_cleaning_strategy(rules, visibility, nr_cleaners, edges.clone(), &manager)?;
            if matches!(result, FogSolution::Cleanable(_)) {
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
        assert_eq!(cleaning_number(circle(5), 0), Ok(2));
        assert_eq!(cleaning_number(circle(5), 1), Ok(2));
        assert_eq!(cleaning_number(circle(5), 2), Ok(1));
        assert_eq!(cleaning_number(circle(25), 2), Ok(2));
        assert_eq!(cleaning_number(circle(25), 11), Ok(2));
        assert_eq!(cleaning_number(circle(25), 12), Ok(1));

        use crate::graph::{planar3d::Embedding3D, shape::Shape};
        let from_shape = |shape, res| Embedding3D::new_map_from(&shape, res).into_edges();
        assert_eq!(cleaning_number(from_shape(Shape::TriangGrid, 4), 1), Ok(1));
        // one cleaner guards a fixed edge, the other cleaners walk the six remaining vertices in parallel.
        assert_eq!(cleaning_number(from_shape(Shape::Cube, 0), 0), Ok(3));
        // two cleaners at opposing sides already see most vertices without moving
        assert_eq!(cleaning_number(from_shape(Shape::Icosahedron, 1), 2), Ok(2));
    }
}
