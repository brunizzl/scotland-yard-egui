//! this module does the bruteforce computation part for [`crate::graph::planar3d::fog`].
//! there are no longer two parties (cops vs robber), but just a single cleaning party.
//! we say a graph is _m-visibility-k-clenable_, if `k` cleaners with visibility `m` have
//! a strategy to remove all fog. should such a strategy exist, it will be found here.

use bitvec::boxed::BitBox;
use itertools::izip;

use crate::graph::EdgeList;

use super::{MAX_COPS, RawCops, Rules, thread_manager};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CompactCleaners(usize);

impl CompactCleaners {
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

    pub fn into_raw(mut self, nr_cleaners: usize, nr_vertices: usize) -> RawCops {
        let mut raw = RawCops::uninit(nr_cleaners);
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
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Fog {
    Smol(usize),
    Big(BitBox),
}

impl Fog {
    fn new_filled(nr_vertices: usize) -> Self {
        const MAX_BITS_PER_INT: usize = std::mem::size_of::<usize>() * 8;
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

    fn count_foggy(&self) -> usize {
        match self {
            Self::Smol(data) => data.count_ones() as usize,
            Self::Big(data) => data.count_ones(),
        }
    }

    fn count_cleaned(&self, nr_vertices: usize) -> usize {
        nr_vertices - self.count_foggy()
    }

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
    prev: CompactCleaners,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
struct QueueEntry {
    nr_cleaned: usize,
    cleaners: CompactCleaners,
}

pub struct CleaningSequence(pub Vec<CompactCleaners>);
pub enum Solution {
    Cleanable(CleaningSequence),
    NotCleanable,
}

pub fn compute_cleaning_strategy<R: Rules>(
    movement_rules: R,
    visibility: usize,
    nr_cleaners: usize,
    edges: EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<Solution, String> {
    if nr_cleaners > MAX_COPS {
        let msg = format!("Rechnung kann für höchstens {MAX_COPS} Cleaner durchgeführt werden.");
        return Err(msg);
    }
    let nr_vertices = edges.nr_vertices();
    if nr_vertices.checked_pow(nr_cleaners as u32).is_none() {
        return Err("Cleanerpositionen passen nicht in usize.".to_string());
    }

    // stores for each position which vertices are cleaned if a cleaner occupies this position.
    let visible = {
        let visible_others = edges.pow(visibility);
        EdgeList::from_iter(
            izip!(0..nr_vertices, visible_others.neighbors())
                .map(|(v, vis)| std::iter::once(v).chain(vis)),
            visible_others.max_neighbors() + 1,
        )
    };

    let mut states = {
        let sym = crate::graph::NoSymmetry::new(nr_vertices);
        let positions = super::CopConfigurations::new(&edges, &sym, nr_cleaners, manager)?;
        let mut states = std::collections::HashMap::new();
        let nr_initial_states = positions.nr_configurations();
        if states.try_reserve(nr_initial_states).is_err() {
            return Err("nicht genug Speicher für States.".to_string());
        }
        for (i, position_index) in izip!(0.., positions.all_positions()) {
            if i % 4096 == 0 {
                manager.update(format!(
                    "initialisiere Cleanerstates: {:.2}%",
                    100.0 * (i as f32) / (nr_initial_states as f32)
                ))?;
            }
            let mut raw_positions = positions.eager_unpack(position_index);
            raw_positions.sort();
            let compact = CompactCleaners::from_raw(nr_vertices, &raw_positions);
            let mut fog = Fog::new_filled(nr_vertices);
            for &cleaner in raw_positions.iter() {
                for vis in visible.neighbors_of(cleaner) {
                    fog.mark_cleaned(vis);
                }
            }
            states.insert(compact, vec![FogState { fog, prev: compact }]);
        }
        states
    };

    manager.update("initialisiere Queue")?;
    let mut queue = std::collections::BinaryHeap::new();
    if queue.try_reserve(states.len()).is_err() {
        return Err("nicht genug Speicher für Queue.".to_string());
    }
    for (&cleaners, init) in &states {
        let nr_cleared = init[0].fog.count_cleaned(nr_vertices);
        queue.push(QueueEntry {
            nr_cleaned: nr_cleared,
            cleaners,
        });
    }

    let mut current_fogs = Vec::new();

    let mut most_cleaned_so_far = 0;
    let mut time_until_log_refresh: usize = 1;
    let final_cleaners = loop {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            manager.update(format!(
                "berechne Reinigungsstrategie:\n{:.2}% in Queue ({}), {} Knoten sauber",
                100.0 * (queue.len() as f32) / (states.len() as f32),
                queue.len(),
                most_cleaned_so_far,
            ))?;
            time_until_log_refresh = 10_000;
        }

        let Some(curr) = queue.pop() else {
            return Ok(Solution::NotCleanable);
        };
        debug_assert!(curr.nr_cleaned <= nr_vertices);
        most_cleaned_so_far = most_cleaned_so_far.max(curr.nr_cleaned);
        if curr.nr_cleaned == nr_vertices {
            break curr.cleaners;
        }

        {
            let curr_states = &states[&curr.cleaners];
            if current_fogs.try_reserve(curr_states.len()).is_err() {
                return Err("zu wenig Speicherplatz für States".to_string());
            }
            for state in curr_states.iter() {
                if state.fog.count_cleaned(nr_vertices) == curr.nr_cleaned {
                    current_fogs.push(state.fog.clone());
                }
            }
        }
        let curr_cleaners = curr.cleaners.into_raw(nr_cleaners, nr_vertices);
        for curr_fog in current_fogs.drain(..) {
            for mut next_cleaners in movement_rules.raw_cop_moves_from(&edges, curr_cleaners) {
                next_cleaners.sort();
                // todo: this function allocates. make it fallible.
                let next_fog = curr_fog.compute_step(&edges, &visible, &next_cleaners);
                let next_compact_cleaners = CompactCleaners::from_raw(nr_vertices, &next_cleaners);
                let next_states = states.get_mut(&next_compact_cleaners).unwrap();
                if next_states.iter().any(|state| state.fog.is_subset_of(&next_fog)) {
                    continue;
                }
                let next_nr_cleared = next_fog.count_cleaned(nr_vertices);

                if next_states.try_reserve(1).is_err() {
                    return Err("zu wenig Speicherplatz für States".to_string());
                }
                next_states.push(FogState {
                    fog: next_fog,
                    prev: curr.cleaners,
                });

                if queue.try_reserve(1).is_err() {
                    return Err("zu wenig Speicherplatz für Queue".to_string());
                }
                queue.push(QueueEntry {
                    nr_cleaned: next_nr_cleared,
                    cleaners: next_compact_cleaners,
                });
            }
        }
    };
    drop(queue);

    let mut cleaning_sequence = vec![final_cleaners];
    let mut curr_cleaners = final_cleaners;
    let mut curr_state = states[&final_cleaners]
        .iter()
        .find(|state| state.fog.count_cleaned(nr_vertices) == nr_vertices)
        .unwrap();
    loop {
        if curr_state.prev == curr_cleaners {
            break;
        }
        let prev_states = &states[&curr_state.prev];
        let raw_curr_cleaners = curr_cleaners.into_raw(nr_cleaners, nr_vertices);
        let Some(prev_state) = prev_states.iter().find(|state| {
            state.fog.compute_step(&edges, &visible, &raw_curr_cleaners) == curr_state.fog
        }) else {
            return Err("Programmfehler".to_string());
        };
        cleaning_sequence.push(curr_state.prev);
        curr_cleaners = curr_state.prev;
        curr_state = prev_state;
    }

    cleaning_sequence.reverse();
    Ok(Solution::Cleanable(CleaningSequence(cleaning_sequence)))
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
            if matches!(result, Solution::Cleanable(_)) {
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
        assert_eq!(cleaning_number(circle(25), 12), Ok(1));

        use crate::graph::{planar3d::Embedding3D, shape::Shape};
        let from_shape = |shape, res| Embedding3D::new_map_from(&shape, res).into_edges();
        assert_eq!(cleaning_number(from_shape(Shape::TriangGrid, 4), 1), Ok(1));
        // one cleaner guards a fixed edge, the other cleaners walk the six remaining vertices in parallel.
        assert_eq!(cleaning_number(from_shape(Shape::Cube, 0), 0), Ok(3));
    }
}
