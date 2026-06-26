use std::collections::VecDeque;

use bitvec::boxed::BitBox;
use itertools::izip;

use crate::graph::EdgeList;

/// this has the same size as just the [`bitvec::boxed::BitBox`] variant on it's own,
/// because the pointer pointing to the `BitBox` storage is guaranteed to never be null.
/// (and because this pointer is a fat pointer, e.g. also carries the length of the held storage.)
/// note: i just hope that the branch predictor is able to overcome any run-time overhead
/// caused by the match of `self` in each non-static method.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Fog {
    Smol(usize),
    Big(BitBox),
}

impl Fog {
    /// number of bits of a usize
    const MAX_SMOL: usize = usize::BITS as usize;

    pub fn new_filled(nr_vertices: usize) -> Self {
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
    pub fn new_initial(cleaners: &[usize], visible: &EdgeList) -> Self {
        let mut result = Self::new_filled(visible.nr_vertices());
        for &cleaner in cleaners {
            for vis in visible.neighbors_of(cleaner) {
                result.mark_cleaned_at(vis);
            }
        }
        result
    }

    pub fn count_foggy(&self) -> usize {
        match self {
            Self::Smol(data) => data.count_ones() as usize,
            Self::Big(data) => data.count_ones(),
        }
    }

    #[cfg(test)]
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

    pub fn is_foggy_at(&self, v: usize) -> bool {
        match self {
            Self::Smol(data) => (*data & (1usize << v)) != 0,
            Self::Big(data) => data[v],
        }
    }

    /// note: us with caution, as we don't know the number of vertices.
    /// the returned value is thus almost always too long.
    fn as_slice(&self) -> &bitvec::slice::BitSlice {
        match self {
            Self::Smol(data) => bitvec::slice::BitSlice::from_element(data),
            Self::Big(data) => data.as_bitslice(),
        }
    }

    pub fn as_mut_slice(&mut self, nr_vertices: usize) -> &mut bitvec::slice::BitSlice {
        let raw = match self {
            Self::Smol(data) => bitvec::slice::BitSlice::from_element_mut(data),
            Self::Big(data) => data.as_mut_bitslice(),
        };
        &mut raw[..nr_vertices]
    }

    /// note that this will usually contain more bits than number of vertices
    pub fn as_raw_mut_slice(&mut self) -> &mut [usize] {
        match self {
            Self::Smol(data) => std::slice::from_mut(data),
            Self::Big(data) => data.as_raw_mut_slice(),
        }
    }

    /// note that this will usually contain more bits than number of vertices
    pub fn as_raw_slice(&self) -> &[usize] {
        match self {
            Self::Smol(data) => std::slice::from_ref(data),
            Self::Big(data) => data.as_raw_slice(),
        }
    }

    pub fn set_cleared(&mut self) {
        self.as_raw_mut_slice().fill(0);
    }

    pub fn or_assign(&mut self, other: &Self) {
        for (self_chunk, other_chunk) in izip!(self.as_raw_mut_slice(), other.as_raw_slice()) {
            *self_chunk |= other_chunk;
        }
    }

    fn iter_foggy(&self) -> impl Iterator<Item = usize> {
        self.as_slice().iter_ones()
    }

    /// returns [`std::cmp::Ordering::Equal`] if `self == other`,
    /// [`std::cmp::Ordering::Less`] if self is a subset of other and
    /// [`std::cmp::Ordering::Greater`] if self is a suberset of other.
    /// should none of these cases hold, [`None`] is returned.
    pub fn subset_ord(&self, other: &Self) -> Option<std::cmp::Ordering> {
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

    pub fn is_subset_of(&self, other: &Self) -> bool {
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
pub struct FogStepComputation<'a> {
    pub edges: &'a EdgeList,
    pub visible: &'a EdgeList,
    pub fog_speed: isize,
    distances: Vec<isize>,
    queue: VecDeque<usize>,
}

impl<'a> FogStepComputation<'a> {
    pub fn new(edges: &'a EdgeList, visible: &'a EdgeList, fog_speed: isize) -> Self {
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

    fn compute_step_general(&mut self, curr_fog: &Fog, new_positions: &[usize]) -> Fog {
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
        let mut next_fog = fog_after_step;
        let select = |v: usize, dists: &[isize], new_dist: isize| {
            if new_dist <= self.fog_speed && out_of_reach.is_foggy_at(v) && dists[v] > new_dist {
                debug_assert!(!next_fog.is_foggy_at(v));
                next_fog.mark_foggy_at(v);
                return true;
            }
            false
        };
        self.edges
            .calc_distances_to_with(&mut self.distances, select, &mut self.queue);

        next_fog
    }

    pub fn compute_step(&mut self, curr_fog: &Fog, new_positions: &[usize]) -> Fog {
        if self.fog_speed == 1 {
            let next_fog = curr_fog.compute_step(self.edges, self.visible, new_positions);
            debug_assert_eq!(next_fog, self.compute_step_general(curr_fog, new_positions));
            next_fog
        } else {
            self.compute_step_general(curr_fog, new_positions)
        }
    }
}

/// for a given vertex `v`, the returned "neighbors" of `v` are
/// the vertices with distance `<= max_dist` in `edges`.
/// note: this means `v` always "neighbors" itself.
pub fn compute_near_vertices(edges: &EdgeList, max_dist: usize) -> EdgeList {
    let max_dist = max_dist.min(edges.nr_vertices());
    let visible_others = edges.pow(max_dist);
    let all_visible = izip!(0..edges.nr_vertices(), visible_others.neighbors())
        .map(|(v, vis)| std::iter::once(v).chain(vis));

    let max_dregree = visible_others.max_degree() + 1;
    EdgeList::from_iter(all_visible, max_dregree)
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
}
