use std::collections::VecDeque;
use std::ops::Range;

use itertools::{izip, Itertools};

use super::*;
use crate::app::character::Character;

mod escapable;
pub use escapable::EscapableNodes;

mod escapable_grid;
pub use escapable_grid::EscapableDirections;

mod escapable_extra;
pub use escapable_extra::DilemmaNodes;

mod plane_cop_strat;
pub use plane_cop_strat::PlaneCopStat;

struct SafeSegment {
    /// indexes into [`CopsHull::flat_boundary_segments`]
    boundary_indices: Range<usize>,
    /// vertices of cops guarding the segment
    guards: (usize, usize),
}

/// geodesic convex hull over set of police positions.
/// this is not guaranteed to always exactly resemble the convex hull.
/// what is guaranteed however, is the following:
///  - every vertex on a shortest path between two policemen is contained.
///  - if the vertex furthest away from any officer is not on such a shortest path, it is not contained.
///  - in the graph without the path vertices,
///    every connected component other than the one with the vertex furthest away is also contained.
pub struct CopsHull {
    /// one entry per vertex in graph
    hull: Vec<InSet>,

    /// one entry per vertex in graph
    dist_to_hull: Vec<isize>,

    /// all winning boundary segments are stored in direct succession
    flat_boundary_segments: Vec<usize>,

    /// if the boundary has `n` segments, this has length `n`.
    safe_segments: Vec<SafeSegment>,
}

impl CopsHull {
    pub fn new() -> Self {
        Self {
            hull: Vec::new(),
            dist_to_hull: Vec::new(),
            flat_boundary_segments: Vec::new(),
            safe_segments: Vec::new(),
        }
    }

    pub fn hull(&self) -> &[InSet] {
        &self.hull
    }

    pub fn dist_to_hull(&self) -> &[isize] {
        &self.dist_to_hull
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of safe parts of boundary, e.g. cops and neighboring vertices cut away from begin and end.
    pub fn safe_boundary_parts(&self) -> impl ExactSizeIterator<Item = &'_ [usize]> + '_ + Clone {
        self.safe_segments
            .iter()
            .map(|seg| &self.flat_boundary_segments[seg.boundary_indices.clone()])
    }

    pub fn boundary_cop_vertices(
        &self,
    ) -> impl ExactSizeIterator<Item = &(usize, usize)> + '_ + Clone {
        self.safe_segments.iter().map(|seg| &seg.guards)
    }

    fn update_inside(
        &mut self,
        cops: &[Character],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        vertices_outside_hull: &[usize],
    ) {
        queue.clear();

        self.hull.clear();
        self.hull.resize(edges.nr_vertices(), InSet::Unknown);
        let hull = &mut self.hull[..];
        for i in 0..cops.len() {
            let cop_i = &cops[i];
            if !cop_i.is_active() {
                continue;
            }
            for cop_j in cops[(i + 1)..].iter().filter(|c| c.is_active()) {
                //walk from cop i to cop j on all shortest paths
                hull[cop_i.vertex()] = InSet::NewlyAdded;
                queue.push_back(cop_i.vertex());
                while let Some(node) = queue.pop_front() {
                    let curr_dist_to_j = cop_j.dists()[node];
                    for neigh in edges.neighbors_of(node) {
                        if cop_j.dists()[neigh] < curr_dist_to_j && hull[neigh] != InSet::NewlyAdded
                        {
                            hull[neigh] = InSet::NewlyAdded;
                            queue.push_back(neigh);
                        }
                    }
                }
                //change these paths from InSet::NewlyAdded to InSet::Yes
                //(to allow new paths to go through the current one)
                queue.push_back(cop_i.vertex());
                hull[cop_i.vertex()] = InSet::Interieur;
                edges.recolor_region((InSet::NewlyAdded, InSet::Interieur), hull, queue);
            }
        }
        //color outside as InSet::No (note: this might miss some edge cases; best to not place cops at rim)
        for &p in vertices_outside_hull {
            if hull[p] == InSet::Unknown {
                hull[p] = InSet::No;
                queue.push_back(p);
            }
        }
        edges.recolor_region((InSet::Unknown, InSet::No), hull, queue);

        //color remaining InSet::Perhaps as InSet::Interieur
        for x in hull {
            if *x == InSet::Unknown {
                *x = InSet::Interieur;
            }
        }
    }

    fn find_boundary(
        &mut self,
        cops: &[Character],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        min_cop_dist: &[isize],
    ) {
        let is_edge = |v1: usize, v2: usize| edges.has_directed_edge(v1, v2);
        let neighs_of = |v: usize| edges.neighbors_of(v);

        assert_eq!(self.hull.len(), edges.nr_vertices());
        assert_eq!(min_cop_dist.len(), edges.nr_vertices());

        let hull = &mut self.hull[..];
        debug_assert!(hull.iter().all(|&h| h.finished_construction() && h != InSet::OnBoundary));

        self.flat_boundary_segments.clear();
        self.safe_segments.clear();

        // after we discovered something vertex `v` is on the boundary, we set `hull[v] == InSet::OnBoundary`.
        // what is considered as outside id defined as our input.
        let inside_next_to = |v: usize, hull: &[InSet], outside: InSet| -> Option<usize> {
            if hull[v] == InSet::Interieur {
                neighs_of(v).find(|&n| hull[n] == outside)
            } else {
                None
            }
        };

        let new_on_boundary =
            |v: usize, hull: &[InSet]| inside_next_to(v, hull, InSet::No).is_some();

        // mark boundary vertices that are not safe
        for cop in cops {
            if !cop.is_active() {
                continue;
            }
            let cop_v = cop.vertex();
            for v in std::iter::once(cop_v).chain(neighs_of(cop_v)) {
                if new_on_boundary(v, hull) {
                    debug_assert!(min_cop_dist[v] <= 1);
                    hull[v] = InSet::OnBoundary;
                    self.flat_boundary_segments.push(v);
                }
            }
        }

        const BND_WITHNESS: InSet = InSet::Unknown;
        // we guarantee to find every safe boundary component by iterating trough every vertex.
        // how far we have iterated previously, is remembered here.
        let mut search_start = 0;
        loop {
            // find first not-yet handled vertex of a winning boundary component
            let (fst_on_boundary, fst_withness) = 'find_new_component: {
                for v in search_start..hull.len() {
                    if let Some(fst_withness) = inside_next_to(v, hull, InSet::No) {
                        if min_cop_dist[v] > 1 {
                            search_start = v + 1;
                            break 'find_new_component (v, fst_withness);
                        } else {
                            debug_assert_eq!(hull[v], InSet::OnBoundary);
                        }
                    }
                }
                debug_assert!((0..hull.len()).all(|v| !new_on_boundary(v, hull)));
                debug_assert!(hull.iter().copied().all(InSet::finished_construction));
                return;
            };
            // mark every suitable withness vertex as withness
            debug_assert!(queue.is_empty());
            queue.push_back(fst_withness);
            while let Some(w) = queue.pop_front() {
                if hull[w] == InSet::No && is_edge(fst_on_boundary, w) {
                    hull[w] = BND_WITHNESS;
                    queue.extend(neighs_of(w));
                }
            }
            for w in neighs_of(fst_on_boundary) {
                if hull[w] == BND_WITHNESS {
                    for ww in neighs_of(w) {
                        if hull[ww] == InSet::No {
                            hull[ww] = BND_WITHNESS;
                        }
                    }
                }
            }

            let segment_index_start = self.flat_boundary_segments.len();
            let mut guard_cops_vec = smallvec::SmallVec::<[&Character; 6]>::new();

            debug_assert!(queue.is_empty());
            queue.push_back(fst_on_boundary);
            while let Some(v) = queue.pop_front() {
                // a vertex is on the boundary, if there exists withness next to him, that was discovered earlier.
                let Some(mut withness) = inside_next_to(v, hull, BND_WITHNESS) else {
                    continue;
                };

                // update withness state
                for w in neighs_of(v) {
                    if hull[w] == BND_WITHNESS {
                        withness = w;
                        let mut find_front_withness = || -> bool {
                            for ww in neighs_of(withness) {
                                if hull[ww] == InSet::No && is_edge(v, ww) {
                                    hull[ww] = BND_WITHNESS;
                                    withness = ww;
                                    return true;
                                }
                            }
                            false
                        };
                        // we advance the withness as far as possible while still neighboring v.
                        // this is both needed on square grids and on outher corners of triangle grids
                        if find_front_withness() {
                            while find_front_withness() {}
                            break;
                        }
                    }
                }
                // this part is only needed on square grids.
                'mark_withness_withness: loop {
                    for n in neighs_of(withness) {
                        if hull[n] == InSet::No {
                            hull[n] = BND_WITHNESS;
                        }
                    }
                    // this is needed for an outher corner on a quad grid
                    for w in neighs_of(v) {
                        if hull[w] == InSet::No {
                            for ww in neighs_of(w) {
                                if hull[ww] == BND_WITHNESS {
                                    hull[w] = BND_WITHNESS;
                                    withness = w;
                                    continue 'mark_withness_withness;
                                }
                            }
                        }
                    }
                    break 'mark_withness_withness;
                }

                // push to queue, update boundary state
                self.flat_boundary_segments.push(v);
                hull[v] = InSet::OnBoundary;
                'test_neighbors: for n in neighs_of(v) {
                    // add new guarding cops that are near by
                    if hull[n] == InSet::OnBoundary && min_cop_dist[n] <= 1 {
                        debug_assert_eq!(min_cop_dist[n], 1);
                        'find_guard_cop: for nn in neighs_of(n) {
                            if min_cop_dist[nn] == 0 {
                                if guard_cops_vec.iter().any(|cop| cop.vertex() == nn) {
                                    continue 'find_guard_cop;
                                }
                                for cop in cops {
                                    if cop.vertex() == nn {
                                        guard_cops_vec.push(cop);
                                    }
                                }
                            }
                        }
                    }

                    if hull[n] != InSet::Interieur {
                        continue 'test_neighbors;
                    }
                    queue.push_back(n);

                    // if n is an "inner corner" on the quad torus, we need this dance here.
                    if neighs_of(n).any(|nn| matches!(hull[nn], BND_WITHNESS | InSet::No)) {
                        continue 'test_neighbors;
                    }
                    let mut nn_withnesses_vec = smallvec::SmallVec::<[_; 8]>::new();
                    nn_withnesses_vec.extend(
                        neighs_of(v).filter(|&w| hull[w] == BND_WITHNESS && !is_edge(n, w)),
                    );
                    let nn_withnesses = &nn_withnesses_vec[..];
                    for nn in neighs_of(n) {
                        if nn != v
                            && hull[nn] == InSet::Interieur
                            && !is_edge(v, nn)
                            && nn_withnesses.iter().any(|&w| is_edge(nn, w))
                        {
                            self.flat_boundary_segments.push(n);
                            queue.push_back(nn);
                            continue 'test_neighbors;
                        }
                    }
                }
            }

            // unmark withnesses
            debug_assert!(queue.is_empty());
            queue.push_back(fst_withness);
            while let Some(w) = queue.pop_front() {
                if hull[w] == BND_WITHNESS {
                    hull[w] = InSet::No;
                    queue.extend(neighs_of(w));
                }
            }
            debug_assert!(!BND_WITHNESS.finished_construction());
            debug_assert!(hull.iter().copied().all(InSet::finished_construction));

            let guard_cops = &guard_cops_vec[..];
            if guard_cops.is_empty() {
                // abort of somehow no officer was found
                self.flat_boundary_segments.truncate(segment_index_start);
                continue;
            }
            // compute the resulting segment
            let segment = {
                let mut max_dist = -1;
                let mut guards = (guard_cops[0], guard_cops[0]);
                // decide which pair of cops will be responsible
                for i1 in 0..guard_cops.len() {
                    for i2 in (i1 + 1)..guard_cops.len() {
                        let cop1 = guard_cops[i1];
                        let cop2 = guard_cops[i2];
                        let pair_dist = cop1.dists()[cop2.vertex()];
                        debug_assert_eq!(cop2.dists()[cop1.vertex()], pair_dist);
                        if pair_dist > max_dist {
                            max_dist = pair_dist;
                            guards = (cop1, cop2);
                        }
                    }
                }

                // order vertices such that they actually form a path
                let indices = segment_index_start..self.flat_boundary_segments.len();
                let new_segment = &mut self.flat_boundary_segments[indices.clone()];
                let dist_guard_0 = guards.0.dists();
                new_segment.sort_by_key(|&v| dist_guard_0[v]);
                // most of the time sorting should be sufficient,
                // this is only required in special cases on tori.
                for i in 0..(new_segment.len() - 1) {
                    let vi = new_segment[i];
                    if is_edge(vi, new_segment[i + 1]) {
                        continue;
                    }
                    for (j, &vj) in izip!((i + 2).., &new_segment[(i + 2)..]) {
                        if is_edge(vi, vj) {
                            new_segment.swap(i + 1, j);
                            debug_assert!(is_edge(vi, new_segment[i + 1]));
                            break;
                        }
                    }
                }
                // TODO: there are some cases on quad tori where the above is not sufficient.
                // debug_assert!(edges.has_path(new_segment));

                SafeSegment {
                    boundary_indices: indices,
                    guards: (guards.0.vertex(), guards.1.vertex()),
                }
            };

            self.safe_segments.push(segment);
        }
    }

    fn compute_dist_to_hull(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        assert_eq!(edges.nr_vertices(), self.hull.len());
        debug_assert!(self.hull.iter().copied().all(InSet::finished_construction));

        queue.clear();
        self.dist_to_hull.clear();
        self.dist_to_hull.resize(edges.nr_vertices(), isize::MAX);

        for (v, &h, d) in izip!(0.., &self.hull, &mut self.dist_to_hull) {
            if h.contained() {
                *d = 0;
                queue.push_back(v);
            }
        }
        edges.calc_distances_to(queue, &mut self.dist_to_hull);
    }

    pub fn update(
        &mut self,
        cops: &[Character],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        vertices_outside_hull: &[usize],
        min_cop_dist: &[isize],
    ) {
        self.update_inside(cops, edges, queue, vertices_outside_hull);
        self.compute_dist_to_hull(edges, queue);
        self.find_boundary(cops, edges, queue, min_cop_dist);
    }
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Keep {
    No,
    Perhaps,
    YesOnBoundary,
    Yes,
    YesButAlreadyVisited,
}

/// very similar to [`CopsHull`], only this time for just a pair of cops.
/// as this is computed (potentially) many times in a single frame, it tries to never iterate over all vertices.
#[derive(Default)]
struct CopPairHull {
    hull: Vec<InSet>,         //one entry per vertex
    all_vertices: Vec<usize>, //indices of all vertices
}

impl CopPairHull {
    fn compute_hull(
        &mut self,
        [cop_1, cop_2]: [&Character; 2],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.hull.resize(edges.nr_vertices(), InSet::No);
        debug_assert!(self.hull.iter().all(|&x| x == InSet::No));
        debug_assert!(queue.is_empty());

        let mut all_entries = std::mem::take(&mut self.all_vertices);
        all_entries.clear();

        self.hull[cop_1.vertex()] = InSet::Interieur;
        queue.push_back(cop_1.vertex());
        all_entries.push(cop_1.vertex());
        while let Some(v) = queue.pop_front() {
            let curr_dist_to_2 = cop_2.dists()[v];
            for n in edges.neighbors_of(v) {
                if cop_2.dists()[n] < curr_dist_to_2 && self.hull[n] == InSet::No {
                    self.hull[n] = InSet::Interieur;
                    queue.push_back(n);
                    all_entries.push(n);
                }
            }
        }
        debug_assert!(all_entries.contains(&cop_1.vertex()));
        debug_assert!(all_entries.contains(&cop_2.vertex()));
        self.all_vertices = all_entries;
    }

    fn reset_hull(&mut self, cop_1: &Character, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        debug_assert_eq!(self.hull.len(), edges.nr_vertices());
        debug_assert!(queue.is_empty());

        //should be faster than iterating over whole array most of the time (by far)
        self.hull[cop_1.vertex()] = InSet::No;
        queue.push_back(cop_1.vertex());
        edges.recolor_region_with(InSet::No, &mut self.hull, |_, _| true, queue);

        debug_assert!(self.hull.iter().all(|&x| x == InSet::No));
    }
}

/// keep one vertex of every distance to cop, choose that vertex to lie close as possible to outwards boundary.
/// result is assumed to be connected path from cop to
/// his colleague (minus first and last two steps, cause minimum distance >= 2)
fn retain_safe_inner_boundary(
    cop_dist: &[isize],
    boundary: &mut Vec<usize>,
    keep: &[Keep],
    edges: &EdgeList,
) {
    debug_assert_eq!(edges.nr_vertices(), cop_dist.len());
    debug_assert_eq!(edges.nr_vertices(), keep.len());
    //boundary is expected to be filtered vertices of convex hull of cop pair,
    //where vertices are ordered the same as when discovered in convex hull.
    debug_assert!(boundary.first().is_none_or(|&v| cop_dist[v] == 2));
    debug_assert!(boundary.iter().tuple_windows().all(|(&v1, &v2)| {
        let d1 = cop_dist[v1];
        let d2 = cop_dist[v2];
        let sorted = d1 <= d2;
        let no_jump = (d1 - d2).abs() <= 1;
        sorted && no_jump
    }));

    let mut retain_end = 0;
    let mut check_front = 0;
    {
        let boundary = &mut boundary[..];
        let mut check_vals = smallvec::SmallVec::<[usize; 6]>::new();
        'check_all: while check_front < boundary.len() {
            let dist_at = |i| cop_dist[boundary[i]];
            let check_dist = dist_at(check_front);
            let mut check_end = check_front + 1;
            while check_end < boundary.len() && dist_at(check_end) == check_dist {
                check_end += 1;
            }

            check_vals.clear();
            check_vals.extend(boundary[check_front..check_end].iter().copied());
            let check_len = check_vals.len();
            if retain_end > 0 {
                let last_v = boundary[retain_end - 1];
                check_vals.retain(|v| edges.has_edge(*v, last_v));
            }

            boundary[retain_end] = 'next_vertex: {
                if check_vals.len() == 1 {
                    break 'next_vertex check_vals[0];
                }
                macro_rules! take_if_unique {
                    ($f:expr) => {{
                        let mut iter = check_vals.iter().filter($f);
                        if let (Some(&v), None) = (iter.next(), iter.next()) {
                            break 'next_vertex v;
                        }
                    }};
                }
                const YOB: Keep = Keep::YesOnBoundary;
                take_if_unique!(|&&v| keep[v] == YOB);
                take_if_unique!(|&&v| edges.neighbors_of(v).any(|n| keep[n] == YOB));
                take_if_unique!(|&&v| edges.neighbors_of(v).any(|n| keep[n] == Keep::Yes));
                take_if_unique!(|&&v| edges.neighbors_of(v).all(|n| keep[n] == Keep::Perhaps));
                take_if_unique!(|&&v| keep[v] == Keep::Yes);

                //at this point something went wrong and we try to continue on a best effort basis
                println!(
                    "fehler in hull::retain_safe_inner_boundary an {:?} (indices {:?}): \
                    bnd = {:?}, n-bnd = {:?}, yes = {:?}, n-yes = {:?}, n-perhaps = {:?}",
                    check_vals,
                    check_front..check_end,
                    check_vals.iter().filter(|&&v| keep[v] == YOB).collect_vec(),
                    check_vals
                        .iter()
                        .filter(|&&v| edges.neighbors_of(v).any(|n| keep[n] == YOB))
                        .collect_vec(),
                    check_vals.iter().filter(|&&v| keep[v] == Keep::Yes).collect_vec(),
                    check_vals
                        .iter()
                        .filter(|&&v| edges.neighbors_of(v).any(|n| keep[n] == Keep::Yes))
                        .collect_vec(),
                    check_vals
                        .iter()
                        .filter(|&&v| edges.neighbors_of(v).all(|n| keep[n] == Keep::Perhaps))
                        .collect_vec(),
                );
                macro_rules! take_largest {
                    ($f:expr) => {{
                        let mut best_nr = 0;
                        let mut best_val = usize::MAX;
                        for &val in &check_vals {
                            let new_nr = $f(val).count();
                            if new_nr > best_nr {
                                best_nr = new_nr;
                                best_val = val;
                            }
                        }
                        if best_val != usize::MAX {
                            println!("^ entschieden für {best_val}");
                            break 'next_vertex best_val;
                        }
                    }};
                }
                take_largest!(|v| edges.neighbors_of(v).filter(|&n| keep[n] == YOB));
                take_largest!(|v| edges.neighbors_of(v).filter(|&n| keep[n] == Keep::Yes));
                take_largest!(|v| edges.neighbors_of(v).filter(|&n| keep[n] != Keep::No));

                println!("^ ungelöst");
                if check_vals.is_empty() {
                    break 'check_all;
                }
                check_vals[0]
            };
            retain_end += 1;
            check_front += check_len;
        }
    }
    boundary.truncate(retain_end);

    debug_assert!(boundary.first().is_none_or(|&v| cop_dist[v] == 2));
    debug_assert!(edges.has_path(boundary));
}
