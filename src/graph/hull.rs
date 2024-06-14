use std::collections::VecDeque;
use std::ops::Range;

use itertools::{izip, Itertools};

use super::*;
use crate::app::character::Character;

mod boundary;

mod escapable;
pub use escapable::EscapeableNodes;

mod escapable_extra;
pub use escapable_extra::DilemmaNodes;

mod plane_cop_strat;
pub use plane_cop_strat::PlaneCopStat;

/// geodesic convex hull over some vertices with respect to some graph
pub struct ConvexHullData {
    /// one entry per vertex in graph
    hull: Vec<InSet>,

    /// enumerates vertices which are on boundary
    /// beginning == end == some cop's position
    boundary: Vec<usize>,

    /// indexes in self.boundary, each segment is delimited by two cops on boundary.
    /// includes only vertices with min cop dist >= 2
    /// segments without any interior point (e.g. delimiting cops with distance <= 2) are NOT stored.
    safe_segments: Vec<Range<usize>>,

    /// same length as [`Self::segments`], stores vertices of cops guarding the segment of same index.
    guards: Vec<(usize, usize)>,
}

impl ConvexHullData {
    pub fn new() -> Self {
        Self {
            hull: Vec::new(),
            boundary: Vec::new(),
            safe_segments: Vec::new(),
            guards: Vec::new(),
        }
    }

    pub fn hull(&self) -> &[InSet] {
        &self.hull
    }

    #[allow(dead_code)]
    pub fn boundary(&self) -> &[usize] {
        &self.boundary
    }

    fn safe_boundary_indices(&self) -> &[Range<usize>] {
        &self.safe_segments
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of safe parts of boundary, e.g. cops and neighboring vertices cut away from begin and end.
    pub fn safe_boundary_parts(&self) -> impl ExactSizeIterator<Item = &'_ [usize]> + '_ + Clone {
        self.safe_segments.iter().map(|seg| &self.boundary[seg.clone()])
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of both boundary cops for a given boundary.
    /// each boundary cop should thus appear twice.
    pub fn boundary_cop_vertices(&self) -> &[(usize, usize)] {
        &self.guards
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
        //mark boundary as such
        let hull = &mut self.hull[..];
        for (v, mut neighs) in izip!(0.., edges.neighbors()) {
            if hull[v].contained() && neighs.any(|n| hull[n].outside()) {
                hull[v] = InSet::OnBoundary;
            }
        }
    }

    /// assumes that self.hull is already up to date
    fn update_boundary(
        &mut self,
        cops: &[Character],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.boundary.clear();
        for cop in cops {
            let v = cop.vertex();
            if cop.is_active() && self.hull[v].on_boundary() {
                self.boundary.push(v);
                if boundary::find_convex_hull_boundary(
                    &mut self.boundary,
                    cop.dists(),
                    &self.hull,
                    edges,
                    queue,
                )
                .is_ok()
                {
                    return;
                }
                self.boundary.clear();
            }
        }
    }

    fn update_segments(&mut self, min_cop_dist: &[isize]) {
        self.guards.clear();
        self.safe_segments.clear();
        debug_assert_eq!(self.hull.len(), min_cop_dist.len());
        if self.boundary.len() < 5 {
            return;
        }
        debug_assert_eq!(0, min_cop_dist[self.boundary[0]]);
        let mut start = 1;
        loop {
            while min_cop_dist[self.boundary[start]] == 0 {
                start += 1;
                if start == self.boundary.len() {
                    return;
                }
            }
            let mut stop = start;
            while min_cop_dist[self.boundary[stop]] != 0 {
                stop += 1;
                if stop == self.boundary.len() {
                    return;
                }
            }

            let (cop_1, cop_2) = (self.boundary[start - 1], self.boundary[stop]);
            debug_assert_eq!(min_cop_dist[cop_1], 0);
            debug_assert_eq!(min_cop_dist[cop_2], 0);
            let safe_part = (start + 1)..(stop - 1);
            if !safe_part.is_empty() {
                self.guards.push((cop_1, cop_2));
                self.safe_segments.push(safe_part);
            }
            start = stop;
        }
    }

    pub fn update(
        &mut self,
        cops: &[Character],
        edges: &EdgeList,
        min_cop_dist: &[isize],
        queue: &mut VecDeque<usize>,
        vertices_outside_hull: &[usize],
    ) {
        self.update_inside(cops, edges, queue, vertices_outside_hull);
        self.update_boundary(cops, edges, queue);
        self.update_segments(min_cop_dist);
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

/// very similar to ConvexHullData, only this time for just a pair of cops.
/// as this is computed (potentially) many times in a single frame, it tries to never iterate over all vertices.
#[derive(Default)]
struct CopPairHullData {
    hull: Vec<InSet>,         //one entry per vertex
    all_vertices: Vec<usize>, //indices of all vertices
}

impl CopPairHullData {
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
    debug_assert!(boundary.first().map_or(true, |&v| cop_dist[v] == 2));
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
        while check_front < boundary.len() {
            let dist_at = |i| cop_dist[boundary[i]];
            let check_dist = dist_at(check_front);
            let mut check_end = check_front + 1;
            while check_end < boundary.len() && dist_at(check_end) == check_dist {
                check_end += 1;
            }

            let check_range = &boundary[check_front..check_end];
            let check_len = check_range.len();

            boundary[retain_end] = 'next_vertex: {
                macro_rules! take_if_unique {
                    ($f:expr) => {{
                        let mut iter = check_range.iter().filter($f);
                        if let (Some(&v), None) = (iter.next(), iter.next()) {
                            break 'next_vertex v;
                        }
                    }};
                }
                if let &[v] = check_range {
                    break 'next_vertex v;
                }
                if retain_end > 0 {
                    let last_v = boundary[retain_end - 1];
                    take_if_unique!(|&&v| edges.has_edge(v, last_v));
                }
                take_if_unique!(|&&v| keep[v] == Keep::YesOnBoundary);
                take_if_unique!(|&&v| edges.neighbors_of(v).any(|n| keep[n] == Keep::Yes));
                take_if_unique!(|&&v| edges.neighbors_of(v).all(|n| keep[n] == Keep::Perhaps));
                //at this point something went wrong and we try to continue on a best effort basis
                println!(
                    "fehler in hull::retain_safe_inner_boundary an {:?}",
                    check_range
                );
                boundary[check_front]
            };
            retain_end += 1;
            check_front += check_len;
        }
    }
    boundary.truncate(retain_end);
}
