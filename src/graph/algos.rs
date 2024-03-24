use std::collections::VecDeque;
use std::ops::Range;

use itertools::{izip, Itertools};

use super::*;
use crate::app::character::Character;

/// tries to find the boundary of `set` as if `edges` belong to a triangulation.
/// returns wether the process was successful.
/// on failure the boundary may be partially constructed.
fn find_hull_boundary_in_triangulation(
    boundary: &mut Vec<usize>,
    hull: &[InSet],
    edges: &EdgeList,
) -> bool {
    debug_assert_eq!(boundary.len(), 1);
    let fst_inside = boundary[0];
    debug_assert!(hull[fst_inside].on_boundary());
    let mut curr_inside = fst_inside;
    let mut last_inside = fst_inside;
    // similar deal to snd_last_outside below
    let mut snd_last_inside = fst_inside;

    let neighbors_of = |v: usize| edges.neighbors_of(v);

    // snd_last_outside is almost not needed: all vertices inside the hull
    // neighboring any given vertex outside must be a clique,
    // else the path connecting them through the outside vertex is a shortest path
    // and thus the vertex not outside the hull.
    // if our (assumed to be locally planar) graph somehow contains a four-clique or
    // the hull is not actually a convex hull, this variable can safe us.
    let mut snd_last_outside = usize::MAX;
    let fst_outside = neighbors_of(fst_inside).find(|&n| hull[n].outside()).unwrap();
    let mut last_outside = fst_outside;
    let Some(mut curr_outside) =
        neighbors_of(fst_inside).find(|&n| hull[n].outside() && edges.has_edge(n, fst_outside))
    else {
        //can fail if object we want to find the boundary of is not actually a convex hull.
        //or if the graph in question is not (locally) planar
        return false;
    };

    // idea: curr_outside must neighbor curr_inside at all time. this way we are guaranteed
    // to not switch sides on a hull with local thickness <= 2.
    // if neighter of curr_outside and curr_inside can be advanced, while keeping
    // the neighbor relation, we have something other than a triangulation and exit.
    let mut change = true;
    while change && curr_outside != fst_outside {
        change = false;

        // step outside the hull but keep curr_inside as neighbor
        let outside_step = neighbors_of(curr_outside).find(|&n| {
            ![last_outside, snd_last_outside].contains(&n)
                && hull[n].outside()
                && edges.has_edge(curr_inside, n)
        });
        if let Some(next_outside) = outside_step {
            snd_last_outside = last_outside;
            last_outside = curr_outside;
            curr_outside = next_outside;
            change = true;
        }

        // step on hull boundary but keep curr_outside as neighbor
        let inside_step = neighbors_of(curr_inside).find(|&n| {
            ![last_inside, snd_last_inside].contains(&n)
                && hull[n].on_boundary()
                && edges.has_edge(curr_outside, n)
        });
        if let Some(next_inside) = inside_step.or_else(|| {
            // special case if hull has thickness 1 and curr_inside is an extreme point
            (!change
                && edges.has_edge(curr_outside, last_inside)
                && edges.has_edge(last_outside, curr_inside))
            .then_some(last_inside)
        }) {
            boundary.push(next_inside);
            snd_last_inside = last_inside;
            last_inside = curr_inside;
            curr_inside = next_inside;
            change = true;
        }

        debug_assert!(edges.has_edge(curr_inside, curr_outside));
        if boundary.len() == edges.nr_vertices() {
            boundary.clear();
            return false;
        }
    }
    // we always try to take a step in the hull after we step outside.
    // if we reach the outside start with an outside step, we are guaranteed to find the
    // inside start with the following step inside.
    // the only special case is where the boundary has length 2 or 3, because then fst_inside will
    // not be readded, as it is also last_inside or snd_last_inside.
    debug_assert!(
        !change || Some(&fst_inside) == boundary.last() || matches!(boundary.len(), 2 | 3)
    );
    change
}

/// build to return something often resembling a hull boundary if the graph in question is
/// not a triangulation.
/// if the function returns "success", it is however not guaranteed that the result is actually correct.
#[allow(dead_code)]
fn find_hull_boundary_fallback(
    boundary: &mut Vec<usize>,
    dist_to_boundary_start: &[isize],
    hull: &[InSet],
    edges: &EdgeList,
    queue: &mut VecDeque<usize>,
) -> bool {
    debug_assert_eq!(boundary.len(), 1);
    let fst_inside = boundary[0];
    debug_assert_eq!(dist_to_boundary_start[fst_inside], 0);
    debug_assert!(hull[fst_inside].on_boundary());

    let mut potential_next = std::mem::take(queue);
    let mut curr_inside = fst_inside;
    let mut last_inside = fst_inside;
    'find_boundary: loop {
        potential_next.clear();
        potential_next.extend(
            edges
                .neighbors_of(curr_inside)
                .filter(|&n| n != last_inside && hull[n].on_boundary()),
        );
        if potential_next.len() > 1 {
            // if we have choice,
            // only keep the options visited the longest time ago / not at all
            for v in boundary.iter().rev() {
                if potential_next.contains(v) {
                    potential_next.retain(|next| next != v);
                    debug_assert!(!potential_next.is_empty());
                    if potential_next.len() == 1 {
                        break;
                    }
                }
            }
        }

        let next_inside = match potential_next.len() {
            1 => potential_next[0], //common case
            0 => {
                // if the hull has some pointy corner where the boundary visits the same vertices twice,
                // once before visiting the corner, once after leaving,
                // we need to take last_inside again when curr_inside is corner.
                // note: the case where the hull consists of only a single vertex is also covered here,
                // as last_inside (like curr_inside) starts beeing equal to fst_inside.
                last_inside
            },
            _ => {
                // if there are multiple options not yet visited,
                // we take the one leading further away from the search start.
                // this is no exact science but should at least be somewhat
                // reliable at the start of construction.
                let max_dist = potential_next
                    .iter()
                    .fold(0, |max, &v| isize::max(max, dist_to_boundary_start[v]));
                potential_next
                    .iter()
                    .copied()
                    .find(|&v| dist_to_boundary_start[v] == max_dist)
                    .unwrap()
            },
        };
        last_inside = curr_inside;
        curr_inside = next_inside;
        boundary.push(next_inside);
        if next_inside == fst_inside {
            break 'find_boundary; //happy case :)
        }
        if boundary.len() > edges.nr_vertices() {
            //emergency exit. something went wrong and we failed to find the boundary :(
            boundary.clear();
            break 'find_boundary;
        }
    }
    potential_next.clear();
    *queue = potential_next;

    boundary.last() == Some(&fst_inside)
}

/// assumes boundary has first vertex already inserted as starting point
/// and that hull is up-to date, including having all boundary vertices marked
fn update_convex_hull_boundary(
    boundary: &mut Vec<usize>,
    _dist_to_boundary_start: &[isize],
    hull: &[InSet],
    edges: &EdgeList,
    _queue: &mut VecDeque<usize>,
) -> bool {
    // currently all graphs with continuous boundary are triangulations.
    // the fallback is thus not of any advantage for now.
    find_hull_boundary_in_triangulation(boundary, hull, edges)
}

/// relies just on [`InSet::No`] and [`InSet::Interieur`], doesn't need [`InSet::OnBoundary`] for `hull` to function.
/// `all_elements` contains every element of `hull`.
fn find_unordered_boundary(
    boundary: &mut Vec<usize>,
    hull: &[InSet],
    edges: &EdgeList,
    all_elements: &[usize],
) {
    debug_assert!(boundary.is_empty());
    debug_assert_eq!(edges.nr_vertices(), hull.len());

    for &v in all_elements {
        debug_assert!(hull[v].contained());
        if edges.neighbors_of(v).any(|n| hull[n].outside()) {
            boundary.push(v);
        }
    }
}

/// geodesic convex hull over some vertices with respect to some graph
pub struct ConvexHullData {
    /// one entry per vertex in graph
    hull: Vec<InSet>,

    /// enumerates vertices which are on boundary
    /// beginning == end == some cop's position
    boundary: Vec<usize>,

    /// indexes in self.boundary, each segment is delimited by two cops on boundary.
    /// includes the boundary cops at beginning and end
    /// segments without any interior point (e.g. delimiting cops with distance <= 2) are NOT stored.
    segments: Vec<Range<usize>>,
}

impl ConvexHullData {
    pub fn new() -> Self {
        Self {
            hull: Vec::new(),
            boundary: Vec::new(),
            segments: Vec::new(),
        }
    }

    pub fn hull(&self) -> &[InSet] {
        &self.hull
    }

    #[allow(dead_code)]
    pub fn boundary(&self) -> &[usize] {
        &self.boundary
    }

    fn safe_boundary_indices(&self) -> impl ExactSizeIterator<Item = Range<usize>> + '_ + Clone {
        self.segments.iter().map(|seg| {
            let interior = (seg.start + 2)..(seg.end - 2);
            debug_assert!(!interior.is_empty());
            interior
        })
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of safe parts of boundary, e.g. cops and neighboring vertices cut away from begin and end.
    pub fn safe_boundary_parts(&self) -> impl ExactSizeIterator<Item = &'_ [usize]> + '_ + Clone {
        self.safe_boundary_indices().map(|interior| &self.boundary[interior])
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of both boundary cops for a given boundary.
    /// each boundary cop should thus appear twice.
    pub fn boundary_cop_vertices(
        &self,
    ) -> impl ExactSizeIterator<Item = (usize, usize)> + '_ + Clone {
        self.segments
            .iter()
            .map(|seg| (self.boundary[seg.start], self.boundary[seg.end - 1]))
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
                hull[cop_i.nearest_node] = InSet::NewlyAdded;
                queue.push_back(cop_i.nearest_node);
                while let Some(node) = queue.pop_front() {
                    let curr_dist_to_j = cop_j.distances[node];
                    for neigh in edges.neighbors_of(node) {
                        if cop_j.distances[neigh] < curr_dist_to_j
                            && hull[neigh] != InSet::NewlyAdded
                        {
                            hull[neigh] = InSet::NewlyAdded;
                            queue.push_back(neigh);
                        }
                    }
                }
                //change these paths from InSet::NewlyAdded to InSet::Yes
                //(to allow new paths to go through the current one)
                queue.push_back(cop_i.nearest_node);
                hull[cop_i.nearest_node] = InSet::Interieur;
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

    /// returns a point on the boundary (if there are any) and the distances to that point
    fn find_fist_boundary_point<'a>(&self, cops: &'a [Character]) -> Option<(usize, &'a [isize])> {
        for cop in cops.iter().filter(|c| c.is_active()) {
            let v = cop.nearest_node;
            if self.hull[v].on_boundary() {
                return Some((v, &cop.distances));
            }
        }
        None
    }

    /// assumes that self.hull is already up to date
    fn update_boundary(
        &mut self,
        cops: &[Character],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.boundary.clear();
        let Some((fst_inside, dist_to_fst)) = self.find_fist_boundary_point(cops) else {
            return;
        };
        self.boundary.push(fst_inside);

        update_convex_hull_boundary(&mut self.boundary, dist_to_fst, &self.hull, edges, queue);
    }

    fn update_segments(&mut self, min_cop_dist: &[isize]) {
        self.segments.clear();
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
            let segment_interior = (start + 1)..(stop - 1);
            if segment_interior.start < segment_interior.end {
                self.segments.push((start - 1)..(stop + 1));
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

/// divides vertices in convex hull into subsets where a robber escape through some boundary segment
/// is possible
/// (each segment lies between two cops on boundary, cops in interior are not considered in computation)
pub struct EscapeableNodes {
    /// intermediary value, kept to reserve allocations etc.
    /// has one entry per vertex, stores distance of that vertex to some (unspecified) node on hull boundary.
    /// which node the distance is in respect to is remembered in `self.last_write_by`
    some_boundary_dist: Vec<isize>, //one entry per vertex

    /// intermediary value, kept to save on allocations and for convinience
    /// one entry per vertex, each boundary node has an index in `ConvexHull::boundary`.
    /// if `last_write_by[vertex_i] == j` then vertex_i was visited last, when distances to boundary vertex
    /// of (boundary) index j where computed.
    last_write_by: Vec<usize>, //one entry per vertex

    /// thing we are actually interested in.
    /// has one entry per vertex. interesting are only vertices in convex hull.
    /// if vertex `v` is escapable via segment `i`, the `(i % 32)`'th bit is set in `escapable[v]`
    escapable: Vec<u32>, //one entry per vertex

    /// intermediary value
    cop_pair_hull: CopPairHullData,

    /// intermediary value, kept to reserve allocations etc.
    /// lists vertices of cop_pair_hull(s) boundaries, but only side torward outher boundary.
    /// only kept permanently to minimize allocations (and for debugging output)
    some_inner_boundaries: [Vec<usize>; 3],

    /// with respect to some escapable region, contains information if a vertex
    /// is not only part of the original escapable, but also of the
    /// shrunk region, because interior cops could endanger some parts.
    keep_escapable: Vec<Keep>, //one entry per vertex
}

impl EscapeableNodes {
    pub fn escapable(&self) -> &[u32] {
        &self.escapable
    }

    #[allow(dead_code)]
    pub fn boundary_dist(&self) -> &[isize] {
        &self.some_boundary_dist
    }

    #[allow(dead_code)]
    pub fn inner_connecting_line(&self) -> impl Iterator<Item = &usize> {
        self.some_inner_boundaries.iter().flat_map(|b| b.iter())
    }

    #[allow(dead_code)]
    pub fn owners(&self) -> &[usize] {
        &self.last_write_by
    }

    pub fn new() -> Self {
        Self {
            some_boundary_dist: Vec::new(),
            last_write_by: Vec::new(),
            escapable: Vec::new(),
            cop_pair_hull: CopPairHullData::default(),
            some_inner_boundaries: Default::default(),
            keep_escapable: Vec::new(),
        }
    }

    /// weird hybrid of region coloring and distance calculation:
    /// update distances of self.some_boundary_dist in neighborhood of queue entries,
    /// recolor region in self.last_write_by to be owner
    fn calc_small_dists(
        &mut self,
        mut select: impl FnMut(usize) -> bool,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        max_dist: isize,
        last_owner: Option<usize>,
        owner: usize,
    ) {
        let fst_in_segment = last_owner.is_none();
        let last_owner = last_owner.unwrap_or(usize::MAX);
        let mut local_queue = std::mem::take(queue);
        while let Some(v) = local_queue.pop_front() {
            let dist = self.some_boundary_dist[v];
            if dist >= max_dist {
                continue;
            }
            for n in edges.neighbors_of(v) {
                //can by specified by caller what region is selected as baseline (e.g. convex hull...)
                if !select(n) {
                    continue;
                }
                //the fist boundary vertex in a segment is allowed to mark all vertices in its reach.
                //all further vertices of that same segment can only mark hull vertices which have been reached by
                //all previous ones.
                //the vertices counted at the end are only those reached by all boundary vertices.
                if !fst_in_segment && self.last_write_by[n] != last_owner {
                    continue;
                }
                //we have been here before -> no need to mark again
                if self.last_write_by[n] == owner {
                    continue;
                }
                //it is key to never check the distance we overwrite. we don't want the minimum distance to all
                //boundary nodes, we actually want the distance to the current boundary node.
                self.some_boundary_dist[n] = dist + 1;
                local_queue.push_back(n);
                self.last_write_by[n] = owner;
            }
        }
        *queue = local_queue;
    }

    /// assumes segment was last colored by owner
    /// also assumes queue to only contain vertices of the region to add
    fn add_region_to_escapable(
        &mut self,
        owner: usize,
        marker: u32,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        debug_assert!(owner != 0);
        while let Some(v) = queue.pop_front() {
            debug_assert!(self.escapable[v] & marker != 0);
            debug_assert_eq!(self.last_write_by[v], owner);
            for n in edges.neighbors_of(v) {
                //already marked -> don't add to queue again
                if self.escapable[n] & marker != 0 {
                    debug_assert_eq!(self.last_write_by[n], owner);
                    continue;
                }
                if self.last_write_by[n] == owner {
                    self.escapable[n] |= marker;
                    queue.push_back(n);
                }
            }
        }
    }

    /// assumes escapable regions from boundary cops inwards have already been build.
    /// these regions will now be deleted in regions where third cops could otherwise interfere.
    fn consider_interior_cops(
        &mut self,
        cops: &[Character],
        hull_data: &ConvexHullData,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.keep_escapable.clear();
        self.keep_escapable.resize(edges.nr_vertices(), Keep::No);

        //owners of boundary vertices where chosen as what the vertice's index in this vector was
        //  -> new owners can start after all valid vector indices
        let mut owner = hull_data.boundary.len();
        queue.clear();
        let iter = izip!(
            hull_data.boundary_cop_vertices(),
            hull_data.safe_boundary_parts(),
            0..
        );
        for ((v_left, v_right), safe_outher_boundary, seg_nr) in iter {
            let left_cop = cops.iter().find(|c| c.nearest_node == v_left).unwrap();
            let right_cop = cops.iter().find(|c| c.nearest_node == v_right).unwrap();
            let marker = 1u32 << (seg_nr % 32);

            let endangers = |escapable: &[u32], c: &Character| {
                let v = c.nearest_node;
                let active = c.is_active();
                let distinct = v != v_left && v != v_right;
                //if cop is not at least next to region, then there must be some point on the region boundary
                //that can be reached faster by a robber starting in the region than by that outside cop.
                //this is by definition of the regions.
                let in_region = escapable[v] & marker != 0;
                let next_to_region = edges.neighbors_of(v).any(|n| escapable[n] & marker != 0);
                active && distinct && (in_region || next_to_region)
            };
            for i in 0..cops.len() {
                let cop_i = &cops[i];
                if !endangers(&self.escapable, cop_i) {
                    continue;
                }
                //note how i == j at first:
                //we generally update self.escapable one cop pair at a time, but single cops are silently also
                //handled in this routine.
                for cop_j in cops.iter().skip(i) {
                    if !endangers(&self.escapable, cop_j) {
                        continue;
                    }
                    //compare with "<=" and ">" to not skip case where i == j
                    let i_closer_eq_to_left = left_cop.distances[cop_i.nearest_node]
                        <= left_cop.distances[cop_j.nearest_node];
                    let i_closer_to_right = right_cop.distances[cop_i.nearest_node]
                        < right_cop.distances[cop_j.nearest_node];
                    if i_closer_eq_to_left == i_closer_to_right {
                        //one cop stands in front of the other -> enough to consider that first cop on its own
                        continue;
                    }
                    let (left_inner, right_inner) = if i_closer_eq_to_left {
                        (cop_i, cop_j)
                    } else {
                        (cop_j, cop_i)
                    };

                    //idea
                    //1. fill each hull of pair with KeepVertex::Perhaps and build boundaries to pair hulls
                    //2. fill region from outher boundary to newly created borders of pair hulls with KeepVertex::Yes
                    //3. prune inner boundaries
                    //4. fill each region admitting escape through one cop pair with KeepVertex::Yes
                    //5. remove markers where not KeepVertex::Yes
                    //6. reset self.keep_in_escapable

                    //step 1: paint in the hull between each cop pair with KeepVertex::Perhaps
                    debug_assert!(self.keep_escapable.iter().all(|&k| k == Keep::No));
                    let cops_line = [left_cop, left_inner, right_inner, right_cop];
                    for ((&c1, &c2), boundary) in izip!(
                        cops_line.iter().tuple_windows(),
                        &mut self.some_inner_boundaries
                    ) {
                        self.cop_pair_hull.compute_hull([c1, c2], edges, queue);
                        boundary.clear();
                        find_unordered_boundary(
                            boundary,
                            &self.cop_pair_hull.hull,
                            edges,
                            &self.cop_pair_hull.all_vertices,
                        );
                        for &v in &self.cop_pair_hull.all_vertices {
                            self.keep_escapable[v] = Keep::Perhaps;
                        }
                        self.cop_pair_hull.reset_hull(c1, edges, queue);
                    }

                    //step 2: paint region between outher boundary and pair hulls with KeepVertex::Yes
                    debug_assert!(queue.is_empty());
                    for &v in safe_outher_boundary {
                        if self.keep_escapable[v] == Keep::No {
                            self.keep_escapable[v] = Keep::Yes;
                            queue.push_back(v);
                        } else {
                            debug_assert_eq!(self.keep_escapable[v], Keep::Perhaps);
                            self.keep_escapable[v] = Keep::YesOnBoundary;
                        }
                    }
                    let paint =
                        |n: usize, keep: &[_]| keep[n] == Keep::No && hull_data.hull[n].contained();
                    edges.recolor_region_with(Keep::Yes, &mut self.keep_escapable, paint, queue);

                    //step 3: prune inner boundaries to only contain safe halfs reaching outside
                    for ((&c1, &c2), boundary) in izip!(
                        cops_line.iter().tuple_windows(),
                        &mut self.some_inner_boundaries
                    ) {
                        //get rid of vertices too close to cops at first, else retain_safe_inner_boundary can run into problems
                        boundary.retain(|&v| isize::min(c1.distances[v], c2.distances[v]) > 1);
                        retain_safe_inner_boundary(
                            &c1.distances,
                            boundary,
                            &self.keep_escapable,
                            edges,
                        );
                        debug_assert!(boundary.first().map_or(true, |&v| c1.distances[v] == 2));
                        debug_assert!(boundary.last().map_or(true, |&v| c2.distances[v] == 2));
                        debug_assert!(edges.has_path(boundary));
                    }

                    //step 4: find escapable region for each cop pair and paint it with KeepVertex::Yes
                    let mut escapable = std::mem::take(&mut self.escapable);
                    let inner_boundaries = std::mem::take(&mut self.some_inner_boundaries);
                    for ((&c1, &c2), safe_segment) in izip!(
                        cops_line.iter().tuple_windows(),
                        inner_boundaries.iter().map(|b| &b[..])
                    ) {
                        let max_escapable_dist = safe_segment.len() as isize - 1;
                        debug_assert_eq!(
                            max_escapable_dist.max(0),
                            (c1.distances[c2.nearest_node] - 4).max(0)
                        );
                        let mut last_owner = None;
                        for &v in safe_segment {
                            if last_owner.is_some() {
                                //algorithm is only correct if all vertices in
                                //safe_segment turn out to be actually safe.
                                debug_assert_eq!(Some(self.last_write_by[v]), last_owner);
                            }
                            debug_assert!(queue.is_empty());
                            queue.push_back(v);
                            self.some_boundary_dist[v] = 0;
                            self.last_write_by[v] = owner;
                            let in_hull = |n: usize| hull_data.hull[n].contained();
                            self.calc_small_dists(
                                in_hull,
                                edges,
                                queue,
                                max_escapable_dist,
                                last_owner,
                                owner,
                            );
                            last_owner = Some(owner);
                            owner += 1;
                        }

                        debug_assert!(queue.is_empty());
                        if let Some(last_owner) = last_owner {
                            let keep = &mut self.keep_escapable[..];
                            for &v in safe_segment {
                                if self.last_write_by[v] == last_owner {
                                    keep[v] = Keep::Yes;
                                    queue.push_back(v);
                                }
                            }
                            let safe = |n, _: &[_]| self.last_write_by[n] == last_owner;
                            edges.recolor_region_with(Keep::Yes, keep, safe, queue);
                        }
                    }

                    //step 5: only keep markers with KeepVertex::Yes
                    for &v in safe_outher_boundary {
                        queue.push_back(v);
                        self.keep_escapable[v] = Keep::YesButAlreadyVisited;
                    }
                    while let Some(v) = queue.pop_front() {
                        use Keep::*;
                        debug_assert_ne!(self.keep_escapable[v], Yes);
                        for n in edges.neighbors_of(v) {
                            let keep = self.keep_escapable[n];
                            if keep == Yes {
                                self.keep_escapable[n] = YesButAlreadyVisited;
                                queue.push_back(n);
                            }
                            if matches!(keep, No | Perhaps) && escapable[n] & marker != 0 {
                                escapable[n] -= marker;
                                queue.push_back(n);
                            }
                        }
                    }

                    //step 6: reset self.keep_in_escapable (use recolor to not iterate over whole array)
                    for &v in safe_outher_boundary {
                        queue.push_back(v);
                        self.keep_escapable[v] = Keep::No;
                    }
                    let keep = &mut self.keep_escapable[..];
                    edges.recolor_region_with(Keep::No, keep, |_, _| true, queue);
                    debug_assert!(keep.iter().all(|&k| k == Keep::No));

                    self.escapable = escapable;
                    self.some_inner_boundaries = inner_boundaries;
                }
            }
        }

        //because of EdgeList::recolor_region_with reasons, we always included the vertices reset here as safe robber
        //positions. this is obv. wrong
        for cop in cops.iter().filter(|c| c.is_active()) {
            self.escapable[cop.nearest_node] = 0;
            for n in edges.neighbors_of(cop.nearest_node) {
                self.escapable[n] = 0;
            }
        }
    }

    /// ignores cops in interior of hull, computes safe regions only with respect to boundary cops
    fn consider_boundary_cops(
        &mut self,
        hull_data: &ConvexHullData,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        queue.clear();
        let iter = izip!(
            hull_data.safe_boundary_indices(),
            hull_data.safe_boundary_parts(),
            0..
        );
        for (indices, vertices, seg_nr) in iter {
            let max_escapable_dist = indices.len() as isize - 1;
            debug_assert_eq!(indices.len(), vertices.len());

            //we call calc_small_dists vor every vertex in our boundary segement.
            //to speed up computation, our first and second vertex are on opposing ends.
            //future boundary vertices thus only consider inner vertices reached by both extremes,
            //which should get us some constant factor in speedup.
            let mut last_owner = None;
            let mut last_v = usize::MAX;
            let mut compute_dists_to = |owner, v| {
                debug_assert!(owner != 0);
                queue.push_back(v);
                self.some_boundary_dist[v] = 0;
                self.last_write_by[v] = owner;
                let in_hull = |x: usize| hull_data.hull[x].contained();
                self.calc_small_dists(in_hull, edges, queue, max_escapable_dist, last_owner, owner);
                last_owner = Some(owner);
                last_v = v;
            };
            let mut iter = izip!(indices.clone(), vertices);
            if let Some((owner, &v)) = iter.next_back() {
                compute_dists_to(owner, v);
            }
            for (owner, &v) in iter {
                compute_dists_to(owner, v);
            }

            let owner = last_owner.unwrap();
            debug_assert!(queue.is_empty());
            queue.push_back(last_v);
            let marker = 1u32 << (seg_nr % 32);
            self.escapable[last_v] |= marker;
            self.add_region_to_escapable(owner, marker, edges, queue);
        }
    }

    pub fn update(
        &mut self,
        cops: &[Character],
        hull_data: &ConvexHullData,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        let nr_vertices = edges.nr_vertices();
        self.escapable.clear();
        self.last_write_by.clear();
        self.some_boundary_dist.clear();
        self.escapable.resize(nr_vertices, 0);
        self.last_write_by.resize(nr_vertices, 0); //no owner is 0 (as a cop stands at pos 0 of hull boundary)
        self.some_boundary_dist.resize(nr_vertices, isize::MAX);

        self.consider_boundary_cops(hull_data, edges, queue);
        self.consider_interior_cops(cops, hull_data, edges, queue);
    }
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

        self.hull[cop_1.nearest_node] = InSet::Interieur;
        queue.push_back(cop_1.nearest_node);
        all_entries.push(cop_1.nearest_node);
        while let Some(v) = queue.pop_front() {
            let curr_dist_to_2 = cop_2.distances[v];
            for n in edges.neighbors_of(v) {
                if cop_2.distances[n] < curr_dist_to_2 && self.hull[n] == InSet::No {
                    self.hull[n] = InSet::Interieur;
                    queue.push_back(n);
                    all_entries.push(n);
                }
            }
        }
        debug_assert!(all_entries.contains(&cop_1.nearest_node));
        debug_assert!(all_entries.contains(&cop_2.nearest_node));
        self.all_vertices = all_entries;
    }

    fn reset_hull(&mut self, cop_1: &Character, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        debug_assert_eq!(self.hull.len(), edges.nr_vertices());
        debug_assert!(queue.is_empty());

        //should be faster than iterating over whole array most of the time (by far)
        self.hull[cop_1.nearest_node] = InSet::No;
        queue.push_back(cop_1.nearest_node);
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

            if retain_end < check_front {
                debug_assert_eq!(dist_at(check_front - 1), dist_at(check_front) - 1);
            }
            if check_end < boundary.len() {
                debug_assert_eq!(dist_at(check_end - 1), dist_at(check_end) - 1);
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
                take_if_unique!(|&&v| keep[v] == Keep::YesOnBoundary);
                take_if_unique!(|&&v| edges.neighbors_of(v).any(|n| keep[n] == Keep::Yes));
                take_if_unique!(|&&v| edges.neighbors_of(v).all(|n| keep[n] == Keep::Perhaps));
                //at this point something went wrong and we try to continue on a best effort basis
                if retain_end > 0 {
                    let last_v = boundary[retain_end - 1];
                    take_if_unique!(|&&v| edges.has_edge(v, last_v));
                }
                boundary[check_front]
            };
            retain_end += 1;
            check_front += check_len;
        }
    }
    boundary.truncate(retain_end);
}
