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
    // or the hull is not actually a convex hull, this variable can safe us.
    let mut snd_last_outside = usize::MAX;
    let fst_outside = neighbors_of(fst_inside).find(|&n| hull[n].outside()).unwrap();
    let mut last_outside = fst_outside;
    let Some(mut curr_outside) = neighbors_of(fst_inside)
        .find(|&n| hull[n].outside() && neighbors_of(n).contains(&fst_outside))
    else {
        //can fail if object we want to find the boundary of is not actually a convex hull.
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
                && neighbors_of(curr_inside).contains(&n)
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
                && neighbors_of(curr_outside).contains(&n)
        });
        if let Some(next_inside) = inside_step.or_else(|| {
            // special case if hull has thickness 1 and curr_inside is an extreme point
            (!change
                && neighbors_of(curr_outside).contains(&last_inside)
                && neighbors_of(last_outside).contains(&curr_inside))
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

    pub fn boundary_interieur_indices(
        &self,
    ) -> impl ExactSizeIterator<Item = Range<usize>> + '_ + Clone {
        self.segments.iter().map(|seg| {
            let interior = (seg.start + 2)..(seg.end - 2);
            debug_assert!(!interior.is_empty());
            interior
        })
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of the interior of the boundary, e.g. cops and neighboring vertices cut away from begin and end.
    pub fn boundary_interieurs(&self) -> impl ExactSizeIterator<Item = &'_ [usize]> + '_ + Clone {
        self.boundary_interieur_indices().map(|interior| &self.boundary[interior])
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
enum KeepVertex {
    No,
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

    /// distance of interior points to some unspecified segment on boundary
    boundary_segment_dist: Vec<isize>, //one entry per vertex

    /// intermediary value, kept to save on allocations and for convinience
    /// one entry per vertex, each boundary node has an index in `ConvexHull::boundary`.
    /// if `last_write_by[vertex_i] == j` then vertex_i was visited last, when distances to boundary vertex
    /// of (boundary) index j where computed.
    last_write_by: Vec<usize>, //one entry per vertex

    /// thing we are actually interested in.
    /// has one entry per vertex. interesting are only vertices in convex hull.
    /// if vertex `v` is escapable via segment `i`, the `(i % 32)`'th bit is set in `escapable[v]`
    escapable: Vec<u32>, //one entry per vertex

    cop_pair_hull: CopPairHullData,

    /// with respect to some escapable region, contains information if a vertex
    /// is not only part of the original escapable, but also of the
    /// shrunk region, because interior cops could endanger some parts.
    keep_in_escapable: Vec<KeepVertex>, //one entry per vertex
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
    pub fn boundary_segment_dist(&self) -> &[isize] {
        &self.boundary_segment_dist
    }

    #[allow(dead_code)]
    pub fn inner_connecting_line(&self) -> &[usize] {
        &self.cop_pair_hull.boundary
    }

    #[allow(dead_code)]
    pub fn owners(&self) -> &[usize] {
        &self.last_write_by
    }

    pub fn new() -> Self {
        Self {
            some_boundary_dist: Vec::new(),
            boundary_segment_dist: Vec::new(),
            last_write_by: Vec::new(),
            escapable: Vec::new(),
            cop_pair_hull: CopPairHullData {
                hull: Vec::new(),
                boundary: Vec::new(),
                boundary_segments: [0..0, 0..0],
            },
            keep_in_escapable: Vec::new(),
        }
    }

    /// stores dist to closest boundary vertex of region owned by owner in `self.some_boundary_dist`.
    fn calc_dist_to_boundary_segement(
        &mut self,
        boundary: &[usize],
        hull: &[InSet],
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        debug_assert_eq!(self.some_boundary_dist.len(), edges.nr_vertices());

        queue.clear();
        let v0 = boundary[0];
        self.boundary_segment_dist[v0] = isize::MAX;
        queue.push_back(v0);
        edges.recolor_region_with(
            isize::MAX,
            &mut self.boundary_segment_dist,
            |_, n, _| hull[n].contained(),
            queue,
        );

        for &v in boundary {
            self.boundary_segment_dist[v] = 0;
            queue.push_back(v);
        }
        edges.calc_distances_to_with(
            &mut self.boundary_segment_dist,
            |v, _| hull[v].contained(),
            queue,
        );
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
        self.boundary_segment_dist.clear();
        self.boundary_segment_dist.resize(edges.nr_vertices(), 0);
        self.keep_in_escapable.clear();
        self.keep_in_escapable.resize(edges.nr_vertices(), KeepVertex::No);

        //owners of boundary vertices where chosen as what the vertice's index in this vector was
        //  -> new owners can start after all valid vector indices
        let mut owner = hull_data.boundary.len();
        queue.clear();
        let iter = izip!(
            hull_data.boundary_interieur_indices(),
            hull_data.boundary_cop_vertices(),
            hull_data.boundary_interieurs(),
            0..
        );
        for (indices, (v_left, v_right), boundary_interior, seg_nr) in iter {
            debug_assert_eq!(indices.len(), boundary_interior.len());
            self.calc_dist_to_boundary_segement(boundary_interior, hull_data.hull(), edges, queue);

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

                    //TODO:
                    //consider multi step process:
                    //1. fill each hull of pair with KeepVertex::Yes (don't need hull boundary for that)
                    //2. fill region from outher boundary to newly created borders of pair hulls with KeepVertex::Yes
                    //3. fill each region admitting escape through one cop pair with KeepVertex::Yes
                    //4. remove markers where not KeepVertex::Yes

                    //idea: connect the four cops by paths. these paths will split the current region of self.escapable
                    //into two parts: one is in front of the inner cops. if the robber is in that part, he can just
                    //escape straight to the boundary as if the inner cops where not present at all.
                    //if the robber is in the other part, he is yet to cross the just constructed line. along this line,
                    //every section is now guarded by the two cops c1 and c2 and can be crossed if the same conditions hold
                    //as for crossing the boundary between left_cop and right_cop.
                    let mut escapable = std::mem::take(&mut self.escapable);
                    for (&c1, &c2) in
                        [left_cop, left_inner, right_inner, right_cop].iter().tuple_windows()
                    {
                        self.cop_pair_hull.compute_boundary_from_cops(c1, c2, edges, queue);
                        let pair_boundary = std::mem::take(&mut self.cop_pair_hull.boundary);
                        let pair_segments = self.cop_pair_hull.boundary_segments.clone();
                        debug_assert_eq!(pair_segments[0].len(), pair_segments[1].len());
                        let max_escapable_dist = pair_segments[0].len() as isize - 5;
                        debug_assert_eq!(max_escapable_dist.max(0), (c1.distances[c2.nearest_node] - 4).max(0));

                        let mut last_owner = None;
                        debug_assert!(queue.is_empty());
                        let mut compute_small_dists_for = |v| {
                            queue.push_back(v);
                            self.some_boundary_dist[v] = 0;
                            self.last_write_by[v] = owner;
                            let in_hull = |n: usize| escapable[n] & marker != 0;
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
                        };
                        if max_escapable_dist >= 0 {
                            for segment in pair_segments.clone() {
                                let safe_segment = (segment.start + 2)..(segment.end - 2);
                                let path = &pair_boundary[safe_segment];
                                debug_assert!(path
                                    .iter()
                                    .all(|&v| isize::min(c1.distances[v], c2.distances[v]) > 1));

                                //same as in consider_boundary_cops: we start with the opposing ends to get some constant speedup
                                //as we shrink the space of escapable vertices faster
                                compute_small_dists_for(path[0]);
                                for &v in path[1..].iter().rev() {
                                    compute_small_dists_for(v);
                                }
                            }

                            let last_owner = last_owner.unwrap();
                            for segment in pair_segments.clone() {
                                let safe_segment = (segment.start + 2)..(segment.end - 2);
                                let path = &pair_boundary[safe_segment];

                                for &v in path {
                                    if self.last_write_by[v] == last_owner {
                                        self.keep_in_escapable[v] = KeepVertex::Yes;
                                        queue.push_back(v);
                                    }
                                }
                                edges.recolor_region_with(
                                    KeepVertex::Yes,
                                    &mut self.keep_in_escapable,
                                    |_, n, _| self.last_write_by[n] == last_owner,
                                    queue,
                                );
                            }
                        }
                        self.cop_pair_hull.boundary = pair_boundary;
                    }
                    //plug holes s.t. recoloring below can't recolor everything
                    for c in [left_cop, left_inner, right_inner, right_cop] {
                        for n in edges.neighbors_of(c.nearest_node) {
                            self.keep_in_escapable[n] = KeepVertex::Yes;
                        }
                    }
                    debug_assert!(queue.is_empty());
                    for &v in boundary_interior {
                        if self.keep_in_escapable[v] != KeepVertex::Yes {
                            queue.push_back(v);
                            self.keep_in_escapable[v] = KeepVertex::Yes;
                        }
                    }
                    //ascent from old boundary up to regions found safe just now
                    edges.recolor_region_with(
                        KeepVertex::Yes,
                        &mut self.keep_in_escapable,
                        |_, n, _| escapable[n] & marker != 0,
                        queue,
                    );

                    //we now want to keep the intersection of self.keep_in_escapable and the old region marked by marker
                    //in self.escapable.
                    //at first, we shrink the region in escapable, then we delete the entries ins self.keep_in_escapable.
                    for &v in boundary_interior {
                        queue.push_back(v);
                        self.keep_in_escapable[v] = KeepVertex::YesButAlreadyVisited;
                    }
                    while let Some(v) = queue.pop_front() {
                        debug_assert!(matches!(
                            self.keep_in_escapable[v],
                            KeepVertex::No | KeepVertex::YesButAlreadyVisited
                        ));
                        for n in edges.neighbors_of(v) {
                            let keep = self.keep_in_escapable[n];
                            if keep == KeepVertex::Yes {
                                self.keep_in_escapable[n] = KeepVertex::YesButAlreadyVisited;
                                queue.push_back(n);
                            }
                            if keep == KeepVertex::No && escapable[n] & marker != 0 {
                                escapable[n] -= marker;
                                queue.push_back(n);
                            }
                        }
                    }
                    //reset for next shrinkage
                    for &v in boundary_interior {
                        queue.push_back(v);
                        self.keep_in_escapable[v] = KeepVertex::No;
                    }
                    edges.recolor_region_with(
                        KeepVertex::No,
                        &mut self.keep_in_escapable,
                        |_, _, _| true,
                        queue,
                    );
                    debug_assert!(self.keep_in_escapable.iter().all(|&k| k == KeepVertex::No));

                    self.escapable = escapable;
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
            hull_data.boundary_interieur_indices(),
            hull_data.boundary_interieurs(),
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
struct CopPairHullData {
    hull: Vec<InSet>,     //one entry per vertex
    boundary: Vec<usize>, //indices of boundary vertices
    boundary_segments: [std::ops::Range<usize>; 2],
}

impl CopPairHullData {
    fn compute_hull(
        &mut self,
        cop_1: &Character,
        cop_2: &Character,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.hull.resize(edges.nr_vertices(), InSet::No);
        debug_assert!(self.hull.iter().all(|&x| x == InSet::No));
        debug_assert!(queue.is_empty());

        let mut all_entries = std::mem::take(&mut self.boundary);
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

        for &v in &all_entries {
            if edges.neighbors_of(v).any(|n| self.hull[n].outside()) {
                self.hull[v] = InSet::OnBoundary;
            }
        }
        all_entries.clear();
        self.boundary = all_entries;
    }

    /// computes the all-around boundary, only keeps half that has a lower min potential in its interior
    fn compute_boundary(
        &mut self,
        cop_1: &Character,
        cop_2: &Character,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.boundary.clear();
        self.boundary.push(cop_1.nearest_node);
        self.boundary_segments = [0..0, 0..0];

        let success = update_convex_hull_boundary(
            &mut self.boundary,
            &cop_1.distances,
            &self.hull,
            edges,
            queue,
        );
        if !success {
            self.boundary.clear();
            return;
        }

        let cop_2_boundary_pos =
            self.boundary.iter().position(|&v| v == cop_2.nearest_node).unwrap();

        let fst_segment = 0..(cop_2_boundary_pos + 1);
        let snd_segment = cop_2_boundary_pos..(self.boundary.len());
        let segment_len = fst_segment.len();
        if segment_len < 4 {
            self.boundary.clear();
            return;
        }
        debug_assert_eq!(segment_len, snd_segment.len()); //follows directly from definition of convex hull
        self.boundary_segments = [fst_segment, snd_segment];
    }

    fn reset_hull(&mut self, cop_1: &Character, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        debug_assert_eq!(self.hull.len(), edges.nr_vertices());
        debug_assert!(queue.is_empty());

        self.hull[cop_1.nearest_node] = InSet::No;
        queue.push_back(cop_1.nearest_node);
        edges.recolor_region_with(InSet::No, &mut self.hull, |_, _, _| true, queue);

        debug_assert!(self.hull.iter().all(|&x| x == InSet::No));
    }

    /// computes hull, computes boundary, resets hull
    fn compute_boundary_from_cops(
        &mut self,
        cop_1: &Character,
        cop_2: &Character,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        debug_assert!(cop_1.is_active());
        debug_assert!(cop_2.is_active());

        self.compute_hull(cop_1, cop_2, edges, queue);
        self.compute_boundary(cop_1, cop_2, edges, queue);
        self.reset_hull(cop_1, edges, queue);
    }
}
