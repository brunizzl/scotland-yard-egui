
use std::collections::VecDeque;
use std::ops::Range;

use itertools::{izip, Itertools};

use egui::*;

use crate::app::character::Character;
use super::*;

pub fn find_nearest_node<F>(visible: &[bool], edges: &EdgeList, pos: Pos2, 
    mut vertex_to_screen: F, start: usize) -> (usize, f32) 
where F: FnMut(usize) -> Pos2
{       
    let potential = |v:usize| {
        let v_screen_pos = vertex_to_screen(v);
        let dist_2d = (v_screen_pos - pos).length_sq();
        let backface_penalty = 10.0 * (!visible[v]) as isize as f32;
        dist_2d + backface_penalty
    };
    edges.find_local_minimum(potential, start)
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

    pub fn boundary_interieur_indices<'a>(&'a self) -> impl ExactSizeIterator<Item = Range<usize>> + 'a + Clone {
        self.segments.iter().map(|seg| {
            let interior = (seg.start + 2)..(seg.end - 2);
            debug_assert!(!interior.is_empty());
            interior
        })
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of both boundary cops and the interior of the boundary.
    pub fn boundary_interieurs<'a>(&'a self) -> impl ExactSizeIterator<Item = &'a [usize]> + 'a + Clone {
        self.boundary_interieur_indices().map(|interior| &self.boundary[interior])
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    /// iterator contains vertices of both boundary cops for a given boundary.
    /// each boundary cop should thus appear twice.
    pub fn boundary_cop_vertices<'a>(&'a self) -> impl ExactSizeIterator<Item = (usize, usize)> + 'a + Clone {
        self.segments.iter().map(|seg| {
            (self.boundary[seg.start], self.boundary[seg.end - 1])
        })
    }

    fn update_inside(&mut self, cops: &[Character], edges: &EdgeList, queue: &mut VecDeque<usize>, 
        vertices_outside_hull: &[usize]) 
    {
        queue.clear();

        self.hull.clear();
        self.hull.resize(edges.nr_vertices(), InSet::Perhaps);
        let hull = &mut self.hull[..];
        for i in 0..cops.len() {
            let cop_i = &cops[i];
            if !cop_i.on_node {
                continue;
            }
            for cop_j in cops[(i + 1)..].iter().filter(|c| c.on_node) {                
                //walk from cop i to cop j on all shortest paths
                hull[cop_i.nearest_node] = InSet::NewlyAdded;
                queue.push_back(cop_i.nearest_node);
                while let Some(node) = queue.pop_front() {
                    let curr_dist_to_j = cop_j.distances[node];
                    for neigh in edges.neighbors_of(node) {
                        if cop_j.distances[neigh] < curr_dist_to_j && hull[neigh] != InSet::NewlyAdded {
                            hull[neigh] = InSet::NewlyAdded;
                            queue.push_back(neigh);
                        }
                    }
                }
                //change these paths from InSet::NewlyAdded to InSet::Yes
                //(to allow new paths to go through the current one)
                queue.push_back(cop_i.nearest_node);
                hull[cop_i.nearest_node] = InSet::Yes;
                edges.recolor_region((InSet::NewlyAdded, InSet::Yes), hull, queue);
            }
        }
        //color outside as InSet::No (note: this might miss some edge cases; best to not place cops at rim)
        for &p in vertices_outside_hull {
            if hull[p] == InSet::Perhaps {
                hull[p] = InSet::No;
                queue.push_back(p);
            }
        }
        edges.recolor_region((InSet::Perhaps, InSet::No), hull, queue);

        //color remaining InSet::Perhaps as InSet::Yes
        for x in hull {
            if *x == InSet::Perhaps {
                *x = InSet::Yes;
            }
        }
        //mark boundary as such
        let hull = &mut self.hull[..];
        for (v, mut neighs) in izip!(0.., edges.neighbors()) {
            if hull[v].inside() && neighs.any(|n| hull[n].outside()) {
                hull[v] = InSet::OnBoundary;
            }
        }
    }

    fn find_fist_boundary_point(&self, cops: &[Character]) -> Option<usize> {
        for cop in cops.iter().filter(|c| c.on_node) {
            let v = cop.nearest_node;
            if self.hull[v].on_boundary() {
                return Some(v);
            }
        }
        None
    }

    /// assumes that self.hull is already up to date
    fn update_boundary(&mut self, cops: &[Character], edges: &EdgeList, queue: &mut VecDeque<usize>) {
        let Some(fst_inside) = self.find_fist_boundary_point(cops) else { return; };

        let mut potential_next = std::mem::take(queue);
        let mut boundary = std::mem::take(&mut self.boundary);
        boundary.clear();
        boundary.push(fst_inside);
        let mut curr_inside = fst_inside;
        let mut last_inside = usize::MAX;
        'find_next: loop {
            if boundary.len() > edges.nr_vertices() {
                break;
            }
            let (boundary_len, boundary_end) = {
                let len = boundary.len();
                const VIEW_LEN: usize = 8;
                let range_start = if len < VIEW_LEN { 0 } else { len - VIEW_LEN };
                (len, &boundary[range_start..])
            };

            for n in edges.neighbors_of(curr_inside) {
                if n == last_inside {
                    continue;
                }
                if n == fst_inside { //we did it boys :)
                    potential_next.clear();
                    potential_next.push_back(n);
                    break;
                }
                //to be completely correct, this should cover all of boundary.
                //the common cases are however correctly handeled here, the others are handled in the match below.
                if boundary_end.contains(&n) {
                    continue;
                }
                if !self.hull[n].on_boundary() {
                    continue;
                }
                potential_next.push_back(n);
            }

            let next_inside = match (potential_next.len(), boundary_len) {
                (0, 1) => fst_inside, //we are in the hull but have no neighbors inside -> the hull is only us.
                (1, _) => potential_next[0], //common case
                (2, 1) => potential_next[0], //the first boundary point decides the direction.
                (0, _) => 'take_oldest_neigh: loop {
                    //take the option visited the longest time ago (note: we assume to 
                    //hit this case for only very few hulls (those of thickness 1), 
                    //thus don't care about the O(boundary.len()) search.)
                    for old in &boundary { 
                        if edges.neighbors_of(curr_inside).contains(old) {
                            break 'take_oldest_neigh *old;
                        }
                    }
                    unreachable!(); //last_inside is guaranteed to break the for loop.
                },
                (_, _) => 'take_one_sharing_side: loop {
                    for &next in potential_next.iter() {
                        //choose the vertex wich shares neighbors with curr that are outside the hull.
                        //(only guaranteed to happen in triangulations)
                        //note: this is not a sufficient condition in some edgecases, where we only connect two cops
                        //in a random triangulation. this path may have multiple disjoint sections of width 1
                        //and it is not considered here, which side is taken on exit of such a section.
                        //i dont't care though. that case is uninteresting anyway.
                        let is_neigh_of_curr_and_outside_hull = |v| edges
                            .neighbors_of(curr_inside)
                            .filter(|&n| self.hull[n].outside())
                            .contains(&v);
                        if edges.neighbors_of(next).any(is_neigh_of_curr_and_outside_hull) {
                            break 'take_one_sharing_side next;
                        }
                    }
                    break 'find_next;
                },
            };
            potential_next.clear();
            last_inside = curr_inside;
            curr_inside = next_inside;
            boundary.push(next_inside);
            if next_inside == fst_inside { //happy exit :)
                self.boundary = boundary;
                return;
            }            
        }
        boundary.clear(); //sad exit :(
        potential_next.clear();
        self.boundary = boundary;
        *queue = potential_next;
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

    pub fn update(&mut self, cops: &[Character], edges: &EdgeList, min_cop_dist: &[isize], queue: &mut VecDeque<usize>, 
        vertices_outside_hull: &[usize]) 
    {
        self.update_inside(cops, edges, queue, vertices_outside_hull);
        self.update_boundary(cops, edges, queue);
        self.update_segments(min_cop_dist);
    }
}


#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum KeepVertex { No, Yes, YesButAlreadyVisited }

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
    last_write_by: Vec<usize>,  //one entry per vertex

    /// thing we are actually interested in. 
    /// has one entry per vertex. interesting are only vertices in convex hull.
    /// if vertex `v` is escapable via segment `i`, the `(i % 32)`'th bit is set in `escapable[v]`
    escapable: Vec<u32>, //one entry per vertex

    /// the shortest path between some pair of cops with the pointwise smallest possible values
    /// in self.some_boundary_dist
    inner_connecting_line: Vec<usize>, //indices of vertices

    /// with respect to some escapable region, contains information if a vertex 
    /// is not only part of the original escapable, but also of the
    /// shrunk region, because interior cops could endanger some parts.
    keep_in_escapable: Vec<KeepVertex> //one entry per vertex
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
    pub fn inner_connecting_line(&self) -> &[usize] {
        &self.inner_connecting_line
    }

    #[allow(dead_code)]
    pub fn owners(&self) -> &[usize] {
        &self.last_write_by
    }

    #[allow(dead_code)]
    pub fn keep_in_escapable(&self) -> &[KeepVertex] {
        &self.keep_in_escapable
    }

    pub fn new() -> Self {
        Self { 
            some_boundary_dist: Vec::new(), 
            boundary_segment_dist: Vec::new(),
            last_write_by: Vec::new(),
            escapable: Vec::new(),
            inner_connecting_line: Vec::new(),
            keep_in_escapable: Vec::new(),
        }
    }

    
    /// stores dist to closest boundary vertex of region owned by owner in `self.some_boundary_dist`.
    fn calc_dist_to_boundary_segement(&mut self, boundary: &[usize], hull: &[InSet], edges: &EdgeList, 
        queue: &mut VecDeque<usize>)
    {
        debug_assert_eq!(self.some_boundary_dist.len(), edges.nr_vertices());

        queue.clear();
        let v0 = boundary[0];
        self.boundary_segment_dist[v0] = isize::MAX;
        queue.push_back(v0);
        edges.recolor_region_with(
            isize::MAX, 
            &mut self.boundary_segment_dist, 
            |_, n, _| hull[n].inside(), 
            queue
        );

        for &v in boundary {
            self.boundary_segment_dist[v] = 0;
            queue.push_back(v);
        }
        edges.calc_distances_to_with(
            &mut self.boundary_segment_dist, 
            |v, _| hull[v].inside(), 
            queue
        );
    }

    /// weird hybrid of region coloring and distance calculation:
    /// update distances of self.some_boundary_dist in neighborhood of queue entries,
    /// recolor region in self.last_write_by to be owner
    fn calc_small_dists(&mut self, mut select: impl FnMut(usize) -> bool, edges: &EdgeList, 
        queue: &mut VecDeque<usize>, max_dist: isize, last_owner: Option<usize>, owner: usize) 
    {
        let fst_in_segment = last_owner == None;
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
    fn add_region_to_escapable(&mut self, owner: usize, marker: u32, edges: &EdgeList, queue: &mut VecDeque<usize>) {
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

    /// assumes cop_1.nearest_node to be current last element in path, will extend path to cop_2 on shortest route minimizing
    /// dist_to_outside for each step
    fn walk_outher_shortest_path(cop_1: &Character, cop_2: &Character, dist_to_outside: &[isize], 
        path: &mut Vec<usize>, edges: &EdgeList) 
    {
        debug_assert!(cop_1.on_node && cop_2.on_node);
        debug_assert_eq!(Some(&cop_1.nearest_node), path.last());
        debug_assert_eq!(cop_1.distances[cop_1.nearest_node], 0);
        debug_assert_eq!(cop_2.distances[cop_2.nearest_node], 0);
        let mut curr_v = cop_1.nearest_node;
        while curr_v != cop_2.nearest_node {
            let mut best_v = usize::MAX;
            let mut best_dist_to_outside = isize::MAX;
            let mut best_dist_to_cop_2 = isize::MAX;
            for n in edges.neighbors_of(curr_v) {
                use std::cmp::Ordering::*;
                let ord = cop_2.distances[n].cmp(&best_dist_to_cop_2);
                if ord == Less || (ord == Equal && dist_to_outside[n] < best_dist_to_outside) {
                    best_v = n;
                    best_dist_to_outside = dist_to_outside[n];
                    best_dist_to_cop_2 = cop_2.distances[n];
                }
            }
            debug_assert!(best_v != usize::MAX);
            path.push(best_v);
            curr_v = best_v;
        }
    }

    

    /// assumes escapable regions from boundary cops inwards have already been build.
    /// these regions will now be deleted in regions where third cops could otherwise interfere.
    fn consider_interior_cops(&mut self, cops: &[Character], hull_data: &ConvexHullData, edges: &EdgeList, queue: &mut VecDeque<usize>)
    {        
        self.boundary_segment_dist.clear();
        self.boundary_segment_dist.resize(edges.nr_vertices(), 0);
        self.keep_in_escapable.clear();
        self.keep_in_escapable.resize(edges.nr_vertices(), KeepVertex::No);

        let mut owner = hull_data.boundary.len();
        queue.clear();
        let iter = izip!(
            hull_data.boundary_interieur_indices(), 
            hull_data.boundary_cop_vertices(), 
            hull_data.boundary_interieurs(), 
            0..);
        for (indices, (v_left, v_right), boundary_interior, seg_nr) in iter {
            debug_assert_eq!(indices.len(), boundary_interior.len());
            self.calc_dist_to_boundary_segement(boundary_interior, hull_data.hull(), edges, queue);

            let left_cop = cops.iter().find(|c| c.nearest_node == v_left).unwrap();
            let right_cop = cops.iter().find(|c| c.nearest_node == v_right).unwrap();
            let marker = 1u32 << (seg_nr % 32);

            let endangers = |escapable: &[u32], c: &Character| {
                let v = c.nearest_node;
                let active = c.on_node;
                let distinct = v != v_left && v != v_right;
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
                for j in i..cops.len() {
                    let cop_j = &cops[j];
                    if !endangers(&self.escapable, cop_j) {
                        continue;
                    }
                    let i_closer_eq_to_left = left_cop.distances[cop_i.nearest_node] <= left_cop.distances[cop_j.nearest_node];
                    let i_closer_to_right = right_cop.distances[cop_i.nearest_node] < right_cop.distances[cop_j.nearest_node];
                    if i_closer_eq_to_left == i_closer_to_right {
                        //one cop stands in front of the other -> enough to consider that first cop on its own
                        continue;
                    }
                    let (left_inner, right_inner) = if i_closer_eq_to_left { 
                        (cop_i, cop_j)
                    } else {
                        (cop_j, cop_i)
                    };

                    //idea: connect the four cops by paths. these paths will split the current region of self.escapable
                    //into two parts: one is in front of the inner cops. if the robber is in that part, he can just 
                    //escape straight to the boundary s if the inner cops where not present at all.
                    //if the robber is in the other part, he is yet to cross the just constructed line. along this line,
                    //every section is now guarded by the two cops c1 and c2 and can be crossed if the same conditions hold
                    //as for crossing the boundary between left_cop and right_cop.
                    let mut path = std::mem::take(&mut self.inner_connecting_line);
                    let mut escapable = std::mem::take(&mut self.escapable);
                    for (&c1, &c2) in [left_cop, left_inner, right_inner, right_cop].iter().tuple_windows() {
                        path.clear();
                        path.push(c1.nearest_node);
                        Self::walk_outher_shortest_path(
                            c1, c2, &self.boundary_segment_dist, &mut path, edges);
                        for &v in &path {
                            self.keep_in_escapable[v] = KeepVertex::Yes;
                        }
                        path.retain(|&v| isize::min(c1.distances[v], c2.distances[v]) > 1);
                        if path.is_empty() {
                            continue;
                        }
                        let max_escapable_dist = path.len() as isize - 1;
                        let mut last_owner = None;
                        debug_assert!(queue.is_empty());
                        for &v in &path {
                            queue.push_back(v);
                            self.some_boundary_dist[v] = 0;
                            self.last_write_by[v] = owner;
                            let in_hull = |x:usize| hull_data.hull[x].inside();
                            self.calc_small_dists(in_hull, edges, queue, max_escapable_dist, last_owner, owner);
                            last_owner = Some(owner);
                            owner += 1;
                        }
                        let last_owner = last_owner.unwrap();
                        for &v in &path {
                            debug_assert_eq!(self.last_write_by[v], last_owner);
                            self.keep_in_escapable[v] = KeepVertex::Yes;
                            queue.push_back(v);  
                        }
                        edges.recolor_region_with(KeepVertex::Yes, 
                            &mut self.keep_in_escapable, 
                            |v, n, _| self.last_write_by[n] == last_owner 
                                && self.boundary_segment_dist[v] < self.boundary_segment_dist[n], 
                            queue
                        );
                    }
                    debug_assert!(queue.is_empty());
                    for &v in boundary_interior {
                        if self.keep_in_escapable[v] != KeepVertex::Yes {
                            queue.push_back(v);
                            self.keep_in_escapable[v] = KeepVertex::Yes;
                        }
                    }
                    edges.recolor_region_with(KeepVertex::Yes, &mut self.keep_in_escapable, |_, n, _| hull_data.hull[n].inside(), queue);
                    debug_assert!(self.keep_in_escapable.iter().all(|k| matches!(k, KeepVertex::No | KeepVertex::Yes)));

                    //we now want to keep the intersection of self.keep_in_escapable and the old region marked by marker
                    //in self.escapable.
                    //at first, we shrink the region in escapable, then we delete the entries ins self.keep_in_escapable.
                    for &v in boundary_interior {
                        queue.push_back(v);
                        self.keep_in_escapable[v] = KeepVertex::YesButAlreadyVisited;
                    }
                    while let Some(v) = queue.pop_front() {
                        debug_assert!(matches!(self.keep_in_escapable[v], KeepVertex::No | KeepVertex::YesButAlreadyVisited));
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
                    edges.recolor_region_with(KeepVertex::No, &mut self.keep_in_escapable, |_, _, _| true, queue);
                    debug_assert!(self.keep_in_escapable.iter().all(|&k| k == KeepVertex::No));

                    self.escapable = escapable;
                    self.inner_connecting_line = path;
                }
            }
        }
        
        //because of EdgeList::recolor_region_with reasons, we always included the vertices reset here as save robber
        //positions. this is obv. wrong
        for cop in cops.iter().filter(|c| c.on_node) {
            self.escapable[cop.nearest_node] = 0;
            for n in edges.neighbors_of(cop.nearest_node) {
                self.escapable[n] = 0;
            }
        }
    }

    /// ignores cops in interior of hull, computes safe regions only with respect to boundary cops
    fn consider_boundary_cops(&mut self, hull_data: &ConvexHullData, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        let iter = izip!(
            hull_data.boundary_interieur_indices(), 
            hull_data.boundary_interieurs(), 
            0..);
        for (indices, vertices, seg_nr) in iter {
            let max_escapable_dist = indices.len() as isize - 1;
            debug_assert_eq!(indices.len(), vertices.len());
    
            //we call calc_small_dists vor every vertex in our boundary segement.
            //to speed up computation, our first and second vertex are on opposing ends. 
            //future boundary vertices thus only consider inner vertices reached by both extremes,
            //which should get us some constant factor in speedup.
            let mut last_i = None;
            let mut last_v = usize::MAX;
            let mut compute_dists_to = |i, v| {
                debug_assert!(i != 0);
                queue.push_back(v);
                self.some_boundary_dist[v] = 0;
                self.last_write_by[v] = i;
                let in_hull = |x:usize| hull_data.hull[x].inside();
                self.calc_small_dists(in_hull, edges, queue, max_escapable_dist, last_i, i);
                last_i = Some(i);
                last_v = v;
            };
            let mut iter = izip!(indices.clone(), vertices);
            if let Some((i, &v)) = iter.next_back() {
                compute_dists_to(i, v);
            }
            for (i, &v) in iter {
                compute_dists_to(i, v);
            }
    
            let owner = last_i.unwrap();
            debug_assert!(queue.is_empty());
            queue.push_back(last_v);
            let marker = 1u32 << (seg_nr % 32);
            self.escapable[last_v] |= marker;
            self.add_region_to_escapable(owner, marker, edges, queue);
        }
    }

    pub fn update(&mut self, cops: &[Character], hull_data: &ConvexHullData, edges: &EdgeList, queue: &mut VecDeque<usize>) {
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

