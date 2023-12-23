
use std::collections::VecDeque;
use std::ops::Range;

use itertools::izip;

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

#[derive(serde::Deserialize, serde::Serialize)]
pub struct ConvexHull {
    /// one entry per vertex in graph
    inside: Vec<InSet>, 
    /// enumerates vertices which are on boundary
    boundary: Vec<usize>, 
    /// indexes in self.boundary, each segment is delimited by two cops on boundary.
    /// the cops' vertices and the vertices neighboring these cops are excluded
    segment_interiors: Vec<Range<usize>>, 
}

impl ConvexHull {
    pub fn new() -> Self {
        Self {
            inside: Vec::new(),
            boundary: Vec::new(),
            segment_interiors: Vec::new(),
        }
    }

    pub fn inside(&self) -> &[InSet] {
        &self.inside
    }

    #[allow(dead_code)]
    pub fn boundary(&self) -> &[usize] {
        &self.boundary
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    pub fn boundary_segments<'a>(&'a self) -> impl Iterator<Item = &'a [usize]> + 'a {
        self.segment_interiors.iter().map(|seg| &self.boundary[seg.clone()])
    }

    fn update_inside(&mut self, cops: &[Character], edges: &EdgeList, queue: &mut VecDeque<usize>, 
        vertices_outside_hull: &[usize]) 
    {
        queue.clear();

        self.inside.clear();
        self.inside.resize(edges.nr_vertices(), InSet::Perhaps);
        let in_hull = &mut self.inside[..];
        for i in 0..cops.len() {
            let cop_i = &cops[i];
            if !cop_i.on_node {
                continue;
            }
            for cop_j in cops[(i + 1)..].iter().filter(|c| c.on_node) {                
                //walk from cop i to cop j on all shortest paths
                in_hull[cop_i.nearest_node] = InSet::NewlyAdded;
                queue.push_back(cop_i.nearest_node);
                while let Some(node) = queue.pop_front() {
                    let curr_dist_to_j = cop_j.distances[node];
                    for neigh in edges.neighbors_of(node) {
                        if cop_j.distances[neigh] < curr_dist_to_j && in_hull[neigh] != InSet::NewlyAdded {
                            in_hull[neigh] = InSet::NewlyAdded;
                            queue.push_back(neigh);
                        }
                    }
                }
                //change these paths from InSet::NewlyAdded to InSet::Yes
                //(to allow new paths to go through the current one)
                queue.push_back(cop_i.nearest_node);
                in_hull[cop_i.nearest_node] = InSet::Yes;
                edges.recolor_region((InSet::NewlyAdded, InSet::Yes), in_hull, queue);
            }
        }
        //color outside as InSet::No (note: this might miss some edge cases; best to not place cops at rim)
        for &p in vertices_outside_hull {
            if in_hull[p] == InSet::Perhaps {
                in_hull[p] = InSet::No;
                queue.push_back(p);
            }
        }
        edges.recolor_region((InSet::Perhaps, InSet::No), in_hull, queue);

        //color remaining InSet::Perhaps as InSet::Yes
        for x in in_hull {
            if *x == InSet::Perhaps {
                *x = InSet::Yes;
            }
        }
    }

    fn update_boundary(&mut self, cops: &[Character], edges: &EdgeList) {
        //idea: walk outside hull and look inside, but actually store the inside nodes
        //this approach will fail, if one cop stands at the border of the map, but this is something
        //2d no happening in the intresting real 3d case anyway.
        let mut boundary = std::mem::take(&mut self.boundary);
        boundary.clear();
        let fst_outside = 'search_start_vertex: loop {
            for cop in cops.iter().filter(|c| c.on_node) {
                for n in edges.neighbors_of(cop.nearest_node) {
                    if !self.inside[n].yes() {
                        boundary.push(cop.nearest_node);
                        break 'search_start_vertex n;
                    }
                }
            }
            return;
        };
        let mut last_outside = usize::MAX;
        let mut curr_outside = fst_outside;
        let mut steps_since_boundary_change = 0;
        loop {
            let mut change_outside = false;
            let mut change_boundary = false;
            for n in edges.neighbors_of(curr_outside) {
                //find vertices on hull boundary
                if self.inside[n].yes() {
                    let len = boundary.len();
                    let range_min = if len < 10 { 0 } else { len - 10 };
                    if !boundary[range_min..].contains(&n) {
                        boundary.push(n);
                        change_boundary = true;
                    }
                }
                else { //find next vertex just outside hull
                    let still_searching = !change_outside;
                    let is_new = n != last_outside; //can only be trusted if still_searching
                    let borders_hull = edges.neighbors_of(n).any(|nn| self.inside[nn].yes());
                    if still_searching && is_new && borders_hull {
                        last_outside = curr_outside;
                        curr_outside = n;
                        change_outside = true;
                    }
                }
            }
            if change_boundary {
                steps_since_boundary_change = 0;
            }
            else {
                steps_since_boundary_change += 1;
            }
            if steps_since_boundary_change > 100 || boundary.len() > edges.nr_vertices() || !change_outside {
                //if !change_outside, we have hit a boundary of the graph itself and thus cant find any more 
                //  vertices outside the hull.
                //the resulting boundary part however is of such shape, that we can keep it for the later uses.
                //should that change: add !change_outside to the failure path.

                //for the other two: i have no idea.
                //some error occured -> give up and keep boundary empty
                //the algorithms should work on a planar triangulation, so the given graph is not planar?
                boundary.clear();
                break;
            }
            if curr_outside == fst_outside {
                break; //happy path :)
            }
        }
        self.boundary = boundary;
    }

    fn update_segments(&mut self, min_cop_dist: &[isize]) {
        self.segment_interiors.clear();
        debug_assert_eq!(self.inside.len(), min_cop_dist.len());
        if self.boundary.len() < 5 {
            return;
        }
        debug_assert_eq!(0, min_cop_dist[self.boundary[0]]);
        let mut start = 1;
        loop {
            while min_cop_dist[self.boundary[start]] <= 1 {
                start += 1;
                if start == self.boundary.len() {
                    return;
                }
            }
            let mut stop = start;
            while min_cop_dist[self.boundary[stop]] > 1 {
                stop += 1;
                if stop == self.boundary.len() {
                    return;
                }
            }
            self.segment_interiors.push(start..stop);
            start = stop;
        }
    }

    pub fn update(&mut self, cops: &[Character], edges: &EdgeList, min_cop_dist: &[isize], queue: &mut VecDeque<usize>, 
        vertices_outside_hull: &[usize]) 
    {
        self.update_inside(cops, edges, queue, vertices_outside_hull);
        self.update_boundary(cops, edges);
        self.update_segments(min_cop_dist);
    }
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
    last_write_by: Vec<usize>, 

    /// thing we are actually interested in. 
    /// has one entry per vertex. interesting are only vertices in convex hull.
    /// if vertex `v` is escapable via segment `i`, the `(i % 16)`'th bit is set in `escapable[v]`
    escapable: Vec<u32>,
}

impl EscapeableNodes {
    pub fn escapable(&self) -> &[u32] {
        &self.escapable
    }

    #[allow(dead_code)]
    pub fn boundary_dist(&self) -> &[isize] {
        &self.some_boundary_dist
    }

    pub fn new() -> Self {
        Self { 
            some_boundary_dist: Vec::new(), 
            last_write_by: Vec::new(),
            escapable: Vec::new(),
        }
    }

    /// stores dist to closest boundary vertex of same marker region in `self.some_boundary_dist`.
    fn calc_boundary_dist(&mut self, hull: &ConvexHull, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        self.some_boundary_dist.clear();
        self.some_boundary_dist.resize(edges.nr_vertices(), isize::MAX);
        let boundary_dist = &mut self.some_boundary_dist[..];

        let mut local_queue = std::mem::take(queue);
        local_queue.clear();
        for segment in hull.boundary_segments() {
            for &v in segment {
                boundary_dist[v] = 0;
                local_queue.push_back(v);
            }
            let marker = self.escapable[segment[0]];
            while let Some(v) = local_queue.pop_front() {
                let dist = boundary_dist[v];
                debug_assert!(dist != isize::MAX);
                debug_assert!(self.escapable[v] & marker != 0);
                for n in edges.neighbors_of(v) {
                    if self.escapable[n] & marker == 0 {
                        continue;
                    }
                    if boundary_dist[n] > dist + 1 {
                        boundary_dist[n] = dist + 1;
                        local_queue.push_back(n);
                    }
                }
            }
        }        
        *queue = local_queue;
    }

    /// weird hybrid of region coloring and distance calculation:
    /// update distances of self.some_boundary_dist in neighborhood of queue entries,
    /// recolor region in self.last_write_by to be owner
    fn calc_small_dists(&mut self, hull: &ConvexHull, edges: &EdgeList, queue: &mut VecDeque<usize>, 
        max_dist: isize, segment: Range<usize>, owner: usize) 
    {
        let fst_in_segment = owner == segment.start;
        let mut local_queue = std::mem::take(queue);
        while let Some(v) = local_queue.pop_front() {
            let dist = self.some_boundary_dist[v];
            if dist >= max_dist {
                continue;
            }
            for n in edges.neighbors_of(v) {
                if !hull.inside[n].yes() {
                    continue;
                }
                //the fist boundary vertex in a segment is allowed to mark all vertices in its reach.
                //all further vertices of that same segment can only mark hull vertices which have been reached by
                //that first one.
                //the vertices counted at the end are only those reached by all boundary vertices.
                if !fst_in_segment && !segment.contains(&self.last_write_by[n]) {
                    continue;
                }
                //adapted from Edgelist::calc_distances_to
                if self.last_write_by[n] != owner {
                    self.some_boundary_dist[n] = dist + 1;
                    local_queue.push_back(n);
                    self.last_write_by[n] = owner;
                }
            }
        }
        *queue = local_queue;
    }

    /// assumes segment has form `some_start..=owner`
    /// also assumes queue to only contain vertices of the region to add
    fn add_region_to_escapable(&mut self, owner: usize, marker: u32, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        debug_assert!(owner != 0);
        debug_assert!(queue.iter().all(|&v| self.last_write_by[v] == owner));
        while let Some(v) = queue.pop_front() {
            debug_assert!(self.escapable[v] | marker != 0);
            if self.last_write_by[v] == 0 {
                continue;
            }
            debug_assert_eq!(self.last_write_by[v], owner);
            self.last_write_by[v] = 0;
            for n in edges.neighbors_of(v) {
                if self.last_write_by[n] == owner {
                    self.escapable[n] |= marker;
                    queue.push_back(n);
                }
            }
        }
    }

    /// ignores cops in interior of hull, computes safe regions only with respect to boundary cops
    fn consider_boundary_cops(&mut self, hull: &ConvexHull, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        for (indices, vertices, seq_nr) in izip!(&hull.segment_interiors, hull.boundary_segments(), 0..) {
            let max_dist = indices.len() as isize - 1;
            debug_assert_eq!(indices.len(), vertices.len());
            for (i, &v) in izip!(indices.clone(), vertices) {
                debug_assert!(i != 0);
                queue.push_back(v);
                self.some_boundary_dist[v] = 0;
                self.last_write_by[v] = i;
                self.calc_small_dists(hull, edges, queue, max_dist, indices.clone(), i);
            }
            let owner = indices.end - 1; //last i of loop above
            debug_assert!(queue.is_empty());
            let last = *vertices.last().unwrap();
            queue.push_back(last);
            let marker = 1u32 << (seq_nr % 16);
            self.escapable[last] |= marker;
            self.add_region_to_escapable(owner, marker, edges, queue);
        }
    }

    fn unmark_interior_near(&mut self, cop: &Character, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        let mut local_queue = std::mem::take(queue);
        let start = cop.nearest_node;
        let mut marker = self.escapable[start];
        if marker != 0 {
            self.escapable[start] = 0;
        }
        for n in edges.neighbors_of(start) {
            if self.escapable[n] != 0 {
                marker |= self.escapable[n];
                self.escapable[n] = 0;
                local_queue.push_back(n);
            }
        }
        let cop_boundary_dist = self.some_boundary_dist[start] - 1;
        let signed_marker = marker as isize;
        while let Some(v) = local_queue.pop_front() {
            debug_assert_eq!(self.escapable[v], 0);
            for n in edges.neighbors_of(v) {
                let in_cops_region = self.escapable[n] as isize - signed_marker <= 0;
                let marked = self.escapable[n] & marker != 0;
                //this is only correct, if the current region looks like a triangle, e.g. if the robber dosn't have
                //to decrease his distance to the interior cop while escaping. 
                let cop_can_intersect = self.some_boundary_dist[n] - cop.distances[n] >= cop_boundary_dist;
                if in_cops_region && marked && cop_can_intersect {
                    self.escapable[n] = 0;
                    local_queue.push_back(n);
                }
            }
        }
        *queue = local_queue;
    }

    /// assumes update_boundary_cops has already run
    #[allow(dead_code)]
    fn consider_interior_cops(&mut self, cops: &[Character], hull: &ConvexHull, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        self.calc_boundary_dist(hull, edges, queue);
        let interior_cops = cops.iter().filter(
            |c| c.on_node && edges.neighbors_of(c.nearest_node).all(
                |n| hull.inside[n].yes()
            )
        );
        for cop in interior_cops {
            self.unmark_interior_near(cop, edges, queue);
        }
    }

    pub fn update(&mut self, _cops: &[Character], hull: &ConvexHull, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        let nr_vertices = edges.nr_vertices();
        self.last_write_by.clear();
        self.escapable.clear();
        self.some_boundary_dist.clear();
        self.some_boundary_dist.resize(nr_vertices, isize::MAX);
        self.last_write_by.resize(nr_vertices, 0); //no owner is 0 (as a cop stands at pos 0 of hull boundary)
        self.escapable.resize(nr_vertices, 0);

        self.consider_boundary_cops(hull, edges, queue);
        //self.consider_interior_cops(cops, hull, edges, queue);
    }
}

