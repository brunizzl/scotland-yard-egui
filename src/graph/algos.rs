
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
#[derive(serde::Deserialize, serde::Serialize)]
pub struct ConvexHull {
    /// one entry per vertex in graph
    inside: Vec<InSet>, 
    /// enumerates vertices which are on boundary
    boundary: Vec<usize>, 

    /// nr of steps to reach outside. e.g. 0 for vertices outside hull
    /// and 1 for vertices on boundary
    dist_to_outside: Vec<isize>,
    /// indexes in self.boundary, each segment is delimited by two cops on boundary.
    /// the cops' vertices and the vertices neighboring these cops are excluded
    segment_interiors: Vec<Range<usize>>, 
}

impl ConvexHull {
    pub fn new() -> Self {
        Self {
            inside: Vec::new(),
            boundary: Vec::new(),
            dist_to_outside: Vec::new(),
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

    #[allow(dead_code)]
    pub fn dist_to_outside(&self) -> &[isize] {
        &self.dist_to_outside
    }

    /// each segment consists of vertices on boundary, segments are divided by cops
    pub fn boundary_segments<'a>(&'a self) -> impl ExactSizeIterator<Item = &'a [usize]> + 'a + Clone {
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

    /// assumes self.inside is already up to date
    fn update_dist_to_outside(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        debug_assert_eq!(self.inside.len(), edges.nr_vertices());
        queue.clear();
        self.dist_to_outside.clear();
        let mut iter = izip!(&self.inside, edges.neighbors(), 0..);
        self.dist_to_outside.resize_with(edges.nr_vertices(), || {
            let (&in_hull, mut neighs, v) = iter.next().unwrap();
            if !in_hull.yes() {
                0
            } else if neighs.all(|n| self.inside[n].yes()) { 
                isize::MAX 
            } else { 
                queue.push_back(v);
                1 
            }
        });

        edges.calc_distances_to(queue, &mut self.dist_to_outside);
    }

    fn find_fist_boundary_point(&self, cops: &[Character], edges: &EdgeList) -> Option<usize> {
        for cop in cops.iter().filter(|c| c.on_node) {
            let v = cop.nearest_node;
            if !self.inside[v].yes() { //only happens when there is only one cop
                continue;
            }
            for n in edges.neighbors_of(v) {
                if !self.inside[n].yes() {
                    debug_assert_eq!(self.dist_to_outside[v], 1);
                    return Some(v);
                }
            }
        }
        None
    }

    /// assumes that self.dist_to_outside is already up to date
    fn update_boundary(&mut self, cops: &[Character], edges: &EdgeList, queue: &mut VecDeque<usize>) {
        let Some(fst_inside) = self.find_fist_boundary_point(cops, edges) else { return; };

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
                if self.dist_to_outside[n] != 1 {
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
                            .filter(|&n| self.dist_to_outside[n] == 0)
                            .contains(&v);
                        if edges.neighbors_of(next).any(is_neigh_of_curr_and_outside_hull) {
                            break 'take_one_sharing_side next;
                        }
                    }
                    break 'find_next;
                },
            };
            potential_next.clear();
            if next_inside == fst_inside { //happy exit :)
                self.boundary = boundary;
                return;
            }            
            last_inside = curr_inside;
            curr_inside = next_inside;
            boundary.push(next_inside);
        }
        boundary.clear(); //sad exit :(
        potential_next.clear();
        self.boundary = boundary;
        *queue = potential_next;
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
        self.update_dist_to_outside(edges, queue);
        self.update_boundary(cops, edges, queue);
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
    /// if vertex `v` is escapable via segment `i`, the `(i % 32)`'th bit is set in `escapable[v]`
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

    #[allow(dead_code)]
    pub fn owners(&self) -> &[usize] {
        &self.last_write_by
    }

    pub fn new() -> Self {
        Self { 
            some_boundary_dist: Vec::new(), 
            last_write_by: Vec::new(),
            escapable: Vec::new(),
        }
    }

    
    /// stores dist to closest boundary vertex of region owned by owner in `self.some_boundary_dist`.
    fn calc_boundary_dist_to(&mut self, boundary: &[usize], owner: usize, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        debug_assert_eq!(self.some_boundary_dist.len(), edges.nr_vertices());
        debug_assert!(boundary.iter().all(|&v| self.last_write_by[v] == owner));

        queue.clear();
        let v0 = boundary[0];
        self.some_boundary_dist[v0] = isize::MAX;
        queue.push_back(v0);
        edges.recolor_region_with(
            isize::MAX, 
            &mut self.some_boundary_dist, 
            |v, _| self.last_write_by[v] == owner, 
            queue
        );

        for &v in boundary {
            self.some_boundary_dist[v] = 0;
            queue.push_back(v);
        }
        edges.calc_distances_to_with(
            &mut self.some_boundary_dist, 
            |v, _| self.last_write_by[v] == owner, 
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
                    continue;
                }
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
                let in_hull = |x:usize| hull.inside[x].yes();
                self.calc_small_dists(in_hull, edges, queue, max_dist, last_i, i);
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
            let marker = 1u32 << (seq_nr % 32);
            self.escapable[last_v] |= marker;
            self.add_region_to_escapable(owner, marker, edges, queue);
            self.calc_boundary_dist_to(vertices, owner, edges, queue);
        }
    }

    pub fn update(&mut self, _cops: &[Character], hull: &ConvexHull, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        let nr_vertices = edges.nr_vertices();
        self.escapable.clear();
        self.last_write_by.clear();
        self.some_boundary_dist.clear();
        self.escapable.resize(nr_vertices, 0);
        self.last_write_by.resize(nr_vertices, 0); //no owner is 0 (as a cop stands at pos 0 of hull boundary)
        self.some_boundary_dist.resize(nr_vertices, isize::MAX);

        self.consider_boundary_cops(hull, edges, queue);
    }
}

