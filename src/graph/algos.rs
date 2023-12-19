
use std::collections::VecDeque;

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
pub struct ConvexHull {
    inside: Vec<InSet>,
    boundary: Vec<usize>,
}

impl ConvexHull {
    pub fn new() -> Self {
        Self {
            inside: Vec::new(),
            boundary: Vec::new(),
        }
    }

    pub fn inside(&self) -> &[InSet] {
        &self.inside
    }

    pub fn boundary(&self) -> &[usize] {
        &self.boundary
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
        loop {
            let mut change = false;
            for n in edges.neighbors_of(curr_outside) {
                //find vertices on hull boundary
                if self.inside[n].yes() {
                    let len = boundary.len();
                    let range_min = if len < 5 { 0 } else { len - 5 };
                    if !boundary[range_min..].contains(&n) {
                        boundary.push(n);
                    }
                }
                else { //find next vertex just outside hull
                    let still_searching = !change;
                    let is_new = n != last_outside; //can only be trusted if still_searching
                    let borders_hull = edges.neighbors_of(n).any(|nn| self.inside[nn].yes());
                    if still_searching && is_new && borders_hull {
                        last_outside = curr_outside;
                        curr_outside = n;
                        change = true;
                    }
                }
            }
            if curr_outside == fst_outside || !change {
                break;
            }
        }
        self.boundary = boundary;
    }

    pub fn update(&mut self, cops: &[Character], edges: &EdgeList, queue: &mut VecDeque<usize>, 
        vertices_outside_hull: &[usize]) 
    {
        self.update_inside(cops, edges, queue, vertices_outside_hull);
        self.update_boundary(cops, edges);
    }
}



