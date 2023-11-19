
use std::collections::VecDeque;

use egui::{Pos2, Vec2, pos2, vec2};
use itertools::Itertools;

pub struct Graph {
    positions: Vec<Pos2>,

    //position i lists all neighbors of node i
    neighbors: Vec<Vec<usize>>,
}

impl Graph {
    pub fn len(&self) -> usize {
        debug_assert_eq!(self.positions.len(), self.neighbors.len());
        self.positions.len()
    }

    pub fn empty() -> Self {
        Graph { positions: Vec::new(), neighbors: Vec::new() }
    }

    pub fn add_vertex(&mut self, pos: Pos2) -> usize {
        let new_index = self.len();
        self.positions.push(pos);
        self.neighbors.push(Vec::new());
        new_index
    }

    pub fn has_edge(&self, v1: usize, v2: usize) -> bool {
        let res = self.neighbors[v1].contains(&v2);
        debug_assert!((self.neighbors[v2].contains(&v1)) == res);
        res
    }

    pub fn add_edge(&mut self, v1: usize, v2: usize) {
        debug_assert!(!self.has_edge(v1, v2));
        self.neighbors[v1].push(v2);
        self.neighbors[v2].push(v1);
    }

    pub fn remove_edge(&mut self, v1: usize, v2: usize) {
        debug_assert!(self.has_edge(v1, v2));
        self.neighbors[v1].retain(|&v| v != v2);
        self.neighbors[v2].retain(|&v| v != v1);
    }

    pub fn add_path_edges<'a>(&mut self, path: impl Iterator<Item = &'a usize>) {
        for (&v1, &v2) in path.tuple_windows() {
            self.add_edge(v1, v2);
        }
    }

    pub fn positions(&self) -> &[Pos2] {
        &self.positions
    }

    pub fn edges(&self) -> impl Iterator<Item = (&Pos2, &Pos2)> {
        self.neighbors
            .iter()
            .enumerate()
            .flat_map(move |(i1, neighbors)| 
                neighbors
                    .iter()
                    .map(move |i2| (&self.positions[i1], &self.positions[*i2])))        
    }

    pub fn neighbors(&self, node: usize) -> &[usize] {
        &self.neighbors[node]
    } 

    pub fn neigbors_with_positions(&self, node: usize) -> impl Iterator<Item = (usize, &Pos2)> {
        self.neighbors[node].iter().map(move |i| (*i, &self.positions[*i]))
    }

    //everything in queue is starting point and expected to already have the correct distance
    pub fn calc_distances_to(&self, queue: &mut VecDeque<usize>, distances: &mut Vec<usize>) {
        while let Some(node) = queue.pop_front() {
            let dist = distances[node];
            for &neigh in &self.neighbors[node] {
                if distances[neigh] > dist + 1 {
                    distances[neigh] = dist + 1;
                    queue.push_back(neigh);
                }
            }
        }
    }

    //paintbucket tool, all in queue are starting vertices
    pub fn recolor_region<Color: Eq + Clone>(&self, (old, new): (Color, Color), 
        colors: &mut [Color], queue: &mut VecDeque<usize>) {

        while let Some(node) = queue.pop_front() {
            for &neigh in self.neighbors(node) {
                if colors[neigh] == old {
                    colors[neigh] = new.clone();
                    queue.push_back(neigh);
                }
            }
        }
    }    

    //returns node index and distance to that index squared
    pub fn find_nearest_node(&self, pos: Pos2, node_hint: usize) -> (usize, f32) {
        let mut nearest = node_hint;
        let mut nearest_dist = (self.positions()[node_hint] - pos).length_sq();
        let mut maybe_neighbor_closer = true;
        while maybe_neighbor_closer {
            maybe_neighbor_closer = false;
            for (neigh, &neigh_pos) in self.neigbors_with_positions(nearest) {
                let neigh_dist = (neigh_pos - pos).length_sq();
                if neigh_dist < nearest_dist {
                    nearest = neigh;
                    nearest_dist = neigh_dist;
                    maybe_neighbor_closer = true;
                }
            }
        }
        (nearest, nearest_dist)
    }

    //assumes point to be in convex face.
    pub fn find_face_of(&self, point: Pos2) -> Vec<usize> {
        //caution: nearest node may not be on face boundary
        let (mut curr, _) = self.find_nearest_node(point, 0);
        let mut curr_pos = self.positions[curr];
        let mut curr_line = geo::line_from_to(point, curr_pos);
        let mut res = vec![curr];
        loop {
            let mut next = usize::MAX;
            let mut next_pos = Pos2::ZERO;
            let mut next_line = curr_line;
            for (neigh, &neigh_pos) in self.neigbors_with_positions(curr) {
                let neigh_line = geo::line_from_to(curr_pos, neigh_pos);
                if geo::left_of_line(curr_line, neigh_pos)
                && geo::left_of_line(next_line, neigh_pos)
                && geo::left_of_line(neigh_line, point) {
                    next = neigh;
                    next_pos = neigh_pos;
                    next_line = neigh_line;
                }
            }
            if next == usize::MAX {
                return Vec::new();
            }
            if let Some(start) = res.iter().position(|&v| v == next) {                
                return res[start..].into();
            }
            res.push(next);
            curr = next;
            curr_pos = next_pos;
            curr_line = next_line;
        }
    }

} //impl Graph

//centered at zero, contained in -1.0..1.0 x -1.0..1.0
pub fn triangulated_regular_polygon(sides: usize, levels: usize) -> Graph {
    let mut graph = Graph::empty();
    graph.add_vertex(Pos2::ZERO);
    if levels < 1 {
        return graph;
    }
    //idea: build sector by sector, first the nodes on the sector borders
    let edge_length = 1.0 / std::cmp::max(1, levels) as f32;
    let mut sector_borders = Vec::new();
    for border_nr in 0..sides {
        let next_free_index = graph.len();
        let border = std::iter::once(0)
            .chain(next_free_index..(next_free_index + levels))
            .collect::<Vec<_>>();
        let angle = std::f32::consts::TAU * ((border_nr as f32 + 0.5) / (sides as f32) + 0.25);
        let unit_dir = vec2(angle.cos(), angle.sin()) * edge_length;
        sector_borders.push((border, unit_dir));
        for dist_to_origin in 1..(levels + 1) {
            graph.add_vertex(Pos2::ZERO + (dist_to_origin as f32) * unit_dir);
        }
    }
    for ((b1, u1), (b2, u2)) in sector_borders.iter().circular_tuple_windows() {
        let unit_diff = *u2 - *u1;
        let mut last_levels_nodes = vec![0]; //lowest level contains only origin
        for level in 1..(levels + 1) {
            let level_start_pos = Pos2::ZERO + (level as f32) * *u1;
            let mut this_levels_nodes = vec![b1[level]];
            let nr_inner_nodes = level - 1;
            for node in 1..(nr_inner_nodes + 1) {
                let node_pos = level_start_pos + (node as f32) * unit_diff;
                let node_index = graph.add_vertex(node_pos);
                this_levels_nodes.push(node_index);
            }
            this_levels_nodes.push(b2[level]);
            //connect level to itself
            graph.add_path_edges(this_levels_nodes.iter());
            //connect level to previous level
            //note: the last added edge connects b2's nodes
            //e.g. for upper sector we see the following paths added (all drawn right to left):
            //level 1: \      2: \/\     3: \/\/\     4: \/\/\/\       ...
            graph.add_path_edges(last_levels_nodes.iter()
                .interleave(this_levels_nodes[1..].iter()));
            
            last_levels_nodes = this_levels_nodes;
        }
    }
    graph
}

//short for geometry
mod geo {
    use super::*;

    type Line = (Pos2, Vec2);

    pub fn line_from_to(from: Pos2, to: Pos2) -> Line {
        (from, to - from)
    }

    pub fn left_of_line((a, dir): Line, p: Pos2) -> bool {
        dir.x * (p.y - a.y) - dir.y * (p.x - a.x) > 0.0
    }

    //assumes dir to be normalized.
    pub fn signed_dist((a, dir): Line, p: Pos2) -> f32 {
        dir.x * (a.y - p.y) - dir.y * (a.x - p.x)
    }
}

pub struct Triangualtion {
    graph: Graph,
    radius: f32,
}

impl Triangualtion {

    pub fn new_base_square(r: f32) -> Self {
        let mut graph = Graph::empty();
        let v0 = graph.add_vertex(pos2(0.0, 0.0));
        let v1 = graph.add_vertex(pos2(r, r));
        let v2 = graph.add_vertex(pos2(r, -r));
        let v3 = graph.add_vertex(pos2(-r, -r));
        let v4 = graph.add_vertex(pos2(-r, r));
        //outer edges
        graph.add_path_edges([v1, v2, v3, v4, v1].iter());
        //to center
        graph.add_edge(v0, v1);
        graph.add_edge(v0, v2);
        graph.add_edge(v0, v3);
        graph.add_edge(v0, v4);
        Self { graph, radius: r }
    }

    pub fn remove_base_square(mut self) -> Graph {
        { //delete vertices of surrounding square
            let mut i = 0;      
            self.graph.neighbors.retain(|_| {
                let keep = i == 0 || i > 4;
                i += 1;
                keep
            });
            i = 0;
            self.graph.positions.retain(|_| {
                let keep = i == 0 || i > 4;
                i += 1;
                keep
            });
        }
        //delete edges to vertives of surrounding square
        for neighs in &mut self.graph.neighbors {
            neighs.retain(|&v| v == 0 || v > 4);
        }
        //adjust edge indices
        for neighs in &mut self.graph.neighbors {
            for neigh in neighs {
                if *neigh != 0 {
                    *neigh -= 4;
                }
            }
        }
        self.graph
    }

    pub fn has_face(&self, v1: usize, v2: usize, v3: usize) -> bool {
        self.graph.has_edge(v1, v2) &&
        self.graph.has_edge(v2, v3) &&
        self.graph.has_edge(v3, v1)
    }

    //assumes edge of (v1, v2) and vertex v3 form a face, 
    //returns other face adjacent to edge
    pub fn neighbor_face_vertex(&self, (v1, v2): (usize, usize), v3: usize) -> usize {
        debug_assert!(self.has_face(v1, v2, v3));

        let v1_neighbors = self.graph.neighbors(v1);
        let v2_neighbors = self.graph.neighbors(v2);
        let v1_pos = self.graph.positions[v1];        
        let v2_pos = self.graph.positions[v2];
        let v3_pos = self.graph.positions[v3];

        let mut best = usize::MAX;
        let mut best_dist = f32::MAX;
        let edge = geo::line_from_to(v1_pos, v2_pos);
        let v3_dist_sign = geo::signed_dist(edge, v3_pos).signum();
        for &neigh in v1_neighbors {
            let n1_pos = self.graph.positions[neigh];
            let (sign, abs) = {
                let dist = geo::signed_dist(edge, n1_pos);
                (dist.signum(), dist.abs())
            };
            if sign != v3_dist_sign
            && abs < best_dist
            && v2_neighbors.contains(&neigh) {
                best = neigh;
                best_dist = abs;
            }
        }
        best
    }

    pub fn find_face_of(&self, point: Pos2) -> Option<[usize; 3]> {        
        let (start, _) = self.graph.find_nearest_node(point, 0);
        let mut curr_pos = self.graph.positions[start];
        let mut curr_line = geo::line_from_to(point, curr_pos);
        
        let mut res = [start, usize::MAX, usize::MAX];
        let mut i = 0; //index in res
        let mut __i = 0;
        loop {
            __i += 1;
            let mut next = usize::MAX;
            let mut next_pos = Pos2::ZERO;
            let mut next_line = curr_line;
            let i_next = if i == 2 { 0 } else { i + 1 };
            for (neigh, &neigh_pos) in self.graph.neigbors_with_positions(res[i]) {  
                let neigh_line = geo::line_from_to(curr_pos, neigh_pos);
                if geo::left_of_line(curr_line, neigh_pos)
                && geo::left_of_line(next_line, neigh_pos)
                && geo::left_of_line(neigh_line, point) {              
                    if res[i_next] == neigh {
                        return Some(res);
                    }
                    next = neigh;
                    next_pos = neigh_pos;
                    next_line = neigh_line;
                }
            }
            if next == usize::MAX || __i > 20 {
                return None;
            }
            res[i_next] = next;
            i = i_next;
            curr_pos = next_pos;
            curr_line = next_line;
        }
    }

    pub fn add_vertex(&mut self, new_pos: Pos2, queue: &mut VecDeque<((usize, usize), usize)>) -> usize {
        debug_assert!(new_pos.x.abs() <= self.radius);
        debug_assert!(new_pos.y.abs() <= self.radius);

        if let Some(face) = self.find_face_of(new_pos) {
            let new = self.graph.add_vertex(new_pos);
            for &v in face.iter() {
                self.graph.add_edge(new, v);
            }
            //we are now again a valid triangulation.
            //following from here is just an attempt to improve the quality,
            //e.g. make the triangles less slim. we do this by proxy: the goal is to find
            //  shortest possible sides for our triangles.
            queue.clear();
            for (&v1, &v2) in face.iter().circular_tuple_windows() {                
                queue.push_back(((v1, v2), new));
                debug_assert!({
                    let v1_pos = self.graph.positions[v1];
                    let v2_pos = self.graph.positions[v2];
                    let fw_edge = geo::line_from_to(v1_pos, v2_pos);
                    let bw_edge = geo::line_from_to(v2_pos, v1_pos);
                    geo::left_of_line(fw_edge, new_pos)
                    && !geo::left_of_line(bw_edge, new_pos)
                });
            }
            while let Some(((v1, v2), v0)) = queue.pop_front() {
                if !self.has_face(v1, v2, v0) {
                    dbg!((v1, v2, v0));
                    continue;
                }
                let v3 = self.neighbor_face_vertex((v1, v2), v0);
                if v3 == usize::MAX {
                    continue;
                }
                debug_assert!(self.has_face(v1, v2, v3));
                let v0_pos = self.graph.positions[v0];
                let v1_pos = self.graph.positions[v1];
                let v2_pos = self.graph.positions[v2];
                let v3_pos = self.graph.positions[v3];

                let new_edge = geo::line_from_to(v0_pos, v3_pos);
                //test if connection from v0 to v3 crosses edge from v1 to v2
                if geo::left_of_line(new_edge, v1_pos) 
                == geo::left_of_line(new_edge, v2_pos) {
                    continue;
                }
                //test if edge from v0 to v3 would be shorter than edge from v1 to v2
                //if so: use new edge instead
                let old_len = (v1_pos - v2_pos).length_sq();
                let new_len = (v0_pos - v3_pos).length_sq();
                if new_len < old_len {
                    self.graph.remove_edge(v1, v2);
                    self.graph.add_edge(v0, v3);
                    queue.push_back(((v2, v3), v0));
                    queue.push_back(((v1, v3), v0));
                    queue.push_back(((v2, v0), v3));
                    queue.push_back(((v1, v0), v3));
                }
            }

            return new;
        }

        usize::MAX     
    }
} //impl Triangulation

pub fn random_triangulated(nr_nodes: usize) -> Graph {
    //start with square with unit circle in middle
    let mut triangulation = Triangualtion::new_base_square(10.0);

    use rand::distributions::{Distribution, Uniform};
    let mut rng = rand::thread_rng();
    let coordinate_generator = Uniform::from(-1.0..1.0);
    let mut random_point = || {
        loop {
            let x = coordinate_generator.sample(&mut rng);
            let y = coordinate_generator.sample(&mut rng);
            if x * x + y * y < 1.0 {
                return pos2(x, y);
            }
        }
    };
    let mut queue = VecDeque::new();
    for _ in 0..nr_nodes {
        let pos = random_point();
        for _ in 0..3 {
            let new = triangulation.add_vertex(pos, &mut queue);
            if new != usize::MAX {
                break;
            }
        }
    }

    triangulation.remove_base_square()
}

pub fn debugging_graph() -> Graph {
    let mut triangulation = Triangualtion::new_base_square(10.0);
    let mut queue = VecDeque::new();
    let positions = [
        pos2(-0.143388033, 0.978398561),
        pos2(0.263813496, 0.514246702),
        pos2(0.74604845, 0.300056696),
        pos2(0.256795406, -0.64158415),
    ];
    for pos in positions {
        let new = triangulation.add_vertex(pos, &mut queue);
        assert!(new != usize::MAX);
    }
    triangulation.remove_base_square()
}
