
use std::collections::VecDeque;
use std::iter::Map;
use std::slice::{Chunks, Iter};

use egui::{Pos2, Vec2, pos2, vec2};
use itertools::Itertools;

use super::*;

pub struct Embedding2D {
    positions: Vec<Pos2>,
    edges: EdgeList,
}

impl Default for Embedding2D {
    fn default() -> Self {
        Self::empty()
    }
}

impl Embedding2D {
    pub fn len(&self) -> usize {
        debug_assert_eq!(self.positions.len(), self.edges.len());
        self.positions.len()
    }

    pub fn empty() -> Self {
        Embedding2D { positions: Vec::new(), edges: EdgeList::empty() }
    }

    pub fn add_vertex(&mut self, pos: Pos2) -> usize {
        let new_index = self.len();
        self.positions.push(pos);
        self.edges.push();
        new_index
    }

    pub fn has_edge(&self, v1: usize, v2: usize) -> bool {
        self.edges.has_edge(v1, v2)
    }

    pub fn add_edge(&mut self, v1: usize, v2: usize) {
        self.edges.add_edge(v1, v2)
    }

    pub fn remove_edge(&mut self, v1: usize, v2: usize) {
        self.edges.remove_edge(v1, v2)
    }

    pub fn add_path_edges<'a>(&mut self, path: impl Iterator<Item = &'a usize>) {
        self.edges.add_path_edges_ref(path)
    }

    pub fn positions(&self) -> &[Pos2] {
        &self.positions
    }

    pub fn neighbors(&self) -> 
        Map<Chunks<'_, Index>, fn(&[Index]) -> Map<Iter<'_, Index>, fn(&Index) -> usize>> {
        self.edges.neighbors()
    }

    pub fn neighbors_of(&self, v: usize) -> Map<Iter<'_, Index>, fn(&Index) -> usize> {
        self.edges.neighbors_of(v)
    }

    pub fn for_each_edge(&self, mut f: impl FnMut(usize, Pos2, usize, Pos2)) {
        self.edges.for_each_edge(|v1, v2| {
            let p1 = self.positions[v1];
            let p2 = self.positions[v2];
            f(v1, p1, v2, p2)
        });
    }

    pub fn sort_neigbors(&mut self) {
        for (v1, neighs) in self.edges.potential_neighbors_mut().enumerate() {
            let p1 = self.positions[v1];

            let angle = |&i2:&Index| if let Some(v2) = i2.get() {
                let p2 = self.positions[v2];
                (p2 - p1).angle()
            }
            else { f32::MAX };

            let order_floats = |f1: f32, f2: f32| 
                if f1 < f2 { std::cmp::Ordering::Less }
                else if f1 > f2 { std::cmp::Ordering::Greater }
                else { std::cmp::Ordering::Equal };

            neighs.sort_by(|n1, n2| order_floats(angle(n1), angle(n2)));
        }
    }

    pub fn neigbors_with_positions(&self, node: usize) -> impl Iterator<Item = (usize, &Pos2)> {
        self.edges.neighbors_of(node).map(move |i| (i, &self.positions[i]))
    }

    /// everything in queue is starting point and expected to already have the correct distance
    pub fn calc_distances_to(&self, queue: &mut VecDeque<usize>, distances: &mut Vec<isize>) {
        self.edges.calc_distances_to(queue, distances)
    }

    /// paintbucket tool, all in queue are starting vertices
    pub fn recolor_region<Color: Eq + Clone>(&self, (old, new): (Color, Color), 
        colors: &mut [Color], queue: &mut VecDeque<usize>) {

        self.edges.recolor_region((old, new), colors, queue)
    }

    /// returns node index and distance to that index squared
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
    #[allow(dead_code)]
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

/// centered at zero, contained in -1.0..1.0 x -1.0..1.0
pub fn triangulated_regular_polygon(sides: usize, levels: usize) -> Embedding2D {
    let mut graph = Embedding2D::empty();
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
        for level in 1..=levels {
            let level_start_pos = Pos2::ZERO + (level as f32) * *u1;
            let mut this_levels_nodes = vec![b1[level]];
            let nr_inner_nodes = level - 1;
            for node in 1..=nr_inner_nodes {
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

pub struct Triangualtion {
    graph: Embedding2D,
    circumference: usize, //nr of edges on the outher rim
}

impl Triangualtion {
    pub fn new_wheel(circumference: usize) -> Self {
        let mut graph = Embedding2D::empty();
        for v in 0..circumference {
            let angle = std::f32::consts::TAU * (v as f32) / (circumference as f32);
            let pos = pos2(angle.cos(), angle.sin());
            let new = graph.add_vertex(pos);
            debug_assert!(v == new);
        }
        for (v1, v2) in (0..circumference).circular_tuple_windows() {
            graph.add_edge(v1, v2);
        }

        graph.add_vertex(Pos2::ZERO);
        for v in 0..circumference {
            graph.add_edge(circumference, v);
        }

        Self { graph, circumference }
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

        let v1_neighs = self.graph.edges.neighbors_of(v1);
        let v2_neighs = self.graph.edges.neighbors_of(v2);
        let v1_pos = self.graph.positions[v1];        
        let v2_pos = self.graph.positions[v2];
        let v3_pos = self.graph.positions[v3];

        let mut best = usize::MAX;
        let mut best_dist = f32::MAX;
        let edge = geo::line_from_to(v1_pos, v2_pos);
        let v3_sign = geo::signed_dist(edge, v3_pos).signum();
        for n1 in v1_neighs {
            let n1_pos = self.graph.positions[n1];
            let (sign, abs) = {
                let dist = geo::signed_dist(edge, n1_pos);
                (dist.signum(), dist.abs())
            };
            if sign != v3_sign && abs < best_dist && v2_neighs.clone().contains(&n1) {
                best = n1;
                best_dist = abs;
            }
        }
        debug_assert!(best == usize::MAX || self.has_face(v1, v2, best));

        best
    }

    //each neighbor pushes v away from its position depending on their distance
    fn neighbor_push_force(&self, v: usize, one_over_avg_edge_len_sq: f32) -> Vec2 {
        let v_pos = self.graph.positions[v];
        let mut force = Vec2::ZERO;
        for neigh in self.graph.edges.neighbors_of(v) {
            let n_pos = self.graph.positions[neigh];
            let to_v = v_pos - n_pos;
            force += to_v / (to_v.length_sq() * one_over_avg_edge_len_sq + 0.1); 
        }
        force * 0.15
    }

    //move every a step into the center of its neighbors positions average
    //requires neighbors to be sorted
    pub fn move_vertices_apart(&self, heat: f32) -> Vec<Pos2> {
        let mut res = self.graph.positions.clone();
        let avg_len = self.avg_edge_len();
        let one_over_avg_edge_len_sq = 1.0 / (avg_len * avg_len);
        //dont wanna move the outher points -> skip circumference
        'step: for (v, neighs) in self.graph.edges.neighbors().enumerate().skip(self.circumference) {
            let mut cum_pos = Vec2::ZERO;
            let mut cum_dist = 0.0;
            for (n1, n2) in neighs.circular_tuple_windows() {
                //sometimes the angle isnt able to order the neighbors accurately, because numerics.
                //in that case we have a fallback to move this bad vertex into the middle of its neighbors.
                if !self.graph.has_edge(n1, n2) {
                    cum_pos = Vec2::ZERO;
                    cum_dist = 0.0;
                    for n in self.graph.edges.neighbors_of(v) {
                        cum_pos += self.graph.positions[n].to_vec2();
                        cum_dist += 1.0;
                    }
                    res[v] = Pos2::ZERO + cum_pos / cum_dist;
                    continue 'step;
                }
                //if all neighbors are sorted correctly, we move the vertex to the average position of 
                //the edges connecting its neighbors.
                let p1 = self.graph.positions[n1].to_vec2();
                let p2 = self.graph.positions[n2].to_vec2();
                let length = (p1 - p2).length();
                cum_dist += 2.0 * length;
                cum_pos += p1 * length;
                cum_pos += p2 * length;
            }
            let push_force = heat * self.neighbor_push_force(v, one_over_avg_edge_len_sq);
            res[v] = Pos2::ZERO + cum_pos / cum_dist + push_force;
        }

        res
    }

    pub fn avg_edge_len(&self) -> f32 {
        std::f32::consts::TAU / (self.circumference as f32)
    }

    pub fn divide_longest_edges(&mut self) {
        let avg_len = self.avg_edge_len();
        let mut queue = VecDeque::new();
        let mut v1_neighbors = Vec::new();
        for v1 in 5..self.graph.len() {
            let v1_pos = self.graph.positions[v1];
            v1_neighbors.clear();
            v1_neighbors.extend(self.graph.edges.neighbors_of(v1));
            for &v2 in &v1_neighbors {
                let v2_pos = self.graph.positions[v2];
                let dir = v2_pos - v1_pos;
                if dir.length() > avg_len * 1.8 {
                    let new_pos = v1_pos + 0.5 * dir;
                    self.add_vertex(new_pos, &mut queue);
                }
            }
        }
    }

    pub fn find_face_of(&self, point: Pos2) -> Option<[usize; 3]> {        
        let (start, _) = self.graph.find_nearest_node(point, self.circumference);
        let mut curr_pos = self.graph.positions[start];
        let mut curr_line = geo::line_from_to(point, curr_pos);
        
        let mut res = [start, usize::MAX, usize::MAX];
        let mut i = 0; //index in res
        let mut nr_steps = 0;
        loop {
            nr_steps += 1;
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
            if next == usize::MAX || nr_steps > 12 {
                return None;
            }
            res[i_next] = next;
            i = i_next;
            curr_pos = next_pos;
            curr_line = next_line;
        }
    }

    pub fn update_edges(&mut self, queue: &mut VecDeque<((usize, usize), usize)>) {
        while let Some(((v1, v2), v0)) = queue.pop_front() {
            if !self.has_face(v1, v2, v0) {
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
    }

    pub fn add_vertex(&mut self, new_pos: Pos2, queue: &mut VecDeque<((usize, usize), usize)>) -> usize {
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
                    let edge = geo::line_from_to(v1_pos, v2_pos);
                    geo::signed_dist(edge, new_pos) <= 0.0
                });
            }
            self.update_edges(queue);

            return new;
        }

        usize::MAX     
    }

    pub fn new_from_positions(positions :&[Pos2], circumference: usize) -> Triangualtion {
        let mut discarded = Vec::new();
        let mut queue = VecDeque::new();
        let mut res = Self::new_wheel(circumference);
        for &pos in positions {
            if res.add_vertex(pos, &mut queue) == usize::MAX {
                discarded.push(pos);
            }
        }
        for pos in discarded {
            res.add_vertex(pos, &mut queue);
        }

        res
    }

    pub fn new_random(nr_nodes: usize, circumference: usize) -> Self {
        //start with square with unit circle in middle
        let mut res = Self::new_wheel(circumference);

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
        let final_node_index = nr_nodes + res.graph.len();
        for _ in 0..(nr_nodes * 2) {
            let pos = random_point();
            let new = res.add_vertex(pos, &mut queue);
            if new == final_node_index {
                break;
            }
        }
        dbg!(res.graph.edges.max_neighbors());
        res
    }
} //impl Triangulation

pub fn random_triangulated(radius: usize, nr_refine_steps: usize) -> Embedding2D {
    let r = radius as f32;
    let circumference = (std::f32::consts::TAU * r) as usize;
    let nr_nodes = (std::f32::consts::PI * r * r) as usize;
    let mut tri = Triangualtion::new_random(nr_nodes, circumference);

    let heat_step = 1.0 / nr_refine_steps as f32;
    for step in 0..nr_refine_steps {
        tri.divide_longest_edges();
        tri.graph.sort_neigbors();
        let heat = 1.0 - step as f32 * heat_step;
        debug_assert!(heat >= 0.0);
        let new_vertices = tri.move_vertices_apart(heat);
        let without_wheel = &new_vertices[(circumference + 1)..];
        tri = Triangualtion::new_from_positions(without_wheel, circumference);
    }
    tri.graph.edges.maybe_shrink_capacity(0);
    tri.graph
}

pub fn debugging_graph() -> Embedding2D {
    Triangualtion::new_wheel(10).graph
}
