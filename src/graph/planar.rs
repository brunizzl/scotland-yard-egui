use std::collections::VecDeque;

use egui::{Pos2, Vec2, pos2};
use itertools::Itertools;

use super::*;
use crate::geo::{Pos3, Vec3, vec3};

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
    pub fn into_parts(self) -> (Vec<Pos2>, EdgeList) {
        (self.positions, self.edges)
    }

    pub fn nr_vertices(&self) -> usize {
        debug_assert_eq!(self.positions.len(), self.edges.nr_vertices());
        self.positions.len()
    }

    pub fn empty() -> Self {
        Embedding2D {
            positions: Vec::new(),
            edges: EdgeList::empty(),
        }
    }

    #[inline(always)]
    fn add_vertex(&mut self, pos: Pos2) -> usize {
        let new_index = self.nr_vertices();
        self.positions.push(pos);
        self.edges.push();
        new_index
    }

    #[inline(always)]
    fn has_edge(&self, v1: usize, v2: usize) -> bool {
        self.edges.has_edge(v1, v2)
    }

    #[inline(always)]
    fn add_edge(&mut self, v1: usize, v2: usize) {
        self.edges.add_edge(v1, v2)
    }

    #[inline(always)]
    fn remove_edge(&mut self, v1: usize, v2: usize) {
        self.edges.remove_edge(v1, v2)
    }

    #[inline(always)]
    pub fn positions(&self) -> &[Pos2] {
        &self.positions
    }

    fn sort_neigbors(&mut self) {
        for (v1, neighs) in self.edges.neighbors_mut().enumerate() {
            let p1 = self.positions[v1];

            let angle = |&i2: &Index| {
                let v2 = i2.get().unwrap();
                let p2 = self.positions[v2];
                (p2 - p1).angle()
            };

            let order_floats = |f1: f32, f2: f32| {
                if f1 < f2 {
                    std::cmp::Ordering::Less
                } else if f1 > f2 {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            };

            neighs.sort_by(|n1, n2| order_floats(angle(n1), angle(n2)));
        }
    }

    pub fn neigbors_with_positions(&self, node: usize) -> impl Iterator<Item = (usize, &Pos2)> {
        self.edges.neighbors_of(node).map(move |i| (i, &self.positions[i]))
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

    /// assumes point to be in convex face.
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
                    && geo::left_of_line(neigh_line, point)
                {
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
}

struct Triangualtion {
    graph: Embedding2D,
    circumference: usize, //nr of edges on the outher rim
}

impl Triangualtion {
    fn new_wheel(circumference: usize) -> Self {
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

    #[inline(always)]
    fn has_face(&self, v1: usize, v2: usize, v3: usize) -> bool {
        self.graph.has_edge(v1, v2) && self.graph.has_edge(v2, v3) && self.graph.has_edge(v3, v1)
    }

    //assumes edge of (v1, v2) and vertex v3 form a face,
    //returns other face adjacent to edge
    #[inline(always)]
    fn neighbor_face_vertex(&self, (v1, v2): (usize, usize), v3: usize) -> usize {
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
    #[inline(always)]
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
    fn move_vertices_apart(&mut self, heat: f32) {
        let mut new_positions = self.graph.positions.clone();
        let avg_len = self.avg_edge_len();
        let one_over_avg_edge_len_sq = 1.0 / (avg_len * avg_len);
        //dont wanna move the outher points -> skip circumference
        'step: for (v, neighs) in self.graph.edges.neighbors().enumerate().skip(self.circumference)
        {
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
                    new_positions[v] = Pos2::ZERO + cum_pos / cum_dist;
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
            new_positions[v] = Pos2::ZERO + cum_pos / cum_dist + push_force;
        }

        self.graph.positions = new_positions;
    }

    #[inline(always)]
    fn avg_edge_len(&self) -> f32 {
        std::f32::consts::TAU / (self.circumference as f32)
    }

    #[inline(always)]
    fn divide_longest_edges(&mut self) {
        let avg_len = self.avg_edge_len();
        let mut queue = VecDeque::new();
        let mut v1_neighbors = Vec::new();
        for v1 in 5..self.graph.nr_vertices() {
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

    #[inline(always)]
    fn find_face_of(&self, point: Pos2) -> Option<[usize; 3]> {
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
                    && geo::left_of_line(neigh_line, point)
                {
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

    #[inline(always)]
    fn update_edges(&mut self, queue: &mut VecDeque<((usize, usize), usize)>) {
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
            if geo::left_of_line(new_edge, v1_pos) == geo::left_of_line(new_edge, v2_pos) {
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

    fn add_vertex(
        &mut self,
        new_pos: Pos2,
        queue: &mut VecDeque<((usize, usize), usize)>,
    ) -> usize {
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

    fn new_from_positions(positions: &[Pos2], circumference: usize) -> Triangualtion {
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

    fn new_random(nr_nodes: usize, circumference: usize, seed: u32) -> Self {
        //start with square with unit circle in middle
        let mut res = Self::new_wheel(circumference);

        let mut rng = crate::rand::Lcg::new(seed as u64);
        rng.waste(10);
        let mut random_point = || loop {
            let x = rng.next_in_unit_range();
            let y = rng.next_in_unit_range();
            if x * x + y * y < 1.0 {
                return pos2(x, y);
            }
        };
        let mut queue = VecDeque::new();
        let final_node_index = nr_nodes + res.graph.nr_vertices();
        for _ in 0..(nr_nodes * 2) {
            let pos = random_point();
            let new = res.add_vertex(pos, &mut queue);
            if new == final_node_index {
                break;
            }
        }
        res
    }
} //impl Triangulation

pub fn random_triangulated(radius: usize, nr_refine_steps: usize, seed: u32) -> Embedding2D {
    let r = radius as f32;
    let circumference = (std::f32::consts::TAU * r) as usize;
    let nr_nodes = (std::f32::consts::PI * r * r) as usize;
    let mut tri = Triangualtion::new_random(nr_nodes, circumference, seed);

    let heat_step = 1.0 / (nr_refine_steps + 1) as f32;
    for step in 0..nr_refine_steps {
        tri.divide_longest_edges();
        tri.graph.sort_neigbors();
        let heat = 1.0 - step as f32 * heat_step;
        debug_assert!(heat >= 0.0);
        tri.move_vertices_apart(heat);
        tri.graph.sort_neigbors();
        let without_wheel = &tri.graph.positions[(circumference + 1)..];
        tri = Triangualtion::new_from_positions(without_wheel, circumference);
    }
    tri.graph.edges.maybe_shrink_capacity(0);
    tri.graph.sort_neigbors();
    tri.graph
}

#[allow(dead_code)]
/// takes vertices and edges of a [`super::Embedding3D`] and applies a stereographic projection
/// (see https://en.wikipedia.org/wiki/Stereographic_projection) with the `north_pole` mapped to infinity.
fn project_stereographic_from_3d(
    positions_3d: &[Pos3],
    edges: EdgeList,
    north_pole: Vec3,
) -> Embedding2D {
    let (unrotated_phi, unrotated_tau) = vec3(0.0, 0.0, 1.0).angle();
    let (phi, tau) = north_pole.angle();
    let delta_phi = unrotated_phi - phi;
    let delta_tau = unrotated_tau - tau;

    let positions = positions_3d
        .iter()
        .map(|p| {
            let p = p
                .to_vec3()
                .rotate_z(delta_phi)
                .rotate_x(delta_tau)
                .rotate_z(std::f32::consts::TAU / 4.0)
                .normalized();
            let raw = Vec2 {
                x: p.x / (1.0 - p.z),
                y: p.y / (1.0 - p.z),
            };
            let raw_len = raw.length();
            let corrected = f32::powf(raw_len, 0.5); //TODO: choose monotone function f with f'(0) = 1
            (2.0 * raw * (corrected / raw_len)).to_pos2()
        })
        .collect_vec();

    Embedding2D { positions, edges }
}
