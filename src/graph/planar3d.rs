
use egui::*;
use itertools::Itertools;

use crate::geo::{Pos3, Vec3, pos3, self};

use crate::graph::*;


pub struct ConvexPolyhedron {
    vertex_positions: Vec<Pos3>,

    /// outher unit normal of each face
    /// this entry decides if a face is rendered or not: only camera-facing faces are.
    face_normals: Vec<Vec3>, 

    /// indices of neighboring vertices 
    vertex_neighbors: EdgeList, 

    /// indices of vertices surrounding face
    face_boundary_vertices: EdgeList, 
}

macro_rules! find_related {
    ($xs:expr, $ys:expr, $f:expr) => {
        $xs
        .iter()
        .map(|&x| $ys
            .iter()
            .enumerate()
            .filter_map(move |(i, &y)| if $f(x, y) { Some(i) } else { None }))
    };
}

/// expect boundary to form a circle, order vertices in circle s.t. neighboring vertices follow each other
fn order_face_boundary(boundary: &mut [Index], neighbors: &EdgeList) {
    debug_assert!(boundary.len() > 2);
    let last = boundary.len() - 1;
    'outher: for i1 in 0..last {
        let v1 = boundary[i1];
        let i2 = i1 + 1;
        for search_i in i2..=last {
            let v = boundary[search_i];
            if neighbors.has_edge_(v1, v) {
                boundary.swap(i2, search_i);
                continue 'outher;
            }
        }
        panic!("vertex boundary is not closed");
    }
    debug_assert!(neighbors.has_edge_(boundary[0], boundary[last]));
}

fn is_small(x: f32) -> bool {
    x.abs() < 1e-4
}

impl ConvexPolyhedron {
    fn discover_flat_faces(vertex_positions: &[Pos3], vertex_neighbors: &EdgeList) 
        -> (EdgeList, Vec<Vec3>) 
    {
        let mut unused_edges = vertex_neighbors.all_valid_edge_indices();

        let mut face_boundary_vertices = EdgeList::new(6, 0);
        let mut normals = Vec::new();
        //idea: find a cycle of unused edge directions where at the first intersection 
        //  we turn as hard as possible and on all further intersections 
        //  we choose the vertex on the same plane as the first three.
        //also: we need to discover all faces in the same rotation direction, else we cant use the
        //  edge directions approach.
        'discover_faces: while let Some(edge_index) = unused_edges.iter().position(|&e| e) {
            unused_edges[edge_index] = false;
            let (v1, v2) = vertex_neighbors.edge_at_index(edge_index);
            let new_face = face_boundary_vertices.push();
            face_boundary_vertices.add_directed_edge(new_face, v1);
            face_boundary_vertices.add_directed_edge(new_face, v2);
            let v1_pos = vertex_positions[v1];
            let v2_pos = vertex_positions[v2];
            let dir_1_2 = v2_pos - v1_pos;

            //see coment above 'discover_faces loop: we choose the rotation by taking v3
            //as having positive component in direction_check_vec's direction
            let direction_check_vec = v1_pos.to_vec3().cross(v2_pos.to_vec3());

            let mut v3 = usize::MAX;
            let mut angle_1_2_3 = f32::MAX;
            for v3_candidate in vertex_neighbors.neighbors_of(v2) {
                if v3_candidate == v1 {
                    continue;
                }
                let index_2_3 = vertex_neighbors.edge_direction_index(v2, v3_candidate).unwrap();
                if !unused_edges[index_2_3] {
                    continue;
                }
                let candidate_pos = vertex_positions[v3_candidate];
                if direction_check_vec.dot(candidate_pos.to_vec3()) < 0.0 {
                    continue;
                }
                let dir_3_2 = v2_pos - candidate_pos;
                let new_angle = Vec3::angle_between(dir_1_2, dir_3_2);
                if new_angle < angle_1_2_3 {
                    v3 = v3_candidate;
                    angle_1_2_3 = new_angle;
                }
            }
            assert_ne!(v3, usize::MAX);
            let index_2_3 = vertex_neighbors.edge_direction_index(v2, v3).unwrap();
            unused_edges[index_2_3] = false;
            face_boundary_vertices.add_directed_edge(new_face, v3);
            let v3_pos = vertex_positions[v3];
            let dir_2_3 = v3_pos - v2_pos;
            let face_normal = Vec3::cross(dir_1_2, dir_2_3).normalized();
            normals.push(face_normal);

            let mut snd_last_v = v2;
            let mut last_v = v3;
            let mut last_pos = v3_pos;
            'next_vertex: loop {
                for v in vertex_neighbors.neighbors_of(last_v) {
                    if v == snd_last_v {
                        continue;
                    }
                    let edge_i = vertex_neighbors.edge_direction_index(last_v, v).unwrap();
                    if v == v1 {
                        assert!(unused_edges[edge_i]);
                        unused_edges[edge_i] = false;
                        continue 'discover_faces;
                    }
                    if !unused_edges[edge_i] {
                        continue;
                    }
                    let pos = vertex_positions[v];
                    let dir = pos - last_pos;
                    if is_small(face_normal.dot(dir)) {
                        face_boundary_vertices.add_directed_edge(new_face, v);
                        unused_edges[edge_i] = false;
                        snd_last_v = last_v;
                        last_v = v;
                        last_pos = pos;
                        continue 'next_vertex;
                    }
                }
                panic!("found dead end!");
            }
        }
        face_boundary_vertices.maybe_shrink_capacity(0);
        (face_boundary_vertices, normals)
    }

    fn compute_platonic_face_normals(vertex_positions: &[Pos3], face_boundary_vertices: &EdgeList) -> Vec<Vec3> {
        face_boundary_vertices.neighbors().map(|boundary| {
            boundary
                .map(|v| vertex_positions[v].to_vec3())
                .fold(Vec3::ZERO, std::ops::Add::add)
                .normalized()
        }).collect_vec()
    }

    fn for_each_vertex_position(&mut self, mut f: impl FnMut(Pos3) -> Pos3) {
        for v in &mut self.vertex_positions {
            *v = f(*v);
        }
    }

    fn rescale_vectices(mut self, scale: f32) -> Self {
        self.for_each_vertex_position(|v| (v.to_vec3() * scale).to_pos3());
        self
    }


    /// connects the closest vertices to have edges,
    /// posissions are assumed to lie centered around the origin
    fn new_platonic_solid_from_positions(vertex_positions: Vec<Pos3>) -> Self 
    {
        debug_assert!(vertex_positions.len() >= 4); //this is supposed to form a platonic solid after all

        let neighbor_vertex_dist = {
            let v1 = vertex_positions[0];
            vertex_positions[1..].iter().fold(f32::MAX, 
                |acc, &v| f32::min((v - v1).length(), acc))
        };

        let mut vertex_neighbors = EdgeList::from_iter(
            find_related!(vertex_positions, vertex_positions, 
                |p1: Pos3, p2: Pos3| is_small((p1 - p2).length() - neighbor_vertex_dist)), 6);
        vertex_neighbors.maybe_shrink_capacity(0);

        let (face_boundary_vertices, face_normals) = 
            Self::discover_flat_faces(
                &vertex_positions, &vertex_neighbors);

        debug_assert!({
            let also_normals = Self::compute_platonic_face_normals(
                &vertex_positions, &face_boundary_vertices);
            let diff = also_normals
                .iter()
                .zip(face_normals.iter())
                .fold(0.0, |acc, (&n1, &n2)| acc + (n1 - n2).length_sq());
            is_small(diff) && also_normals.len() == face_normals.len()
        });

        Self { 
            vertex_positions, 
            vertex_neighbors, 
            face_boundary_vertices, 
            face_normals 
        }
    }

    pub fn new_tetrahedron(scale: f32) -> Self {
        let a = 1.0;
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let vs = vec![
            pos3(a, 0.0, -s),
            pos3(-a, 0.0, -s),
            pos3(0.0, a, s),
            pos3(0.0, -a, s),
        ];
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(scale)
    }

    pub fn new_octahedron(scale: f32) -> Self {
        let vs = vec![
            pos3(1.0, 0.0, 0.0),
            pos3(-1.0, 0.0, 0.0),
            pos3(0.0, 1.0, 0.0),
            pos3(0.0, -1.0, 0.0),
            pos3(0.0, 0.0, 1.0),
            pos3(0.0, 0.0, -1.0),
        ];
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(scale)
    }

    pub fn new_cube(scale: f32) -> Self {
        let p = 1.0;
        let n = -1.0;
        let vertex_positions = vec![
            pos3(p, p, p),
            pos3(p, p, n),
            pos3(p, n, p),
            pos3(p, n, n),
            pos3(n, p, p),
            pos3(n, p, n),
            pos3(n, n, p),
            pos3(n, n, n),
        ];
        Self::new_platonic_solid_from_positions(vertex_positions)
            .rescale_vectices(scale)
    }

    pub fn new_dodecahedron(scale: f32) -> Self {
        let a = 1.0;
        let p = (1.0 + f32::sqrt(5.0)) / 2.0;
        let d = 2.0 / (1.0 + f32::sqrt(5.0));
        let vs = vec![
            pos3(a, a, a),
            pos3(a, a, -a),
            pos3(a, -a, a),
            pos3(a, -a, -a),
            pos3(-a, a, a),
            pos3(-a, a, -a),
            pos3(-a, -a, a),
            pos3(-a, -a, -a),
            pos3(0.0, p, d),
            pos3(0.0, p, -d),
            pos3(0.0, -p, d),
            pos3(0.0, -p, -d),
            pos3(d, 0.0, p),
            pos3(d, 0.0, -p),
            pos3(-d, 0.0, p),
            pos3(-d, 0.0, -p),
            pos3(p, d, 0.0),
            pos3(p, -d, 0.0),
            pos3(-p, d, 0.0),
            pos3(-p, -d, 0.0),
        ];
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(scale)
    }

    pub fn new_icosahedron(scale: f32) -> Self {        
        let a = 1.0;
        let c = (1.0 + f32::sqrt(5.0)) / 2.0;
        let vs = vec![
            pos3(0.0, a, c),
            pos3(0.0, a, -c),
            pos3(0.0, -a, c),
            pos3(0.0, -a, -c),
            pos3(a, c, 0.0),
            pos3(a, -c, 0.0),
            pos3(-a, c, 0.0),
            pos3(-a, -c, 0.0),
            pos3(c, 0.0, a),
            pos3(c, 0.0, -a),
            pos3(-c, 0.0, a),
            pos3(-c, 0.0, -a),
        ];
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(scale)
    }

    pub fn draw_visible_faces(&self, to_screen: &geo::ToScreen, painter: &Painter, stroke: Stroke) 
    {
        for (&normal, boundary) in self.face_normals.iter().zip(self.face_boundary_vertices.neighbors()) {
            if to_screen.faces_camera(normal) {
                for (v1, v2) in boundary.circular_tuple_windows() {
                    let p1 = self.vertex_positions[v1];
                    let p2 = self.vertex_positions[v2];

                    let edge = [
                        to_screen.apply(p1), 
                        to_screen.apply(p2)];
                    let line = Shape::LineSegment { points: edge, stroke };
                    painter.add(line);
                }
            }
        }
    }

    //draws edges on camera-facing half with strokes[0], others with strokes[1]
    pub fn draw_edges(&self, to_screen: &geo::ToScreen, painter: &Painter, strokes: [Stroke; 2]) {
        for (v1, neighs) in self.vertex_neighbors.neighbors().enumerate() {
            let p1 = self.vertex_positions[v1];
            for v2 in neighs {
                if v2 <= v1 { //skip edges already drawn the other way around
                    continue;
                }
                let p2 = self.vertex_positions[v2];
                let edge = [
                    to_screen.apply(p1), 
                    to_screen.apply(p2)];
                let mid = p1.lerp(p2, 0.5).to_vec3();
                let stroke = strokes[to_screen.faces_camera(mid) as usize];
                let line = Shape::LineSegment { points: edge, stroke };
                painter.add(line);
            }
        } 
    }
}






















