
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
                let index_2_3 = vertex_neighbors.edge_direction_index(v2, v3_candidate);
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
            let index_2_3 = vertex_neighbors.edge_direction_index(v2, v3);
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
                    let edge_i = vertex_neighbors.edge_direction_index(last_v, v);
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

    fn for_each_vertex_as_vec3(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for v in &mut self.vertex_positions {
            *v = f(v.to_vec3()).to_pos3();
        }
    }

    fn rescale_vectices(mut self, scale: f32) -> Self {
        self.for_each_vertex_as_vec3(|v| v * scale);
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
        let corrected_scale = scale / vs[0].to_vec3().length();
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(corrected_scale)
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
        let corrected_scale = scale / vs[0].to_vec3().length();
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(corrected_scale)
    }

    pub fn new_cube(scale: f32) -> Self {
        let p = 1.0;
        let n = -1.0;
        let vs = vec![
            pos3(p, p, p),
            pos3(p, p, n),
            pos3(p, n, p),
            pos3(p, n, n),
            pos3(n, p, p),
            pos3(n, p, n),
            pos3(n, n, p),
            pos3(n, n, n),
        ];
        let corrected_scale = scale / vs[0].to_vec3().length();
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(corrected_scale)
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
        let corrected_scale = scale / vs[0].to_vec3().length();
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(corrected_scale)
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
        let corrected_scale = scale / vs[0].to_vec3().length();
        Self::new_platonic_solid_from_positions(vs)
            .rescale_vectices(corrected_scale)
    }

    pub fn new_subdivided_icosahedron(scale: f32, divisions: usize) -> Self {
        let iso = Self::new_icosahedron(1.0);
        Self::subdivide_platonic_with_triangles(iso, divisions)
            .rescale_vectices(scale)
    }

    pub fn new_subdivided_octahedron(scale: f32, divisions: usize) -> Self {
        let oct = Self::new_octahedron(1.0);
        Self::subdivide_platonic_with_triangles(oct, divisions)
            .rescale_vectices(scale)
    }

    pub fn new_subdivided_tetrahedron(scale: f32, divisions: usize) -> Self {
        let tet = Self::new_tetrahedron(1.0);
        Self::subdivide_platonic_with_triangles(tet, divisions)
            .rescale_vectices(scale)
    }

    /// each triangle of the original icosahedron is subdivided in smaller triangles, where divisions
    /// denotes the number of vertices added per original boundary
    fn subdivide_platonic_with_triangles(plat: Self, divisions: usize) -> Self {
        assert_eq!(plat.face_boundary_vertices.min_degree(), 3);
        assert_eq!(plat.face_boundary_vertices.max_degree(), 3);
        if divisions == 0 {
            return plat;
        }
        let mut vertex_positions = plat.vertex_positions.clone();
        let mut vertex_neighbors = EdgeList::new(6, vertex_positions.len());

        let scaled_dir = |p1, p2| (p2 - p1) / ((divisions + 1) as f32);

        //the indices of vertices lying on edges of the original icosahedron, grouped by on which edge they lie.
        //edges are indexed by their edge index, however only the direction (v1, v2) where v1 < v2 is used.
        let mut edge_vertices = vec![Vec::new(); plat.vertex_neighbors.used_space()];
        for boundary in plat.face_boundary_vertices.neighbors() {
            for (v1, v2) in boundary.circular_tuple_windows() {
                if v1 > v2 {
                    continue;
                }
                let p1 = vertex_positions[v1];
                let p2 = vertex_positions[v2];
                let dir = scaled_dir(p1, p2);
                let mut edge = vec![v1];
                for steps in 1..=divisions {
                    let p = p1 + (steps as f32) * dir;
                    let v = vertex_neighbors.push();
                    debug_assert_eq!(v, vertex_positions.len());
                    vertex_positions.push(p);
                    edge.push(v);
                }
                edge.push(v2);
                vertex_neighbors.add_path_edges(edge.iter());
                let edge_index = plat.vertex_neighbors.edge_direction_index(v1, v2);
                edge_vertices[edge_index] = edge;
            }
        }

        let mut boundary_vec = Vec::new(); //temporary to collect the boundary vertices in
        for boundary in plat.face_boundary_vertices.neighbors() {
            boundary_vec.clear();
            boundary_vec.extend(boundary);
            boundary_vec.sort();
            if let [v1, v2, v3] = boundary_vec[..] {
                let edge_1_2_index = plat.vertex_neighbors.edge_direction_index(v1, v2);
                let edge_1_3_index = plat.vertex_neighbors.edge_direction_index(v1, v3);
                let edge_2_3_index = plat.vertex_neighbors.edge_direction_index(v2, v3);
                let edge_1_2 = &edge_vertices[edge_1_2_index]; 
                let edge_1_3 = &edge_vertices[edge_1_3_index];
                let edge_2_3 = &edge_vertices[edge_2_3_index];
                debug_assert_eq!(edge_1_2.first(), Some(&v1));
                debug_assert_eq!(edge_1_2.last(), Some(&v2));
                debug_assert_eq!(edge_1_3.first(), Some(&v1));
                debug_assert_eq!(edge_1_3.last(), Some(&v3));
                debug_assert_eq!(edge_2_3.first(), Some(&v2));
                debug_assert_eq!(edge_2_3.last(), Some(&v3));

                let mut last_levels_nodes = vec![v1];
                let p1 = vertex_positions[v1];
                let p2 = vertex_positions[v2];
                let p3 = vertex_positions[v3];
                let dir_1_2 = scaled_dir(p1, p2);
                let dir_2_3 = scaled_dir(p2, p3);
                //taken more or less directly from GraphDrawing::triangulated_regular_polygon
                for level in 1..=divisions {
                    let level_start_pos = p1 + (level as f32) * dir_1_2;
                    let mut this_levels_nodes =vec![edge_1_2[level]];
                    let nr_inner_nodes = level - 1;
                    for node in 1..=nr_inner_nodes {
                        let node_pos = level_start_pos + (node as f32) * dir_2_3;
                        let node_index = vertex_neighbors.push();
                        debug_assert_eq!(node_index, vertex_positions.len());
                        vertex_positions.push(node_pos);
                        this_levels_nodes.push(node_index);
                    }
                    this_levels_nodes.push(edge_1_3[level]);
                    //connect level to itself
                    vertex_neighbors.add_path_edges(this_levels_nodes.iter());
                    //connect level to previous level
                    //note: this differs from GraphDrawing::triangulated_regular_polygon, because we 
                    //  dont know how often we see an ico-edge as one of edge_1_2 and edge_2_3. 
                    //to not add existing connections, we connected the divided ico edges already when 
                    //  they where constructed.
                    //level 1: none      2: /\     3: /\/\     4: /\/\/\       ...
                    let window = &this_levels_nodes[1..(this_levels_nodes.len() - 1)];
                    vertex_neighbors.add_path_edges(last_levels_nodes.iter()
                        .interleave(window.iter()));

                    last_levels_nodes = this_levels_nodes;
                }
                //connect last inner level to upper edge, e.g. edge_2_3
                let window = &edge_2_3[1..(edge_2_3.len() - 1)];
                vertex_neighbors.add_path_edges(last_levels_nodes.iter()
                        .interleave(window.iter()));
            }
            else {
                panic!("an icosahedron only consists of treeangles");
            }
        }

        //bring all vertives to sphere surface
        for pos in &mut vertex_positions {
            *pos = pos.to_vec3().normalized().to_pos3();
        }

        let (face_boundary_vertices, face_normals) = 
            Self::discover_flat_faces(
                &vertex_positions, &vertex_neighbors);

        Self { 
            vertex_positions, 
            vertex_neighbors, 
            face_boundary_vertices, 
            face_normals 
        }
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

    /// draws edges on camera-facing half with strokes[0], others with strokes[1]
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






















