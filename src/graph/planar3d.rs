
use std::iter;
use std::collections::VecDeque;

use egui::*;
use itertools::Itertools;

use crate::geo::{Pos3, Vec3, pos3, self};

use crate::graph::*;


/// triangulation of a closed 2d surface embedded in 3d
pub struct ConvexTriangleHull {
    vertices: Vec<Pos3>,

    /// outher unit normal of each face
    /// this entry decides if a face is rendered or not: only camera-facing faces are.
    face_normals: Vec<Vec3>, 

    /// indices of neighboring vertices 
    edges: EdgeList, 

    /// indices of vertices surrounding face
    triangles: Vec<[usize; 3]>, 
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

fn is_small(x: f32) -> bool {
    x.abs() < 1e-4
}

impl ConvexTriangleHull {
    fn discover_faces(vertices: &[Pos3], edges: &EdgeList) -> (Vec<[usize; 3]>, Vec<Vec3>) {
        //enumerates each edge twice: once in each direction
        let mut unused_edges = edges.all_valid_edge_indices();

        let mut triangles = Vec::new();
        let mut normals = Vec::new();

        let mut next_search_start = 0;
        while let Some(edge_index) = unused_edges.iter().skip(next_search_start).position(|&e| e).map(|p| p + next_search_start) {
            next_search_start = edge_index + 1;
            let (v1, v2) = edges.edge_at_index(edge_index);
            let v1_pos = vertices[v1];
            let v2_pos = vertices[v2];

            let v1_neighs = edges.neighbors_of(v1);
            let v2_neighs = edges.neighbors_of(v2);
            let shared_neighs = v1_neighs.filter(|v| v2_neighs.clone().contains(v));

            //see coment above 'discover_faces loop: we choose the rotation by taking v3
            //as having positive component in direction_check_vec's direction
            let direction_check_vec = v1_pos.to_vec3().cross(v2_pos.to_vec3());

            let mut v3 = usize::MAX; 
            let mut index_2_3 = usize::MAX;
            let mut index_3_1 = usize::MAX;
            let mut dist = f32::MAX;
            for v3_candidate in shared_neighs {
                let new_pos = vertices[v3_candidate];
                let new_2_3 = edges.directed_index(v2, v3_candidate);
                let new_3_1 = edges.directed_index(v3_candidate, v1);
                if !unused_edges[new_2_3] || !unused_edges[new_3_1] {
                    continue;
                }
                if direction_check_vec.dot(new_pos.to_vec3()) < 0.0 {
                    continue;
                }
                //only closest common neighbor in given direction shares same face
                let new_dist = (new_pos - v1_pos).length() + (new_pos - v2_pos).length();
                if new_dist > dist {
                    continue;
                }
                v3 = v3_candidate;
                index_2_3 = new_2_3;
                index_3_1 = new_3_1;
                dist = new_dist;
            }
            //TODO: find out why this is needed
            if v3 == usize::MAX {
                continue;
            }
            unused_edges[edge_index] = false;
            unused_edges[index_2_3] = false;
            unused_edges[index_3_1] = false;
            
            let v3_pos = vertices[v3];
            let dir_1_2 = v2_pos - v1_pos;
            let dir_2_3 = v3_pos - v2_pos;
            let face_normal = Vec3::cross(dir_1_2, dir_2_3).normalized();
            normals.push(face_normal);
            triangles.push([v1, v2, v3]);
        }
        assert!(!unused_edges.into_iter().any(|e| e));

        (triangles, normals)
    }

    fn for_each_vertex_as_vec3(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for v in &mut self.vertices {
            *v = f(v.to_vec3()).to_pos3();
        }
    }

    fn rescale_vectices(mut self, scale: f32) -> Self {
        self.for_each_vertex_as_vec3(|v| v * scale);
        self
    }

    pub fn sorted_triangles(&self) -> impl Iterator<Item = [usize; 3]> + '_ {
        self.triangles.iter().map(|corners| {
            let mut corners = *corners;
            corners.sort();
            corners
        })
    }


    /// connects the closest vertices to have edges,
    /// positions are assumed to lie centered around the origin
    /// assumes all edges to have same length.
    fn new_uniform_from_positions(vertex_positions: Vec<Pos3>) -> Self 
    {
        debug_assert!(vertex_positions.len() >= 4); //can't make a hull of 3 points in 3d

        let neighbor_vertex_dist = {
            let v1 = vertex_positions[0];
            vertex_positions[1..].iter().fold(f32::MAX, 
                |acc, &v| f32::min((v - v1).length(), acc))
        };

        let mut vertex_neighbors = EdgeList::from_iter(
            find_related!(vertex_positions, vertex_positions, 
                |p1: Pos3, p2: Pos3| is_small((p1 - p2).length() - neighbor_vertex_dist)), 6);
        vertex_neighbors.maybe_shrink_capacity(0);

        let (triangles, face_normals) = 
            Self::discover_faces(
                &vertex_positions, &vertex_neighbors);

        Self { 
            vertices: vertex_positions, 
            edges: vertex_neighbors, 
            triangles, 
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
        Self::new_uniform_from_positions(vs)
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
        Self::new_uniform_from_positions(vs)
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
        Self::new_uniform_from_positions(vs)
            .rescale_vectices(corrected_scale)
    }

    pub fn new_subdivided_icosahedron(scale: f32, divisions: usize) -> Self {
        let iso = Self::new_icosahedron(1.0);
        Self::subdivide_hull(iso, divisions)
            .rescale_vectices(scale)
    }

    pub fn new_subdivided_octahedron(scale: f32, divisions: usize) -> Self {
        let oct = Self::new_octahedron(1.0);
        Self::subdivide_hull(oct, divisions)
            .rescale_vectices(scale)
    }

    pub fn new_subdivided_tetrahedron(scale: f32, divisions: usize) -> Self {
        let tet = Self::new_tetrahedron(1.0);
        Self::subdivide_hull(tet, divisions)
            .rescale_vectices(scale)
    }

    /// each triangle of the original shape is subdivided in smaller triangles, where divisions
    /// denotes the number of vertices added per original boundary
    fn subdivide_hull(plat: Self, divisions: usize) -> Self {
        let scale = plat.vertices.first().map_or(1.0, |v| v.to_vec3().length());
        let embedding = Embedding3D::subdivide_platonic_with_triangles(plat, divisions);
        let (mut vertices, edges) = (embedding.vertices, embedding.edges);

        //bring all vertices to sphere surface
        for pos in &mut vertices {
            *pos = pos.to_vec3().normalized().to_pos3();
        }

        let (triangles, face_normals) = 
            Self::discover_faces(&vertices, &edges);

        Self { 
            vertices, 
            edges, 
            triangles, 
            face_normals 
        }.rescale_vectices(scale)
    }

    pub fn draw_visible_faces(&self, to_screen: &geo::ToScreen, painter: &Painter, stroke: Stroke) 
    {
        for (&normal, boundary) in self.face_normals.iter().zip(self.triangles.iter()) {
            if to_screen.faces_camera(normal) {
                for (&v1, &v2) in boundary.iter().circular_tuple_windows() {
                    let p1 = self.vertices[v1];
                    let p2 = self.vertices[v2];

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
        for (v1, neighs) in self.edges.neighbors().enumerate() {
            let p1 = self.vertices[v1];
            for v2 in neighs {
                if v2 <= v1 { //skip edges already drawn the other way around
                    continue;
                }
                let p2 = self.vertices[v2];
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

pub struct Embedding3D {
    /// all vertices are expected to lie on this surface.
    surface: ConvexTriangleHull,
    /// maps edges of surface to the sequence of vertex indices of self.vertices dividing that edge
    /// (beginning and end vertices have the same index in self.vertices and self.surface.vertex_positions)
    /// Vec is indexed by surface.vertex_neighbors.edge_direction_index
    edge_dividing_vertices: Vec<std::ops::Range<usize>>,
    /// indexed by surface's face index
    inner_vertices: Vec<std::ops::Range<usize>>,

    /// position of each vertex
    vertices: Vec<Pos3>,
    /// neighbors of each vertex
    edges: EdgeList,
}

impl Embedding3D {
    pub fn nr_vertices(&self) -> usize {
        self.vertices.len()
    }

    pub fn vertices(&self) -> &[Pos3] {
        &self.vertices
    }

    fn subdivide_platonic_with_triangles(surface: ConvexTriangleHull, divisions: usize) -> Self {
        let mut vertices = surface.vertices.clone();
        let mut edges = EdgeList::new(6, vertices.len());
        let mut edge_dividing_vertices = vec![0..0; surface.edges.used_space()];
        let mut inner_vertices = Vec::new();

        let scaled_dir = |p1, p2| (p2 - p1) / ((divisions + 1) as f32);
        for boundary in surface.triangles.iter() {
            for (&v1, &v2) in boundary.iter().circular_tuple_windows() {
                if v1 > v2 {
                    continue;
                }
                let p1 = vertices[v1];
                let p2 = vertices[v2];
                let dir = scaled_dir(p1, p2);
                let fst_inner_edge_vertex = vertices.len();
                for steps in 1..=divisions {
                    let p = p1 + (steps as f32) * dir;
                    let v = edges.push();
                    debug_assert_eq!(v, vertices.len());
                    vertices.push(p);
                }
                //this relies on the triangles boundaries beeing in the same order as 
                //when created, else this range may be empty.
                let inner = fst_inner_edge_vertex..(vertices.len());
                let fw_edge = surface.edges.directed_index(v1, v2);
                let bw_edge = surface.edges.directed_index(v2, v1);
                edge_dividing_vertices[fw_edge] = inner.clone();
                edge_dividing_vertices[bw_edge] = inner.clone();
                edges.add_path_edges(iter::once(v1).chain(inner).chain(iter::once(v2)));
            }
        }
        assert_eq!(vertices.len(), edges.len());

        for [v1, v2, v3] in surface.sorted_triangles() {
            let fst_inner_face_vertex = vertices.len();
            let edge_1_2_index = surface.edges.directed_index(v1, v2);
            let edge_1_3_index = surface.edges.directed_index(v1, v3);
            let edge_2_3_index = surface.edges.directed_index(v2, v3);
            let edge_1_2 = edge_dividing_vertices[edge_1_2_index].clone(); 
            let edge_1_3 = edge_dividing_vertices[edge_1_3_index].clone();
            let edge_2_3 = edge_dividing_vertices[edge_2_3_index].clone();

            let mut last_levels_nodes = vec![v1];
            let p1 = vertices[v1];
            let p2 = vertices[v2];
            let p3 = vertices[v3];
            let dir_1_2 = scaled_dir(p1, p2);
            let dir_2_3 = scaled_dir(p2, p3);
            //taken more or less directly from GraphDrawing::triangulated_regular_polygon
            if divisions > 0 {          
                for (bd_1_2, bd_2_3, level) in itertools::izip!(edge_1_2, edge_1_3, 1..) {
                    let level_start_pos = p1 + (level as f32) * dir_1_2;
                    let mut this_levels_nodes =vec![bd_1_2];
                    let nr_inner_nodes = level - 1;
                    for node in 1..=nr_inner_nodes {
                        let node_pos = level_start_pos + (node as f32) * dir_2_3;
                        let node_index = edges.push();
                        debug_assert_eq!(node_index, vertices.len());
                        vertices.push(node_pos);
                        this_levels_nodes.push(node_index);
                    }
                    this_levels_nodes.push(bd_2_3);
                    //connect level to itself
                    edges.add_path_edges_ref(this_levels_nodes.iter());
                    //connect level to previous level
                    //note: this differs from GraphDrawing::triangulated_regular_polygon, because we 
                    //  dont know how often we see an ico-edge as one of edge_1_2 and edge_2_3. 
                    //to not add existing connections, we connected the divided ico edges already when 
                    //  they where constructed.
                    //level 1: none      2: /\     3: /\/\     4: /\/\/\       ...
                    let window = &this_levels_nodes[1..(this_levels_nodes.len() - 1)];
                    edges.add_path_edges_ref(last_levels_nodes.iter()
                        .interleave(window.iter()));

                    last_levels_nodes = this_levels_nodes;
                }
            }
            //connect last inner level to upper edge, e.g. edge_2_3
            edges.add_path_edges(last_levels_nodes.into_iter().interleave(edge_2_3));

            let faces_inner = fst_inner_face_vertex..(vertices.len());
            inner_vertices.push(faces_inner);
        }

        Self { 
            surface,
            edge_dividing_vertices,
            inner_vertices,
            vertices, 
            edges,
        }
    }

    pub fn new_subdivided_icosahedron(scale: f32, divisions: usize) -> Self {
        let ico = ConvexTriangleHull::new_icosahedron(scale);
        Self::subdivide_platonic_with_triangles(ico, divisions)
    }

    pub fn new_subdivided_subdivided_icosahedron(scale: f32, d1: usize, d2: usize) -> Self {
        let ico = ConvexTriangleHull::new_subdivided_icosahedron(scale, d1);
        Self::subdivide_platonic_with_triangles(ico, d2)
    }

    pub fn new_subdivided_tetrahedron(scale: f32, divisions: usize) -> Self {
        let tet = ConvexTriangleHull::new_tetrahedron(scale);
        Self::subdivide_platonic_with_triangles(tet, divisions)
    }

    pub fn new_subdivided_octahedron(scale: f32, divisions: usize) -> Self {
        let oct = ConvexTriangleHull::new_octahedron(scale);
        Self::subdivide_platonic_with_triangles(oct, divisions)
    }

    pub fn draw_visible_edges(&mut self, to_screen: &geo::ToScreen, painter: &Painter, stroke: Stroke) {
        let draw_line = |vertices: &[Pos3], v1, v2| {
            let edge = [
                to_screen.apply(vertices[v1]), 
                to_screen.apply(vertices[v2])];
            let line = Shape::LineSegment { points: edge, stroke };
            painter.add(line);
        };
        let iter = itertools::izip!(
            self.surface.face_normals.iter(), 
            self.surface.sorted_triangles(),
        );
        for (&normal, [v1, v2, v3]) in iter {
            if !to_screen.faces_camera(normal) {
                continue;
            }
            let p1 = self.vertices[v1];
            let p2 = self.vertices[v2];
            let p3 = self.vertices[v3];
            if !to_screen.triangle_visible(p1, p2, p3) {
                continue;
            }
            //draw visible edges of self.surface
            draw_line(&self.vertices, v1, v2);
            draw_line(&self.vertices, v2, v3);
            draw_line(&self.vertices, v3, v1);

            //draw inner edges
            //this dosn't draw each actual tiny edge, but instead all edges lying in 
            //  one line at once.
            let edge_1_2_index = self.surface.edges.directed_index(v1, v2);
            let edge_1_3_index = self.surface.edges.directed_index(v1, v3);
            let edge_2_3_index = self.surface.edges.directed_index(v2, v3);
            let edge_1_2 = self.edge_dividing_vertices[edge_1_2_index].clone(); 
            let edge_1_3 = self.edge_dividing_vertices[edge_1_3_index].clone();
            let edge_2_3 = self.edge_dividing_vertices[edge_2_3_index].clone();
            for (v1, v2) in edge_1_2.clone().zip(edge_1_3.clone()) {
                draw_line(&self.vertices, v1, v2);
            }
            for (v1, v2) in edge_2_3.clone().zip(edge_1_3.clone()) {
                draw_line(&self.vertices, v1, v2);
            }
            for (v1, v2) in edge_1_2.clone().zip(edge_2_3.clone().rev()) {
                draw_line(&self.vertices, v1, v2);
            }
        }
    }

    pub fn find_local_minimum(&self, mut potential: impl FnMut(Pos3) -> f32, node_hint: usize) -> (usize, f32) {
        let mut nearest = node_hint;
        let mut smallest_pot = potential(self.vertices[node_hint]);
        let mut maybe_neighbor_better = true;
        while maybe_neighbor_better {
            maybe_neighbor_better = false;
            for neigh in self.edges.neighbors_of(nearest) {
                let neigh_pos = self.vertices[neigh];
                let neigh_pot = potential(neigh_pos);
                if neigh_pot < smallest_pot {
                    nearest = neigh;
                    smallest_pot = neigh_pot;
                    maybe_neighbor_better = true;
                }
            }
        }
        (nearest, smallest_pot)
    }    

    /// everything in queue is starting point and expected to already have the correct distance
    pub fn calc_distances_to(&self, queue: &mut VecDeque<usize>, distances: &mut Vec<isize>) {
        self.edges.calc_distances_to(queue, distances)
    }
}





















