use std::collections::BTreeSet;

use bool_csr::BoolCSR;
use itertools::{Itertools, izip};
use smallvec::{SmallVec, smallvec};

use crate::geo::{self, Pos3, Vec3, pos3, vec3};

use crate::graph::*;

mod equivalence;
pub use equivalence::*;

pub mod grid;

mod bidirectional_range;
use bidirectional_range::*;

mod convex_hull;
use convex_hull::*;

fn is_small(x: f32) -> bool {
    x.abs() < 1e-4
}

/// connects the closest vertices to have edges,
/// assumes all connected vertices to have same respective distances.
fn edges_from_uniform_positions(vertex_positions: &[Pos3]) -> EdgeList {
    debug_assert!(vertex_positions.len() >= 4); //can't make a hull of 3 points in 3d

    let neighbor_vertex_dist = {
        let v1 = vertex_positions[0];
        vertex_positions[1..]
            .iter()
            .fold(f32::MAX, |acc, &v| f32::min((v - v1).length(), acc))
    };
    let related = vertex_positions.iter().map(|&p1| {
        vertex_positions.iter().enumerate().filter_map(move |(i, &p2)| {
            is_small((p1 - p2).length() - neighbor_vertex_dist).then_some(i)
        })
    });
    let max_neighbors = related.clone().fold(0, |acc, neighs| acc.max(neighs.count()));
    EdgeList::from_iter(related, max_neighbors)
}

fn normalize_positions(positions: &mut [Pos3]) {
    //makes only sense if graph is centered around origin
    debug_assert!(Pos3::average(positions.iter().copied()).to_vec3().length() < 1e-4);
    for p in positions {
        *p = p.to_vec3().normalized().to_pos3();
    }
}

fn sort_neigbors(edges: &mut EdgeList, positions: &[Pos3]) {
    for (v1, neighs) in edges.neighbors_mut().enumerate() {
        let p1 = positions[v1];

        let diff = |&i2: &Index| {
            let v2 = i2.get().unwrap();
            let p2 = positions[v2];
            p2 - p1
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
        let order_dirs = |d1: Vec3, d2: Vec3| {
            let cmp_x = order_floats(d1.x, d2.x);
            if cmp_x.is_ne() {
                return cmp_x;
            }
            let cmp_y = order_floats(d1.y, d2.y);
            if cmp_y.is_ne() {
                return cmp_y;
            }
            order_floats(d1.z, d2.z)
        };

        neighs.sort_by(|n1, n2| order_dirs(diff(n1), diff(n2)));
    }
}

/// to improve positioning of characters when switching to a 2d embedding,
/// the embedding sits outside the x-y plane.
pub const Z_OFFSET_2D: f32 = 0.5;

pub struct Embedding3D {
    /// all vertices are expected to lie on this surface.
    surface: ConvexHull,

    shape: Shape,

    /// maps edges of surface to the sequence of vertex indices of self.vertices dividing that edge
    /// (beginning and end vertices have the same index in self.vertices and self.surface.vertex_positions)
    /// Vec is indexed by surface.vertex_neighbors.edge_direction_index
    edge_dividing_vertices: Vec<BidirectionalRange>,
    /// indexed by surface's face index
    inner_vertices: Vec<std::ops::Range<usize>>,

    /// position of each vertex
    vertices: Vec<Pos3>,
    /// neighbors of each vertex
    edges: EdgeList,

    sym_group: SymGroup,
}

impl Embedding3D {
    fn uses_surface(&self) -> bool {
        !self.surface.vertices.is_empty()
    }

    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    pub fn edges(&self) -> &EdgeList {
        &self.edges
    }

    pub fn into_edges(self) -> EdgeList {
        self.edges
    }

    pub fn nr_vertices(&self) -> usize {
        debug_assert_eq!(self.vertices.len(), self.edges.nr_vertices());
        self.vertices.len()
    }

    pub fn positions(&self) -> &[Pos3] {
        &self.vertices
    }

    pub fn sym_group(&self) -> &SymGroup {
        &self.sym_group
    }

    /// returns whih edge lengh should be considered when displaying other features,
    /// e.g. how large characters and markers are drawn.
    /// also used to determine on tori which edges are not drawn, because the embedding looks ugly otherwise.
    pub fn max_scaling_edge_length(&self) -> f32 {
        if matches!(self.shape, Shape::SquareTorus | Shape::TriangTorus) {
            // just above 0.5 vs just below sqrt(2)
            if self.nr_vertices() > 4 { 0.55 } else { 1.4 }
        } else if !self.vertices.is_empty() {
            let p0 = self.vertices[0];
            1.5 * self
                .edges()
                .neighbors_of(0)
                .map(|v1| (self.vertices[v1] - p0).length_sq())
                .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or(1e10)
                .sqrt()
        } else {
            1e10
        }
    }

    #[inline(always)]
    fn add_vertex(&mut self, pos: Pos3) -> usize {
        let new_index = self.nr_vertices();
        self.vertices.push(pos);
        self.edges.push();
        new_index
    }

    fn subdivide_surface_with_triangles(
        surface: ConvexHull,
        divisions: usize,
        is_platonic: bool,
        shape: Shape,
    ) -> Self {
        let mut vertices = surface.vertices.clone();
        let mut edges = EdgeList::new(6, vertices.len());
        let mut edge_dividing_vertices =
            vec![BidirectionalRange::uninitialized(); surface.edges.used_space()];
        let mut inner_vertices = Vec::new();

        let scaled_dir = |p1, p2| (p2 - p1) / ((divisions + 1) as f32);
        surface.edges.for_each_edge(|v2, v1| {
            debug_assert!(v1 < v2); //every edge is only iterated over in one direction
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
            let inner = BidirectionalRange::new_forward(fst_inner_edge_vertex, vertices.len());
            let fw_edge = surface.edges.directed_index(v1, v2);
            let bw_edge = surface.edges.directed_index(v2, v1);
            debug_assert!(edge_dividing_vertices[fw_edge].is_uninitialized());
            debug_assert!(edge_dividing_vertices[bw_edge].is_uninitialized());
            edge_dividing_vertices[fw_edge] = inner;
            edge_dividing_vertices[bw_edge] = inner.reversed();
            use std::iter::once;
            edges.add_path_edges(once(v1).chain(inner).chain(once(v2)));
        });
        assert_eq!(vertices.len(), edges.nr_vertices());

        for face in surface.faces.iter_rows() {
            let &[v1, v2, v3] = face else {
                panic!();
            };
            let fst_inner_face_vertex = vertices.len();
            let edge_1_2_index = surface.edges.directed_index(v1, v2);
            let edge_1_3_index = surface.edges.directed_index(v1, v3);
            let edge_2_3_index = surface.edges.directed_index(v2, v3);
            let edge_1_2 = edge_dividing_vertices[edge_1_2_index];
            let edge_1_3 = edge_dividing_vertices[edge_1_3_index];
            let edge_2_3 = edge_dividing_vertices[edge_2_3_index];

            let mut last_levels_nodes = vec![v1];
            let p1 = vertices[v1];
            let p2 = vertices[v2];
            let p3 = vertices[v3];
            let dir_1_2 = scaled_dir(p1, p2);
            let dir_2_3 = scaled_dir(p2, p3);
            if divisions > 0 {
                for (bd_1_2, bd_2_3, level) in itertools::izip!(edge_1_2, edge_1_3, 1..) {
                    let level_start_pos = p1 + (level as f32) * dir_1_2;
                    let mut this_levels_nodes = vec![bd_1_2];
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
                    edges.add_path_edges(this_levels_nodes.iter().copied());
                    //connect level to previous level
                    //note: this differs from GraphDrawing::triangulated_regular_polygon, because we
                    //  dont know how often we see an ico-edge as one of edge_1_2 and edge_2_3.
                    //to not add existing connections, we connected the divided ico edges already when
                    //  they where constructed.
                    //level 1: none      2: /\     3: /\/\     4: /\/\/\       ...
                    let window = &this_levels_nodes[1..(this_levels_nodes.len() - 1)];
                    edges.add_path_edges(
                        last_levels_nodes.iter().interleave(window.iter()).copied(),
                    );

                    last_levels_nodes = this_levels_nodes;
                }
            }
            //connect last inner level to upper edge, e.g. edge_2_3
            edges.add_path_edges(last_levels_nodes.into_iter().interleave(edge_2_3));

            let faces_inner = fst_inner_face_vertex..(vertices.len());
            inner_vertices.push(faces_inner);
        }

        let sym_group = SymGroup::None(NoSymmetry::new(vertices.len()));

        sort_neigbors(&mut edges, &vertices);
        let mut res = Self {
            surface,
            shape,
            edge_dividing_vertices,
            inner_vertices,
            vertices,
            edges,
            sym_group,
        };
        if is_platonic {
            if let Some(expl) = ExplicitClasses::new_for_subdivided_platonic(&res, divisions) {
                res.sym_group = SymGroup::Explicit(expl);
            }
        }
        res
    }

    fn new_subdivided_icosahedron(divisions: usize) -> Self {
        let ico = ConvexHull::new_icosahedron();
        Self::subdivide_surface_with_triangles(ico, divisions, true, Shape::Icosahedron)
    }

    fn new_subdivided_subdivided_icosahedron(d1: usize, d2: usize, shape: Shape) -> Self {
        let ico = ConvexHull::new_subdivided_icosahedron(d1);
        Self::subdivide_surface_with_triangles(ico, d2, false, shape)
    }

    fn new_subdivided_tetrahedron(divisions: usize) -> Self {
        let tet = ConvexHull::new_tetrahedron();
        Self::subdivide_surface_with_triangles(tet, divisions, true, Shape::Tetrahedron)
    }

    fn new_subdivided_octahedron(divisions: usize) -> Self {
        let oct = ConvexHull::new_octahedron();
        Self::subdivide_surface_with_triangles(oct, divisions, true, Shape::Octahedron)
    }

    /// face_info stores circumference in .0 and weather to make the inner vertices of given face type visible in .1
    fn embed_archimedian_solid(
        mut vertices: Vec<Pos3>,
        mut edges: EdgeList,
        face_info: &[(usize, bool)],
        shape: Shape,
    ) -> (Self, Vec<Vec<usize>>) {
        //vertices should be centered around origin (among other things...)
        debug_assert!(Pos3::average(vertices.iter().copied()).to_vec3().length() < 1e-4);
        //all visible face inner thingies must be at the start, because of how the data structure works
        debug_assert!(face_info.iter().tuple_windows().all(|((_, a), (_, b))| a >= b));

        let mut faces = Vec::new();
        let mut hull = ConvexHull {
            vertices: vertices.clone(),
            edges: edges.clone(),
            face_normals: Vec::new(),
            faces: BoolCSR::new(),
            dual_edges: EdgeList::empty(), //caution: needs to be set to correct values at end
        };
        for &(circumference, _) in face_info {
            debug_assert!(circumference > 2);
            let new_faces = find_all_circles(&edges, circumference).into_iter().collect_vec();
            debug_assert!(new_faces.iter().all(|f| f.len() == circumference));
            faces.push(new_faces);
        }

        fn direct_triangle(pos: &[Pos3], [v1, v2, v3]: [usize; 3]) -> [usize; 3] {
            let p1 = pos[v1].to_vec3();
            let p2 = pos[v2].to_vec3();
            let p3 = pos[v3].to_vec3();
            if Vec3::cross(p1, p2).dot(p3) < 0.0 {
                [v2, v1, v3]
            } else {
                [v1, v2, v3]
            }
        }

        for (faces, &(_, triangulate)) in faces.iter_mut().zip(face_info) {
            for face in faces.iter_mut() {
                let compute_normal = |vs: &[Pos3], v1, v2, v3| {
                    let p1: Pos3 = vs[v1];
                    let p2: Pos3 = vs[v2];
                    let p3: Pos3 = vs[v3];
                    let dir1_2 = p2 - p1;
                    let dir1_3 = p3 - p1;
                    let normal = Vec3::cross(dir1_2, dir1_3).normalized();
                    if normal.dot(p1.to_vec3()) > 0.0 {
                        normal
                    } else {
                        -normal
                    }
                };

                {
                    // enforce consistent direction in face
                    let [v1, v2, v3, ..] = face[..] else {
                        panic!();
                    };
                    face[0..3].clone_from_slice(&direct_triangle(&hull.vertices, [v1, v2, v3]));
                }
                //order face s.t. neighboring vertices in graph are neighbors in face
                'find_swap: for i in 2..face.len() {
                    let last_v = face[i - 2];
                    for j in (i - 1)..face.len() {
                        if edges.neighbors_of(last_v).contains(&face[j]) {
                            face.swap(i - 1, j);
                            continue 'find_swap;
                        }
                    }
                    panic!();
                }
                if triangulate {
                    let center = Pos3::average(face.iter().map(|&v| hull.vertices[v]));
                    vertices.push(center);
                    edges.push();
                    hull.vertices.push(center);
                    let center_v = hull.edges.push();
                    for (v1, v2) in face.iter().copied().circular_tuple_windows() {
                        debug_assert!(edges.neighbors_of(v1).contains(&v2));
                        hull.edges.add_edge(v1, center_v);
                        edges.add_edge(v1, center_v);
                        let tri = direct_triangle(&hull.vertices, [center_v, v1, v2]);
                        hull.face_normals.push(compute_normal(&hull.vertices, center_v, v1, v2));
                        hull.faces.add_row(tri.into_iter());
                    }
                } else {
                    let [v1, v2, v3, ..] = face[..] else {
                        panic!();
                    };
                    hull.face_normals.push(compute_normal(&hull.vertices, v1, v2, v3));
                    hull.faces.add_row(face.iter().copied());
                }
            }
        }

        let hull_dual_edges = ConvexHull::discover_dual(&hull.faces);
        hull.dual_edges = hull_dual_edges;

        let edge_dividing_vertices =
            vec![BidirectionalRange::uninitialized(); hull.edges.used_space()];
        let inner_vertices = vec![0..0; hull.face_normals.len()];
        let sym_group = SymGroup::None(NoSymmetry::new(vertices.len()));

        sort_neigbors(&mut edges, &vertices);
        let res = Self {
            surface: hull,
            shape,
            edge_dividing_vertices,
            inner_vertices,
            vertices,
            edges,
            sym_group,
        };
        (res, faces.join(&Vec::new()))
    }

    fn football_vertices() -> Vec<Pos3> {
        let phi = (1.0 + f32::sqrt(5.0)) / 2.0;
        #[allow(unused_parens)]
        let mut vertices = vec![
            pos3(0.0, 1.0, 3.0 * phi),
            pos3(0.0, 1.0, -3.0 * phi),
            pos3(0.0, -1.0, 3.0 * phi),
            pos3(0.0, -1.0, -3.0 * phi),
            pos3(3.0 * phi, 0.0, 1.0),
            pos3(3.0 * phi, 0.0, -1.0),
            pos3(-3.0 * phi, 0.0, 1.0),
            pos3(-3.0 * phi, 0.0, -1.0),
            pos3(1.0, 3.0 * phi, 0.0),
            pos3(1.0, -3.0 * phi, 0.0),
            pos3(-1.0, 3.0 * phi, 0.0),
            pos3(-1.0, -3.0 * phi, 0.0),
            pos3(1.0, (2.0 + phi), 2.0 * phi),
            pos3(1.0, (2.0 + phi), -2.0 * phi),
            pos3(1.0, -(2.0 + phi), 2.0 * phi),
            pos3(1.0, -(2.0 + phi), -2.0 * phi),
            pos3(-1.0, (2.0 + phi), 2.0 * phi),
            pos3(-1.0, (2.0 + phi), -2.0 * phi),
            pos3(-1.0, -(2.0 + phi), 2.0 * phi),
            pos3(-1.0, -(2.0 + phi), -2.0 * phi),
            pos3(2.0 * phi, 1.0, (2.0 + phi)),
            pos3(2.0 * phi, 1.0, -(2.0 + phi)),
            pos3(2.0 * phi, -1.0, (2.0 + phi)),
            pos3(2.0 * phi, -1.0, -(2.0 + phi)),
            pos3(-2.0 * phi, 1.0, (2.0 + phi)),
            pos3(-2.0 * phi, 1.0, -(2.0 + phi)),
            pos3(-2.0 * phi, -1.0, (2.0 + phi)),
            pos3(-2.0 * phi, -1.0, -(2.0 + phi)),
            pos3((2.0 + phi), 2.0 * phi, 1.0),
            pos3((2.0 + phi), 2.0 * phi, -1.0),
            pos3((2.0 + phi), -2.0 * phi, 1.0),
            pos3((2.0 + phi), -2.0 * phi, -1.0),
            pos3(-(2.0 + phi), 2.0 * phi, 1.0),
            pos3(-(2.0 + phi), 2.0 * phi, -1.0),
            pos3(-(2.0 + phi), -2.0 * phi, 1.0),
            pos3(-(2.0 + phi), -2.0 * phi, -1.0),
            pos3(phi, 2.0, (2.0 * phi + 1.0)),
            pos3(phi, 2.0, -(2.0 * phi + 1.0)),
            pos3(phi, -2.0, (2.0 * phi + 1.0)),
            pos3(phi, -2.0, -(2.0 * phi + 1.0)),
            pos3(-phi, 2.0, (2.0 * phi + 1.0)),
            pos3(-phi, 2.0, -(2.0 * phi + 1.0)),
            pos3(-phi, -2.0, (2.0 * phi + 1.0)),
            pos3(-phi, -2.0, -(2.0 * phi + 1.0)),
            pos3((2.0 * phi + 1.0), phi, 2.0),
            pos3((2.0 * phi + 1.0), phi, -2.0),
            pos3((2.0 * phi + 1.0), -phi, 2.0),
            pos3((2.0 * phi + 1.0), -phi, -2.0),
            pos3(-(2.0 * phi + 1.0), phi, 2.0),
            pos3(-(2.0 * phi + 1.0), phi, -2.0),
            pos3(-(2.0 * phi + 1.0), -phi, 2.0),
            pos3(-(2.0 * phi + 1.0), -phi, -2.0),
            pos3(2.0, (2.0 * phi + 1.0), phi),
            pos3(2.0, (2.0 * phi + 1.0), -phi),
            pos3(2.0, -(2.0 * phi + 1.0), phi),
            pos3(2.0, -(2.0 * phi + 1.0), -phi),
            pos3(-2.0, (2.0 * phi + 1.0), phi),
            pos3(-2.0, (2.0 * phi + 1.0), -phi),
            pos3(-2.0, -(2.0 * phi + 1.0), phi),
            pos3(-2.0, -(2.0 * phi + 1.0), -phi),
        ];
        normalize_positions(&mut vertices);
        vertices
    }

    /// turns each edge into a path of length `nr_subdivisions + 1`,
    /// e.g. `graph.subdivide_all_edges(0)` does nothing
    /// this assumes, that `self.surface` contains the full graph of `self` so far,
    /// especially no edge of `self.surface` may already be divided.
    ///
    /// if not `update_faces`, only edge and vertex information is updated,
    /// not things like the convex hull where the graph is embedded on etc.
    fn subdivide_all_edges(&mut self, nr_subdivisions: usize, update_faces: bool) {
        if nr_subdivisions == 0 {
            return;
        }
        let nr_og_vertices = self.nr_vertices();
        if update_faces {
            assert!(self.surface.nr_vertices() >= self.nr_vertices());
            debug_assert!(self.surface.vertices[..self.nr_vertices()] == self.vertices[..]);
        }

        let mut v1_neighbors = Vec::new();
        for v1 in 0..nr_og_vertices {
            v1_neighbors.clear();
            v1_neighbors.extend(self.edges.neighbors_of(v1));
            for &v2 in &v1_neighbors {
                if v2 < nr_og_vertices {
                    // if v2 < v1, then v2 would have been visited first and the edge to v1 would have been cut.
                    debug_assert!(v2 > v1);
                    let pos1 = self.vertices[v1];
                    let pos2 = self.vertices[v2];
                    let step = (pos2 - pos1) / (nr_subdivisions as f32 + 1.0);
                    self.edges.remove_edge(v1, v2);
                    let mut curr = v1;
                    let fst_inner = self.vertices.len();
                    for i in 1..=nr_subdivisions {
                        let next = self.add_vertex(pos1 + (i as f32) * step);
                        self.edges.add_edge(curr, next);
                        curr = next;
                    }
                    self.edges.add_edge(curr, v2);

                    if !update_faces {
                        continue;
                    }

                    let inner_end = curr + 1;
                    let inner = BidirectionalRange::new_forward(fst_inner, inner_end);
                    let fw_edge = self.surface.edges.directed_index(v1, v2);
                    let bw_edge = self.surface.edges.directed_index(v2, v1);
                    debug_assert!(self.edge_dividing_vertices[fw_edge].is_uninitialized());
                    debug_assert!(self.edge_dividing_vertices[bw_edge].is_uninitialized());
                    self.edge_dividing_vertices[fw_edge] = inner;
                    self.edge_dividing_vertices[bw_edge] = inner.reversed();
                }
            }
        }
        sort_neigbors(&mut self.edges, &self.vertices);
        self.sym_group = SymGroup::None(NoSymmetry::new(self.vertices.len()));
    }

    /// all vertices with distance <= `n` become meighbors (except no vertex is neighbor of itself).
    ///
    /// note: the symmetry is kept (or increased, but we ignore this possibility) by this operation.
    #[allow(dead_code)]
    fn edge_pow(&mut self, n: usize) {
        let old_edges = &self.edges;
        let mut new_edges = old_edges.clone();
        for _ in 0..n {
            let new_neighbors = izip!(0.., new_edges.neighbors())
                .map(|(v, ns)| {
                    let mut new_neighs = Vec::from_iter(ns.clone());
                    for n in ns {
                        new_neighs.extend(old_edges.neighbors_of(n));
                    }
                    new_neighs.sort();
                    new_neighs.dedup();
                    new_neighs.retain(|&u| u != v);
                    new_neighs
                })
                .collect_vec();
            let max_neighbors = new_neighbors.iter().map(Vec::len).max().unwrap_or(0);
            new_edges = EdgeList::from_iter(
                new_neighbors.into_iter().map(|ns| ns.into_iter()),
                max_neighbors,
            );
        }
        self.edges = new_edges;
    }

    /// custom subdivision of graph described in Fabian Hamann's masters thesis.
    pub fn new_subdivided_football(divisions: usize, show_hex_mid: bool) -> Self {
        let vertices = Self::football_vertices();
        let edges = edges_from_uniform_positions(&vertices);
        debug_assert_eq!(vertices.len(), 60);
        debug_assert_eq!(edges.count_entries(), 180);

        let face_info = [(6, show_hex_mid), (5, false)];
        let shape = if show_hex_mid {
            Shape::FabianHamann
        } else {
            Shape::Football
        };
        let (mut res, _faces) = Self::embed_archimedian_solid(vertices, edges, &face_info, shape);
        res.subdivide_all_edges(divisions, true);
        res
    }

    fn dodecahedron_vertices() -> Vec<Pos3> {
        let a = 1.0;
        let p = (1.0 + f32::sqrt(5.0)) / 2.0;
        let d = 2.0 / (1.0 + f32::sqrt(5.0));
        let mut vs = vec![
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
        normalize_positions(&mut vs);
        vs
    }

    fn new_subdivided_dodecahedron(
        divisions: usize,
        show_divisions: bool,
        triangulate: bool,
    ) -> Self {
        let vertices = Self::dodecahedron_vertices();
        let edges = edges_from_uniform_positions(&vertices);
        let face_info = [(5, show_divisions || triangulate)];

        let (mut res, _faces) =
            Self::embed_archimedian_solid(vertices, edges, &face_info, Shape::Dodecahedron);
        if triangulate {
            res.surface.normalize_positions();
            res = Self::subdivide_surface_with_triangles(
                res.surface,
                divisions,
                false,
                Shape::Dodecahedron,
            );
        } else {
            res.subdivide_all_edges(divisions, true);
        }
        res
    }

    /// u and v are assumed to be vertices in self.surface
    fn subdivided_edge(&self, u: usize, v: usize) -> BidirectionalRange {
        let edge_index = self.surface.edges.directed_index(u, v);
        self.edge_dividing_vertices[edge_index]
    }

    fn new_subdivided_cube(divisions: usize) -> Self {
        let p = 1.0;
        let n = -1.0;
        let mut vertices = vec![
            pos3(p, p, p),
            pos3(p, p, n),
            pos3(p, n, p),
            pos3(p, n, n),
            pos3(n, p, p),
            pos3(n, p, n),
            pos3(n, n, p),
            pos3(n, n, n),
        ];
        normalize_positions(&mut vertices);
        let edges = edges_from_uniform_positions(&vertices);
        let face_info = [(4, false)];
        let (mut res, faces) =
            Self::embed_archimedian_solid(vertices, edges, &face_info, Shape::Cube);
        res.subdivide_all_edges(divisions, true);

        // add inner vertices
        res.inner_vertices.clear();
        for face in faces {
            let [v1, v2, v3, v4] = face[..] else {
                panic!();
            };
            debug_assert!(res.surface.edges.has_path(&face));
            debug_assert!(res.surface.edges.has_edge(v1, v4));
            if divisions == 0 {
                let nr = res.nr_vertices();
                res.inner_vertices.push(nr..nr);
                continue;
            }
            let corner = res.vertices[v1];
            let scale = 1.0 / (divisions as f32 + 1.0);
            let dir_a = scale * (res.vertices[v2] - corner);
            let dir_b = scale * (res.vertices[v4] - corner);

            let start = res.nr_vertices();
            for ia in 1..=divisions {
                for ib in 1..=divisions {
                    res.edges.push();
                    res.vertices.push(corner + (ia as f32) * dir_a + (ib as f32) * dir_b);
                }
            }
            let end = res.nr_vertices();
            res.inner_vertices.push(start..end);

            let index = |ia, ib| start + ia * divisions + ib;

            let edge_12 = res.subdivided_edge(v1, v2);
            let edge_43 = res.subdivided_edge(v4, v3);
            for (ia, start_a, end_a) in izip!(0.., edge_12, edge_43) {
                res.edges.add_edge(index(ia, 0), start_a);
                res.edges.add_edge(index(ia, divisions - 1), end_a);
                for (ib1, ib2) in (0..divisions).tuple_windows() {
                    res.edges.add_edge(index(ia, ib1), index(ia, ib2));
                }
            }
            let edge_23 = res.subdivided_edge(v2, v3);
            let edge_14 = res.subdivided_edge(v1, v4);
            for (ib, start_b, end_b) in izip!(0.., edge_14, edge_23) {
                res.edges.add_edge(index(0, ib), start_b);
                res.edges.add_edge(index(divisions - 1, ib), end_b);
                for (ia1, ia2) in (0..divisions).tuple_windows() {
                    res.edges.add_edge(index(ia1, ib), index(ia2, ib));
                }
            }
        }
        let sym = ExplicitClasses::new_for_subdivided_platonic(&res, divisions);
        res.sym_group = SymGroup::Explicit(sym.unwrap());
        res
    }

    /// rendered flat as two subdivided equilateral triangles.
    /// if wrap: connect the left and right sides and the top and bottom sides.
    /// this is topologically a torus.
    fn new_subdivided_triangle_grid(len: isize, wrap: bool) -> Self {
        assert!(len >= 2);
        let nr_vertices = (len * len) as usize;
        let mut vertices = Vec::with_capacity(nr_vertices);
        let mut edges = EdgeList::new(6, nr_vertices);
        let this = grid::GridGraph::new(len as usize, grid::Norm::Hex, wrap);

        let scale = 1.0 / (len as f32 + 1.0);
        let x_step = scale * vec3(1.0, 0.0, 0.0);
        let y_step = scale * vec3(-0.5, f32::sqrt(3.0) / 2.0, 0.0);
        let corner = pos3(-0.25, -f32::sqrt(3.0) / 4.0, Z_OFFSET_2D);
        for x in 0..len {
            for y in 0..len {
                let pos = corner + (x as f32) * x_step + (y as f32) * y_step;
                vertices.push(pos);

                let vxy = grid::Coords { x, y };
                let v = this.unchecked_index_of(vxy);
                debug_assert_eq!(v + 1, vertices.len());
                for n in this.neighbor_indices_of(vxy) {
                    if len > 2 || !edges.has_directed_edge(v, n) {
                        edges.add_directed_edge(v, n);
                    }
                }
            }
        }

        let (sym_group, shape) = if wrap {
            (
                SymGroup::Torus6(torus::TorusSymmetry6::new(nr_vertices)),
                Shape::TriangTorus,
            )
        } else {
            (
                SymGroup::None(NoSymmetry::new(nr_vertices)),
                Shape::TriangGrid,
            )
        };

        sort_neigbors(&mut edges, &vertices);
        Self {
            surface: ConvexHull::empty(),
            shape,
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices,
            edges,
            sym_group,
        }
    }

    /// rendered flat as one large subdivided square.
    /// if wrap: connect the left and right sides and the top and bottom sides.
    /// this is topologically a torus.
    fn new_subdivided_squares_grid(len: isize, wrap: bool) -> Self {
        assert!(len >= 2);
        let nr_vertices = (len * len) as usize;
        let mut vertices = Vec::with_capacity(nr_vertices);
        let mut edges = EdgeList::new(6, nr_vertices);
        let this = grid::GridGraph::new(len as usize, grid::Norm::Quad, wrap);

        let scale = 1.0 / (len as f32 + 1.0);
        let x_step = scale * vec3(1.0, 0.0, 0.0);
        let y_step = scale * vec3(0.0, 1.0, 0.0);
        let corner = pos3(-0.5, -0.5, Z_OFFSET_2D);
        for x in 0..len {
            for y in 0..len {
                let pos = corner + (x as f32) * x_step + (y as f32) * y_step;
                vertices.push(pos);

                let vxy = grid::Coords { x, y };
                let v = this.unchecked_index_of(vxy);
                debug_assert_eq!(v + 1, vertices.len());
                for n in this.neighbor_indices_of(vxy) {
                    if len > 2 || !edges.has_directed_edge(v, n) {
                        edges.add_directed_edge(v, n);
                    }
                }
            }
        }

        let (sym_group, shape) = if wrap {
            (
                SymGroup::Torus4(torus::TorusSymmetry4::new(nr_vertices)),
                Shape::SquareTorus,
            )
        } else {
            (
                SymGroup::None(NoSymmetry::new(nr_vertices)),
                Shape::SquareGrid,
            )
        };

        sort_neigbors(&mut edges, &vertices);
        Self {
            surface: ConvexHull::empty(),
            shape,
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices,
            edges,
            sym_group,
        }
    }

    fn new_2d_triangulated_regular_polygon(sides: usize, divisions: usize) -> Self {
        let surface_positions = Vec::from_iter({
            let center = std::iter::once(pos3(0.0, 0.0, Z_OFFSET_2D));
            let corners = (0..sides).map(|i| {
                let angle = std::f32::consts::TAU * ((i as f32 + 0.5) / (sides as f32) + 0.25);
                pos3(angle.cos(), angle.sin(), Z_OFFSET_2D)
            });
            center.chain(corners)
        });

        let mut surface_edges = EdgeList::new(sides, sides + 1);
        for (v1, v2) in (1..(sides + 1)).circular_tuple_windows() {
            surface_edges.add_edge(0, v1);
            surface_edges.add_edge(v1, v2);
        }

        let surface = ConvexHull::new_from_triangulation(surface_positions, surface_edges);
        let mut res = Self::subdivide_surface_with_triangles(
            surface,
            divisions,
            false,
            Shape::RegularPolygon2D(sides as isize),
        );
        sort_neigbors(&mut res.edges, &res.vertices);
        res
    }

    /// draws all visible edges, updates `visible` while doing so.
    fn draw_visible_hull_edges(
        &self,
        to_screen: &geo::ToScreen,
        painter: &egui::Painter,
        stroke: egui::Stroke,
        visible: &mut [bool],
        draw_inner_fast: bool,
    ) {
        debug_assert!(
            !draw_inner_fast
                || matches!(
                    self.shape,
                    Shape::Octahedron
                        | Shape::Tetrahedron
                        | Shape::Icosahedron
                        | Shape::DividedIcosahedron(_)
                        | Shape::RegularPolygon2D(_)
                )
        );

        debug_assert!(self.uses_surface());
        debug_assert_eq!(visible.len(), self.nr_vertices());
        visible.iter_mut().for_each(|v| *v = false);

        let draw_line = |vertices: &[Pos3], v1, v2| {
            let edge = [to_screen.apply(vertices[v1]), to_screen.apply(vertices[v2])];
            let line = egui::Shape::LineSegment { points: edge, stroke };
            painter.add(line);
        };
        let iter = itertools::izip!(
            self.surface.face_normals.iter(),
            self.surface.faces.iter_rows(),
            self.inner_vertices.iter()
        );
        for (&normal, face, inner) in iter {
            if !to_screen.faces_camera(normal) {
                continue;
            }
            if !to_screen.face_maybe_visible(face.iter().map(|&v| self.vertices[v])) {
                continue;
            }

            //draw edges of self.surface
            for (&u, &v) in face.iter().circular_tuple_windows() {
                draw_line(&self.surface.vertices, u, v);

                //update vertex visibility
                visible[v] = true;
                self.subdivided_edge(u, v).for_each(|v| visible[v] = true);
            }
            inner.clone().for_each(|v| visible[v] = true);

            //draw inner edges
            //this dosn't draw each actual tiny edge, but instead all edges lying in
            //  one line at once.
            //note: updating the vertex visibility below results in this function overall
            //  still having the same O complexity, but combining edges this way yields
            //  smaller screenshots.
            if draw_inner_fast {
                let &[v1, v2, v3] = face else {
                    panic!();
                };
                let edge_1_2 = self.subdivided_edge(v1, v2);
                let edge_1_3 = self.subdivided_edge(v1, v3);
                let edge_2_3 = self.subdivided_edge(v2, v3);

                for (v1, v2) in edge_1_2.zip(edge_1_3) {
                    draw_line(&self.vertices, v1, v2);
                }
                for (v1, v2) in edge_2_3.zip(edge_1_3) {
                    draw_line(&self.vertices, v1, v2);
                }
                for (v1, v2) in edge_1_2.zip(edge_2_3.reversed()) {
                    draw_line(&self.vertices, v1, v2);
                }
            } else {
                // assumption: an inner vertex is only connected to other
                // inner vertices of same face or vertices on edges / corners of face
                let inner_edges = self.edges.neighbors().skip(inner.start);
                for (v, ns) in izip!(inner.clone(), inner_edges) {
                    for n in ns {
                        // only draw each edge in one direction, assume that edges
                        // to face edge / corner are to vertices with smaller index
                        debug_assert!(inner.contains(&n) || n < v);
                        if n < v {
                            draw_line(&self.vertices, v, n);
                        }
                    }
                }
            }
        }
    }

    fn draw_torus_edges(
        &self,
        to_screen: &geo::ToScreen,
        painter: &egui::Painter,
        stroke: egui::Stroke,
        draw_diagonals: bool,
    ) {
        debug_assert!(matches!(
            self.shape,
            Shape::TriangTorus | Shape::SquareTorus | Shape::TriangGrid | Shape::SquareGrid
        ));
        let nr_vs = self.nr_vertices();
        let side_len = f32::sqrt(nr_vs as f32) as usize;
        debug_assert_eq!(side_len * side_len, self.nr_vertices());

        let draw_line = |v1, v2| {
            let p1 = self.vertices[v1];
            let p2 = self.vertices[v2];
            let points = [to_screen.apply(p1), to_screen.apply(p2)];
            painter.add(egui::Shape::LineSegment { points, stroke });
        };

        // draw lines stored als blocks
        for block in 0..side_len {
            let block_start = block * side_len;
            let block_last = block_start + (side_len - 1);
            draw_line(block_start, block_last);
        }

        // draw lines stored at same offset in each block
        {
            let first_block_start = 0;
            let last_block_start = nr_vs - side_len;
            for off in 0..side_len {
                draw_line(first_block_start + off, last_block_start + off);
            }
        }

        if draw_diagonals {
            // only go up to snd-last vertex in both direction, as last is identical
            let first_block_rev = (0..(side_len - 1)).rev();
            let blocks_lasts = (1..side_len).map(|n| n * side_len + (side_len - 1));
            for (v1, v2) in izip!(first_block_rev, blocks_lasts) {
                draw_line(v1, v2);
            }

            // only go up to snd-last vertex in both direction, as last is identical
            // only start at snd vertex, as first is drawn above
            let last_block_rev = ((nr_vs - side_len + 1)..(nr_vs - 1)).rev();
            let blocks_firsts = (1..(side_len - 1)).map(|n| n * side_len);
            for (v1, v2) in izip!(last_block_rev, blocks_firsts) {
                draw_line(v1, v2);
            }
        }
    }

    fn draw_all_edges(
        &self,
        to_screen: &geo::ToScreen,
        painter: &egui::Painter,
        stroke: egui::Stroke,
    ) {
        self.edges.for_each_edge(|v1, v2| {
            let p1 = self.vertices[v1];
            let p2 = self.vertices[v2];
            let points = [to_screen.apply(p1), to_screen.apply(p2)];
            painter.add(egui::Shape::LineSegment { points, stroke });
        });
    }

    fn draw_camera_facing_edges(
        &self,
        to_screen: &geo::ToScreen,
        painter: &egui::Painter,
        stroke: egui::Stroke,
        visible: &mut [bool],
    ) {
        let screen_dist = |p: Pos3| to_screen.to_plane.signed_dist(p.to_vec3());
        for (p, vis) in izip!(&self.vertices, visible.iter_mut()) {
            *vis = screen_dist(*p) > -0.2;
        }
        self.edges.for_each_edge(|v1, v2| {
            if !visible[v1] && !visible[v2] {
                return;
            }
            let p1 = self.vertices[v1];
            let p2 = self.vertices[v2];
            let points = [to_screen.apply(p1), to_screen.apply(p2)];

            let p_avg = Pos3::average([p1, p2].into_iter());
            let opacity = (screen_dist(p_avg).clamp(-0.2, 1.0) + 0.2) / 1.2;
            let mut stroke = stroke;
            stroke.color = stroke.color.gamma_multiply(opacity);

            painter.add(egui::Shape::LineSegment { points, stroke });
        });
    }

    pub fn draw_edges_and_update_visibility(
        &self,
        to_screen: &geo::ToScreen,
        painter: &egui::Painter,
        stroke: egui::Stroke,
        visible: &mut [bool],
    ) {
        let visible_needs_update = match &self.shape {
            Shape::Octahedron
            | Shape::Tetrahedron
            | Shape::Icosahedron
            | Shape::DividedIcosahedron(_)
            | Shape::RegularPolygon2D(_) => {
                self.draw_visible_hull_edges(to_screen, painter, stroke, visible, true);
                false
            },
            Shape::Cube | Shape::FabianHamann | Shape::Football | Shape::Dodecahedron => {
                self.draw_visible_hull_edges(to_screen, painter, stroke, visible, false);
                false
            },
            Shape::TriangTorus | Shape::TriangGrid => {
                self.draw_torus_edges(to_screen, painter, stroke, true);
                true
            },
            Shape::SquareTorus | Shape::SquareGrid => {
                self.draw_torus_edges(to_screen, painter, stroke, false);
                true
            },
            Shape::Random2D(_) => {
                self.draw_all_edges(to_screen, painter, stroke);
                true
            },
            Shape::Custom(_) => {
                self.draw_camera_facing_edges(to_screen, painter, stroke, visible);
                false
            },
        };

        if visible_needs_update {
            debug_assert_eq!(visible.len(), self.nr_vertices());
            for (vis, &pos) in izip!(visible, self.positions()) {
                *vis = to_screen.pos_visible(pos);
            }
        }
    }

    pub fn find_global_minimum(&self, mut f: impl FnMut(usize, Pos3) -> f32) -> (usize, f32) {
        let pot = |v| f(v, self.vertices[v]);
        self.edges.find_global_minimum(pot)
    }

    pub fn from_2d(planar: Embedding2D, shape: Shape) -> Self {
        let (positions_2d, edges) = planar.into_parts();
        const Z: f32 = Z_OFFSET_2D;
        let vertices = positions_2d.iter().map(|p| pos3(p.x, p.y, Z)).collect_vec();
        let sym_group = SymGroup::None(NoSymmetry::new(vertices.len()));
        Self {
            surface: ConvexHull::empty(),
            shape,
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices,
            edges,
            sym_group,
        }
    }

    pub fn new_map_from(shape: Shape, res: usize) -> Self {
        use Shape::*;
        match shape {
            Icosahedron => Self::new_subdivided_icosahedron(res),
            Octahedron => Self::new_subdivided_octahedron(res),
            Tetrahedron => Self::new_subdivided_tetrahedron(res),
            DividedIcosahedron(pressure) => {
                let res1 = usize::min(res, pressure as usize);
                let res2 = if res1 == 0 {
                    res
                } else {
                    (usize::max(res, 1) - 1) / (res1 + 1)
                };
                Self::new_subdivided_subdivided_icosahedron(res1, res2, shape)
            },
            RegularPolygon2D(nr_sides) => {
                let sides = nr_sides as usize;
                Self::new_2d_triangulated_regular_polygon(sides, res)
            },
            Cube => Self::new_subdivided_cube(res),
            Dodecahedron => Self::new_subdivided_dodecahedron(res, false, false),
            Football => Self::new_subdivided_football(res, false),
            FabianHamann => Self::new_subdivided_football(res, true),
            Random2D(seed) => Self::from_2d(super::random_triangulated(res, 8, seed), shape),
            TriangTorus => Self::new_subdivided_triangle_grid(res as isize, true),
            SquareTorus => Self::new_subdivided_squares_grid(res as isize, true),
            TriangGrid => Self::new_subdivided_triangle_grid(res as isize, false),
            SquareGrid => Self::new_subdivided_squares_grid(res as isize, false),
            Custom(c) => {
                let mut result = Self::new_map_from(c.basis.clone(), res);
                for step in &c.build_steps {
                    match step {
                        shape::BuildStep::NeighNeihs(n) => {
                            result.edge_pow(*n);
                        },
                        shape::BuildStep::SubdivEdges(n) => {
                            result.subdivide_all_edges(*n, false);
                        },
                    }
                }
                result.shape = Custom(c);
                result
            },
        }
    }

    /// returns all faces of [`Self::surface`], which are touched by `v`.
    /// (this means mostly one face, except two when on an edge and more if vertex is vertex of surface.)
    #[allow(dead_code)]
    pub fn faces_of(&self, v: usize) -> SmallVec<[usize; 5]> {
        let v_pos = self.vertices[v];
        let (closest_face, _) = self.surface.dual_edges.find_local_minimum(
            |face_i| {
                let face_mid = Pos3::average(
                    self.surface.faces[face_i].iter().map(|&v| self.surface.vertices[v]),
                );
                //assumes convexity (like so many other places)
                (face_mid - v_pos).length_sq()
            },
            0,
        );
        if self.inner_vertices[closest_face].contains(&v) {
            return smallvec![closest_face];
        }
        let face = &self.surface.faces[closest_face];
        if face.contains(&v) {
            // v is vertex of surface -> is corner of multiple faces
            let mut res = smallvec![closest_face];
            let mut last_face = usize::MAX;
            'next_face: loop {
                let curr_face = *res.last().unwrap();
                for next_face in self.surface.dual_edges.neighbors_of(curr_face) {
                    if next_face == last_face {
                        continue;
                    }
                    if next_face == closest_face {
                        break 'next_face;
                    }
                    if self.surface.faces[next_face].contains(&v) {
                        res.push(next_face);
                        last_face = curr_face;
                        continue 'next_face;
                    }
                }
                break;
            }
            return res;
        }
        for (&v1, &v2) in face.iter().circular_tuple_windows() {
            let edge_index = self.surface.edges.directed_index(v1, v2);
            let edge = self.edge_dividing_vertices[edge_index];
            if edge.clone().contains(&v) {
                let neigh_face = self
                    .surface
                    .dual_edges
                    .neighbors_of(closest_face)
                    .find(|&neigh_face| {
                        let neigh_face = &self.surface.faces[neigh_face];
                        neigh_face.contains(&v1) && neigh_face.contains(&v2)
                    })
                    .unwrap();
                return smallvec![closest_face, neigh_face];
            }
        }
        panic!()
    }
}

impl Default for Embedding3D {
    fn default() -> Self {
        Self::new_subdivided_icosahedron(2)
    }
}

fn find_rest_of_circle(
    edges: &EdgeList,
    path: &mut Vec<usize>,
    circles: &mut Vec<Vec<usize>>,
    nr_left: usize,
) {
    debug_assert!(!path.is_empty());
    if nr_left == 0 {
        let first = *path.first().unwrap();
        let last = path.last().unwrap();
        if edges.neighbors_of(first).contains(last) {
            circles.push(path.clone());
        }
        return;
    }

    let curr = *path.last().unwrap();
    let last = if path.len() > 1 {
        path[path.len() - 2]
    } else {
        usize::MAX
    };
    for n in edges.neighbors_of(curr) {
        if n != last {
            path.push(n);
            find_rest_of_circle(edges, path, circles, nr_left - 1);
            path.pop();
        }
    }
}

fn find_all_circles_at(edges: &EdgeList, start: usize, circumference: usize) -> Vec<Vec<usize>> {
    if circumference < 3 {
        return Vec::new();
    }
    let mut res = Vec::new();
    for n in edges.neighbors_of(start) {
        let mut path = vec![start, n];
        let nr_left = circumference - path.len();
        find_rest_of_circle(edges, &mut path, &mut res, nr_left);
    }
    res
}

fn find_all_circles(edges: &EdgeList, circumference: usize) -> BTreeSet<Vec<usize>> {
    let mut res = BTreeSet::new();
    for v in 0..edges.nr_vertices() {
        let new_circles = find_all_circles_at(edges, v, circumference);
        for mut circle in new_circles {
            circle.sort();
            res.insert(circle);
        }
    }
    res
}
