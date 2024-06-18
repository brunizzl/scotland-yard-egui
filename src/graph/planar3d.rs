use std::collections::BTreeSet;

use egui::*;
use itertools::{izip, Itertools};
use smallvec::{smallvec, SmallVec};

use crate::geo::{self, pos3, vec3, Pos3, Vec3};

use crate::graph::*;

mod equivalence;
pub use equivalence::*;

mod bidirectional_range;
use bidirectional_range::*;

mod convex_triangle_hull;
use convex_triangle_hull::*;

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
    surface: ConvexTriangleHull,
    /// some surface vertices may only be there for technical reasons. these will not be drawn.
    nr_visible_surface_vertices: usize,
    /// allows faster rendering of triangles in interior of face
    is_regular_triangulation: bool,

    /// 2d rendering (no camera turns allowed)
    is_flat: bool,

    /// only used when `self.is_flat`. edges longer than this are not shown.
    max_shown_edge_length: f32,

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

    pub fn is_flat(&self) -> bool {
        self.is_flat
    }

    pub fn edges(&self) -> &EdgeList {
        &self.edges
    }

    pub fn into_edges(self) -> EdgeList {
        self.edges
    }

    pub fn nr_vertices(&self) -> usize {
        self.vertices.len()
    }

    pub fn positions(&self) -> &[Pos3] {
        &self.vertices
    }

    pub fn sym_group(&self) -> &SymGroup {
        &self.sym_group
    }

    pub fn max_shown_edge_length(&self) -> f32 {
        self.max_shown_edge_length
    }

    #[allow(dead_code)]
    pub fn surface(&self) -> &ConvexTriangleHull {
        &self.surface
    }

    #[inline(always)]
    fn add_vertex(&mut self, pos: Pos3) -> usize {
        let new_index = self.nr_vertices();
        self.vertices.push(pos);
        self.edges.push();
        new_index
    }

    fn subdivide_surface_with_triangles(
        surface: ConvexTriangleHull,
        divisions: usize,
        is_platonic: bool,
        is_flat: bool,
    ) -> Self {
        let mut vertices = surface.vertices.clone();
        let mut edges = EdgeList::new(6, vertices.len());
        let mut edge_dividing_vertices =
            vec![BidirectionalRange::uninitialized(); surface.edges.used_space()];
        let mut inner_vertices = Vec::new();

        let scaled_dir = |p1, p2| (p2 - p1) / ((divisions + 1) as f32);
        surface.edges.for_each_edge(|v2, v1| {
            debug_assert!(v1 < v2); //every edge is only iterated over in one direction
            let fw_edge = surface.edges.directed_index(v1, v2);
            let bw_edge = surface.edges.directed_index(v2, v1);
            debug_assert!(edge_dividing_vertices[fw_edge].is_uninitialized());
            debug_assert!(edge_dividing_vertices[bw_edge].is_uninitialized());
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
            edge_dividing_vertices[fw_edge] = inner;
            edge_dividing_vertices[bw_edge] = inner.reversed();
            use std::iter::once;
            edges.add_path_edges(once(v1).chain(inner).chain(once(v2)));
        });
        assert_eq!(vertices.len(), edges.nr_vertices());

        for &[v1, v2, v3] in &surface.triangles {
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

        let nr_visible_surface_vertices = surface.nr_vertices();
        let sym_group = SymGroup::None(NoSymmetry::new(vertices.len()));

        sort_neigbors(&mut edges, &vertices);
        let mut res = Self {
            surface,
            nr_visible_surface_vertices,
            is_regular_triangulation: true,
            is_flat,
            max_shown_edge_length: 1e10,
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

    pub fn new_subdivided_icosahedron(divisions: usize) -> Self {
        let ico = ConvexTriangleHull::new_icosahedron();
        Self::subdivide_surface_with_triangles(ico, divisions, true, false)
    }

    pub fn new_subdivided_subdivided_icosahedron(d1: usize, d2: usize) -> Self {
        let ico = ConvexTriangleHull::new_subdivided_icosahedron(d1);
        Self::subdivide_surface_with_triangles(ico, d2, false, false)
    }

    pub fn new_subdivided_tetrahedron(divisions: usize) -> Self {
        let tet = ConvexTriangleHull::new_tetrahedron();
        Self::subdivide_surface_with_triangles(tet, divisions, true, false)
    }

    pub fn new_subdivided_octahedron(divisions: usize) -> Self {
        let oct = ConvexTriangleHull::new_octahedron();
        Self::subdivide_surface_with_triangles(oct, divisions, true, false)
    }

    /// face_info stores circumference in .0 and weather to make the inner vertices of given face type visible in .1
    fn embed_archimedian_solid(
        mut vertices: Vec<Pos3>,
        mut edges: EdgeList,
        face_info: &[(usize, bool)],
    ) -> (Self, Vec<Vec<usize>>) {
        //vertices should be centered around origin (among other things...)
        debug_assert!(Pos3::average(vertices.iter().copied()).to_vec3().length() < 1e-4);
        //all visible face inner thingies must be at the start, because of how the data structure works
        debug_assert!(face_info.iter().tuple_windows().all(|((_, a), (_, b))| a >= b));

        let mut faces = Vec::new();
        let mut hull = ConvexTriangleHull {
            vertices: vertices.clone(),
            edges: edges.clone(),
            face_normals: Vec::new(),
            triangles: Vec::new(),
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

        for (faces, &(_, show)) in faces.iter_mut().zip(face_info) {
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

                if let [v1, v2, v3] = face[..] {
                    let [u, v, w] = direct_triangle(&hull.vertices, [v1, v2, v3]);
                    hull.face_normals.push(compute_normal(&hull.vertices, u, v, w));
                    hull.triangles.push([u, v, w]);
                    continue;
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
                let center = Pos3::average(face.iter().map(|&v| hull.vertices[v]));
                let center_v = hull.edges.push();
                debug_assert_eq!(center_v, hull.vertices.len());
                hull.vertices.push(center);
                if show {
                    vertices.push(center);
                    edges.push();
                }
                for (&v1, &v2) in face.iter().circular_tuple_windows() {
                    debug_assert!(edges.neighbors_of(v1).contains(&v2));
                    hull.edges.add_edge(v1, center_v);
                    if show {
                        edges.add_edge(v1, center_v)
                    }
                    let tri = direct_triangle(&hull.vertices, [center_v, v1, v2]);
                    hull.face_normals.push(compute_normal(&hull.vertices, center_v, v1, v2));
                    hull.triangles.push(tri);
                }
            }
        }

        let hull_dual_edges = ConvexTriangleHull::discover_dual(&hull.triangles);
        hull.dual_edges = hull_dual_edges;

        let edge_dividing_vertices =
            vec![BidirectionalRange::uninitialized(); hull.edges.used_space()];
        let inner_vertices = vec![0..0; hull.face_normals.len()];
        let sym_group = SymGroup::None(NoSymmetry::new(vertices.len()));

        sort_neigbors(&mut edges, &vertices);
        let res = Self {
            surface: hull,
            nr_visible_surface_vertices: vertices.len(),
            is_regular_triangulation: false,
            is_flat: false,
            max_shown_edge_length: 1e10,
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
    fn subdivide_all_edges(&mut self, nr_subdivisions: usize) {
        if nr_subdivisions == 0 {
            return;
        }
        let nr_og_vertices = self.nr_vertices();
        assert!(self.surface.nr_vertices() >= self.nr_vertices());
        debug_assert!(self.surface.vertices[..self.nr_vertices()] == self.vertices[..]);

        let mut v1_neighbors = Vec::new();
        for v1 in 0..nr_og_vertices {
            v1_neighbors.clear();
            v1_neighbors.extend(self.edges.neighbors_of(v1));
            for &v2 in &v1_neighbors {
                if v2 < nr_og_vertices {
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

                    let inner_end = curr + 1;
                    for (u, v) in [(v1, v2), (v2, v1)] {
                        let i = self.surface.edges.directed_index(u, v);
                        debug_assert!(self.edge_dividing_vertices[i].count() == 0);
                        self.edge_dividing_vertices[i] =
                            BidirectionalRange::new_forward(fst_inner, inner_end);
                    }
                }
            }
        }
        sort_neigbors(&mut self.edges, &self.vertices);
    }

    /// custom subdivision of graph described in Fabian Hamann's masters thesis.
    pub fn new_subdivided_football(divisions: usize, show_hex_mid: bool) -> Self {
        let vertices = Self::football_vertices();
        let edges = edges_from_uniform_positions(&vertices);
        debug_assert_eq!(vertices.len(), 60);
        debug_assert_eq!(edges.count_entries(), 180);

        let face_info = [(6, show_hex_mid), (5, false)];
        let (mut res, _faces) = Self::embed_archimedian_solid(vertices, edges, &face_info);
        res.subdivide_all_edges(divisions);
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

    pub fn new_subdivided_dodecahedron(
        divisions: usize,
        show_divisions: bool,
        triangulate: bool,
    ) -> Self {
        let vertices = Self::dodecahedron_vertices();
        let edges = edges_from_uniform_positions(&vertices);
        let face_info = [(5, show_divisions || triangulate)];

        let (mut res, _faces) = Self::embed_archimedian_solid(vertices, edges, &face_info);
        if triangulate {
            res.surface.normalize_positions();
            res = Self::subdivide_surface_with_triangles(res.surface, divisions, false, false);
        } else {
            res.subdivide_all_edges(divisions);
        }
        res
    }

    pub fn new_subdivided_cube(divisions: usize) -> Self {
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
        let (mut res, _faces) = Self::embed_archimedian_solid(vertices, edges, &face_info);
        res.subdivide_all_edges(divisions);
        res
    }

    /// rendered flat as two subdivided equilateral triangles.
    /// imagine taking a square and  connecting the left and right sides and the top and bottom sides.
    /// this is topologically a torus.
    /// now slant the shape to the right until the upper left corner sits centered above the bottom side.
    /// connecting the upper left and lower right corners yields the two equilateral triangles.
    pub fn new_subdivided_triangle_torus(len: isize) -> Self {
        assert!(len >= 2);
        let nr_vertices = (len * len) as usize;
        let mut vertices = Vec::with_capacity(nr_vertices);
        let mut edges = EdgeList::new(6, nr_vertices);
        let index_of = |x, y| (x * len + y) as usize;

        let scale = 1.0 / (len as f32 + 1.0);
        let x_step = scale * vec3(1.0, 0.0, 0.0);
        let y_step = scale * vec3(-0.5, f32::sqrt(3.0) / 2.0, 0.0);
        let corner = pos3(-0.25, -f32::sqrt(3.0) / 4.0, Z_OFFSET_2D);
        for x in 0..len {
            for y in 0..len {
                let pos = corner + (x as f32) * x_step + (y as f32) * y_step;
                vertices.push(pos);

                let v = index_of(x, y);
                debug_assert_eq!(v + 1, vertices.len());
                if len > 2 {
                    for (nx, ny) in [
                        (x - 1, y),
                        (x + 1, y),
                        (x, y - 1),
                        (x, y + 1),
                        (x - 1, y - 1),
                        (x + 1, y + 1),
                    ] {
                        let nx = (nx + len) % len;
                        let ny = (ny + len) % len;
                        let nv = index_of(nx, ny);
                        if nv < v {
                            edges.add_edge(v, nv);
                        }
                    }
                }
            }
        }
        if len == 2 {
            for v1 in 0..4 {
                for v2 in (v1 + 1)..4 {
                    edges.add_edge(v1, v2);
                }
            }
        }
        let sym = torus::TorusSymmetry::new(nr_vertices);

        sort_neigbors(&mut edges, &vertices);
        Self {
            surface: ConvexTriangleHull::empty(),
            nr_visible_surface_vertices: usize::MAX,
            is_regular_triangulation: false,
            is_flat: true,
            max_shown_edge_length: if len > 0 { 0.55 } else { 1.1 },
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices,
            edges,
            sym_group: SymGroup::Torus6(sym),
        }
    }

    /// rendered flat as one large subdivided square.
    /// imagine taking a square and  connecting the left and right sides and the top and bottom sides.
    /// this is topologically a torus.
    #[allow(dead_code)]
    pub fn new_subdivided_squares_torus(len: isize) -> Self {
        assert!(len >= 2);
        let nr_vertices = (len * len) as usize;
        let mut vertices = Vec::with_capacity(nr_vertices);
        let mut edges = EdgeList::new(6, nr_vertices);
        let index_of = |x, y| (x * len + y) as usize;

        let scale = 1.0 / (len as f32 + 1.0);
        let x_step = scale * vec3(1.0, 0.0, 0.0);
        let y_step = scale * vec3(0.0, 1.0, 0.0);
        let corner = pos3(-0.5, -0.5, Z_OFFSET_2D);
        for x in 0..len {
            for y in 0..len {
                let pos = corner + (x as f32) * x_step + (y as f32) * y_step;
                vertices.push(pos);

                let v = index_of(x, y);
                debug_assert_eq!(v + 1, vertices.len());
                if len > 2 {
                    for (nx, ny) in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)] {
                        let nx = (nx + len) % len;
                        let ny = (ny + len) % len;
                        let nv = index_of(nx, ny);
                        if nv < v {
                            edges.add_edge(v, nv);
                        }
                    }
                }
            }
        }
        if len == 2 {
            debug_assert_eq!(edges.nr_vertices(), 4);
            edges.add_edge(0, 1);
            edges.add_edge(1, 3);
            edges.add_edge(3, 2);
            edges.add_edge(2, 0);
        }
        let sym = torus::TorusSymmetry::new(nr_vertices);

        sort_neigbors(&mut edges, &vertices);
        Self {
            surface: ConvexTriangleHull::empty(),
            nr_visible_surface_vertices: usize::MAX,
            is_regular_triangulation: false,
            is_flat: true,
            max_shown_edge_length: if len > 0 { 0.55 } else { 1.9 },
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices,
            edges,
            sym_group: SymGroup::Torus4(sym),
        }
    }

    pub fn new_2d_triangulated_regular_polygon(sides: usize, divisions: usize) -> Self {
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

        let surface = ConvexTriangleHull::new_from_graph(surface_positions, surface_edges);
        let mut res = Self::subdivide_surface_with_triangles(surface, divisions, false, true);
        sort_neigbors(&mut res.edges, &res.vertices);
        res
    }

    /// draws all visible edges, updates visible while doing it
    fn draw_visible_edges_3d(
        &self,
        to_screen: &geo::ToScreen,
        painter: &Painter,
        stroke: Stroke,
        visible: &mut [bool],
    ) {
        debug_assert!(self.uses_surface());
        debug_assert_eq!(visible.len(), self.nr_vertices());
        visible.iter_mut().for_each(|v| *v = false);

        let draw_line = |vertices: &[Pos3], v1, v2| {
            let edge = [to_screen.apply(vertices[v1]), to_screen.apply(vertices[v2])];
            let line = Shape::LineSegment { points: edge, stroke };
            painter.add(line);
        };
        let iter = itertools::izip!(
            self.surface.face_normals.iter(),
            self.surface.triangles.iter(),
            self.inner_vertices.iter()
        );
        for (&normal, &[v1, v2, v3], inner) in iter {
            if !to_screen.faces_camera(normal) {
                continue;
            }
            let p1 = self.surface.vertices[v1];
            let p2 = self.surface.vertices[v2];
            let p3 = self.surface.vertices[v3];
            if !to_screen.triangle_visible(p1, p2, p3) {
                continue;
            }

            //draw visible edges of self.surface
            let show_v = |v| v < self.nr_visible_surface_vertices;
            let show_edge = |u, v| show_v(u) && show_v(v);
            for (u, v) in [v1, v2, v3].into_iter().circular_tuple_windows() {
                if show_edge(u, v) {
                    draw_line(&self.surface.vertices, u, v);
                }
            }

            let edge_1_2_index = self.surface.edges.directed_index(v1, v2);
            let edge_1_3_index = self.surface.edges.directed_index(v1, v3);
            let edge_2_3_index = self.surface.edges.directed_index(v2, v3);
            let edge_1_2 = self.edge_dividing_vertices[edge_1_2_index];
            let edge_1_3 = self.edge_dividing_vertices[edge_1_3_index];
            let edge_2_3 = self.edge_dividing_vertices[edge_2_3_index];
            if self.is_regular_triangulation {
                //draw inner edges
                //this dosn't draw each actual tiny edge, but instead all edges lying in
                //  one line at once.
                //note: updating the vertex visibility below results in this function overall
                //  still having the same O complexity, but combining edges this way yields
                //  smaller screenshots.
                for (v1, v2) in edge_1_2.zip(edge_1_3) {
                    draw_line(&self.vertices, v1, v2);
                }
                for (v1, v2) in edge_2_3.zip(edge_1_3) {
                    draw_line(&self.vertices, v1, v2);
                }
                for (v1, v2) in edge_1_2.zip(edge_2_3.reversed()) {
                    draw_line(&self.vertices, v1, v2);
                }
            }

            //update vertex visibility
            let mut mark_visible_at = |v| {
                if show_v(v) {
                    visible[v] = true;
                }
            };
            mark_visible_at(v1);
            mark_visible_at(v2);
            mark_visible_at(v3);
            inner.clone().for_each(|v| visible[v] = true);
            edge_1_2.for_each(|v| visible[v] = true);
            edge_1_3.for_each(|v| visible[v] = true);
            edge_2_3.for_each(|v| visible[v] = true);
        }

        if !self.is_regular_triangulation {
            //TODO: draw interior edges
        }
    }

    fn draw_all_short_edges(&self, to_screen: &geo::ToScreen, painter: &Painter, stroke: Stroke) {
        let max_len_squared = self.max_shown_edge_length * self.max_shown_edge_length;
        self.edges.for_each_edge(|v1, v2| {
            let p1 = self.vertices[v1];
            let p2 = self.vertices[v2];
            if (p1 - p2).length_sq() < max_len_squared {
                let points = [to_screen.apply(p1), to_screen.apply(p2)];
                painter.add(Shape::LineSegment { points, stroke });
            }
        });
    }

    pub fn draw_edges_and_update_visibility(
        &self,
        to_screen: &geo::ToScreen,
        painter: &Painter,
        stroke: Stroke,
        visible: &mut [bool],
    ) {
        if self.uses_surface() {
            self.draw_visible_edges_3d(to_screen, painter, stroke, visible);
        } else {
            debug_assert_eq!(visible.len(), self.nr_vertices());
            for (vis, &pos) in izip!(visible, self.positions()) {
                *vis = to_screen.pos_visible(pos);
            }
            self.draw_all_short_edges(to_screen, painter, stroke);
        }
    }

    pub fn find_local_minimum(
        &self,
        mut potential: impl FnMut(usize, Pos3) -> f32,
        node_hint: usize,
    ) -> (usize, f32) {
        let pot = |v| potential(v, self.vertices[v]);
        self.edges.find_local_minimum(pot, node_hint)
    }

    pub fn empty() -> Self {
        Self {
            surface: ConvexTriangleHull::empty(),
            nr_visible_surface_vertices: usize::MAX,
            is_regular_triangulation: false,
            is_flat: false,
            max_shown_edge_length: 1e10,
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices: Vec::new(),
            edges: EdgeList::empty(),
            sym_group: SymGroup::None(NoSymmetry::new(0)),
        }
    }

    pub fn from_2d(planar: Embedding2D) -> Self {
        let (positions_2d, edges) = planar.into_parts();
        const Z: f32 = Z_OFFSET_2D;
        let vertices = positions_2d.iter().map(|p| pos3(p.x, p.y, Z)).collect_vec();
        let sym_group = SymGroup::None(NoSymmetry::new(vertices.len()));
        Self {
            surface: ConvexTriangleHull::empty(),
            nr_visible_surface_vertices: usize::MAX,
            is_regular_triangulation: false,
            is_flat: true,
            max_shown_edge_length: 1e10,
            edge_dividing_vertices: Vec::new(),
            inner_vertices: Vec::new(),
            vertices,
            edges,
            sym_group,
        }
    }
}

impl Embedding3D {
    /// returns all faces of [`Self::surface`], which are touched by `v`.
    /// (this means mostly one face, except two when on an edge and more if vertex is vertex of surface.)
    #[allow(dead_code)]
    pub fn faces_of(&self, v: usize) -> SmallVec<[usize; 5]> {
        assert_eq!(self.surface.nr_vertices(), self.nr_visible_surface_vertices);

        let v_pos = self.vertices[v];
        let (closest_face, _) = self.surface.dual_edges.find_local_minimum(
            |face_i| {
                let face_mid = Pos3::average(
                    self.surface.triangles[face_i].iter().map(|&v| self.surface.vertices[v]),
                );
                //assumes convexity (like so many other places)
                (face_mid - v_pos).length_sq()
            },
            0,
        );
        if self.inner_vertices[closest_face].contains(&v) {
            return smallvec![closest_face];
        }
        let tri = self.surface.triangles[closest_face];
        if tri.contains(&v) {
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
                    if self.surface.triangles[next_face].contains(&v) {
                        res.push(next_face);
                        last_face = curr_face;
                        continue 'next_face;
                    }
                }
                break;
            }
            return res;
        }
        for (&v1, &v2) in tri.iter().circular_tuple_windows() {
            let edge_index = self.surface.edges.directed_index(v1, v2);
            let edge = self.edge_dividing_vertices[edge_index];
            if edge.clone().contains(&v) {
                let neigh_face = self
                    .surface
                    .dual_edges
                    .neighbors_of(closest_face)
                    .find(|&neigh_face| {
                        let neigh_tri = self.surface.triangles[neigh_face];
                        neigh_tri.contains(&v1) && neigh_tri.contains(&v2)
                    })
                    .unwrap();
                return smallvec![closest_face, neigh_face];
            }
        }
        panic!()
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
