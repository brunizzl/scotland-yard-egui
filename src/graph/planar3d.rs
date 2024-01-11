
use std::iter;
use std::collections::BTreeSet;

use egui::*;
use itertools::{izip, Itertools};

use crate::geo::{Pos3, Vec3, pos3, self, Matrix3x3};

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

fn is_small(x: f32) -> bool {
    x.abs() < 1e-4
}

/// connects the closest vertices to have edges,
/// assumes all connected vertices to have same respective distances.
fn edges_from_uniform_positions(vertex_positions: &[Pos3]) -> EdgeList {
    debug_assert!(vertex_positions.len() >= 4); //can't make a hull of 3 points in 3d

    let neighbor_vertex_dist = {
        let v1 = vertex_positions[0];
        vertex_positions[1..].iter().fold(f32::MAX, 
            |acc, &v| f32::min((v - v1).length(), acc))
    };
    let related = vertex_positions.iter().map(|&p1| 
        vertex_positions.iter().enumerate().filter_map(
            move |(i, &p2)| (is_small((p1-p2).length()-neighbor_vertex_dist).then_some(i)
        )
    ));
    EdgeList::from_iter(related, 6)
}

fn normalize_positions(positions: &mut [Pos3]) {
    //makes only sense if graph is centered around origin
    debug_assert!(Pos3::average_ref(positions.iter()).to_vec3().length() < 1e-4);
    for p in positions {
        *p = p.to_vec3().normalized().to_pos3();
    }
}

impl ConvexTriangleHull {
    #[allow(dead_code)]
    pub fn nr_vertices(&self) -> usize {
        self.vertices.len()
    }

    pub fn empty() -> Self {
        Self {
            vertices: Vec::new(),
            face_normals: Vec::new(),
            edges: EdgeList::empty(),
            triangles: Vec::new(),
        }
    }

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
            let direction_check_vec = Vec3::cross(v1_pos.to_vec3(), v2_pos.to_vec3());

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

    /// connects the closest vertices to have edges,
    /// positions are assumed to lie centered around the origin
    /// assumes all edges to have same length.
    fn new_uniform_from_positions(vertex_positions: Vec<Pos3>) -> Self 
    {
        let mut vertex_neighbors = edges_from_uniform_positions(&vertex_positions);
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

    pub fn new_tetrahedron() -> Self {
        let a = 1.0;
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let mut vs = vec![
            pos3(a, 0.0, -s),
            pos3(-a, 0.0, -s),
            pos3(0.0, a, s),
            pos3(0.0, -a, s),
        ];
        normalize_positions(&mut vs);
        Self::new_uniform_from_positions(vs)
    }

    pub fn new_octahedron() -> Self {
        let mut vs = vec![
            pos3(1.0, 0.0, 0.0),
            pos3(-1.0, 0.0, 0.0),
            pos3(0.0, 1.0, 0.0),
            pos3(0.0, -1.0, 0.0),
            pos3(0.0, 0.0, 1.0),
            pos3(0.0, 0.0, -1.0),
        ];
        normalize_positions(&mut vs);
        Self::new_uniform_from_positions(vs)
    }

    pub fn new_icosahedron() -> Self {        
        let a = 1.0;
        let c = (1.0 + f32::sqrt(5.0)) / 2.0;
        let mut vs = vec![
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
        normalize_positions(&mut vs);
        Self::new_uniform_from_positions(vs)
    }

    pub fn new_subdivided_icosahedron(divisions: usize) -> Self {
        let mut iso = Self::new_icosahedron();
        normalize_positions(&mut iso.vertices);
        Self::subdivide_hull(iso, divisions)
    }

    #[allow(dead_code)]
    pub fn new_subdivided_octahedron(divisions: usize) -> Self {
        let mut oct = Self::new_octahedron();
        normalize_positions(&mut oct.vertices);
        Self::subdivide_hull(oct, divisions)
    }

    #[allow(dead_code)]
    pub fn new_subdivided_tetrahedron(divisions: usize) -> Self {
        let mut tet = Self::new_tetrahedron();
        normalize_positions(&mut tet.vertices);
        Self::subdivide_hull(tet, divisions)
    }

    /// each triangle of the original shape is subdivided in smaller triangles, where divisions
    /// denotes the number of vertices added per original boundary
    fn subdivide_hull(plat: Self, divisions: usize) -> Self {
        let embedding = Embedding3D::subdivide_platonic_with_triangles(plat, divisions);
        let (mut vertices, edges) = (embedding.vertices, embedding.edges);

        //bring all vertices to sphere surface
        normalize_positions(&mut vertices);

        let (triangles, face_normals) = 
            Self::discover_faces(&vertices, &edges);

        Self { 
            vertices, 
            edges, 
            triangles, 
            face_normals 
        }
    }
}

#[derive(Clone, Copy)]
pub struct BidirectionalRange {
    start: usize, //first element included
    end: usize, //first element not included
    step: isize,
}

impl BidirectionalRange {
    pub fn empty() -> Self {
        Self { start: 0, end: 0, step: 0 }        
    }

    fn add(a: usize, b: isize) -> usize {
        ((a as isize) + b) as usize
    }

    pub fn new_forward(start: usize, end: usize) -> Self {
        let end = if end < start { start } else { end };
        Self { start, end, step: 1 }
    }

    #[allow(dead_code)]
    pub fn new_backward(start: usize, end: usize) -> Self {
        let end = if end > start { start } else { end };
        Self { start, end, step: -1 }
    }

    pub fn reversed(&self) -> Self {
        let step = -1 * self.step;
        let start = Self::add(self.end, step);
        let end = Self::add(self.start, step);
        Self { start, end, step }
    }
}

impl Iterator for BidirectionalRange {
    type Item = usize;
    fn next(&mut self) -> Option<Self::Item> {
        if self.start == self.end {
            None
        }
        else {
            let res = self.start;
            self.start = Self::add(self.start, self.step);
            Some(res)
        }
    }
}

/// in symmetric graphs, each vertex can be put into an equivalence class. 
#[derive(Clone)]
pub struct EquivalenceClass {
    /// one entry per vertex, stores to which class vertex belongs
    class: Vec<u16>,
    /// one entry per class, stores which vertex is taken to represent it's whole class (this is arbitrary)
    representative: Vec<usize>,
    /// one entry per vertex, all positions normalized
    vertex_directions: Vec<Vec3>,
}

impl EquivalenceClass {
    pub fn new_for_subdivided_platonic(graph: &Embedding3D, divisions: usize) -> Option<Self> {
        //indicator that passed in graph actually is a subdivision of a platonic (just nessecairy cond, not sufficient though)
        if graph.surface.edges.max_degree() != graph.surface.edges.min_degree() {
            return None;
        }

        let mut equivalence = Self {
            class: vec![u16::MAX; graph.vertices.len()], 
            representative: Vec::new(),
            vertex_directions: Vec::from_iter(graph.positions().iter().map(|p| p.to_vec3().normalized())),
        };
        //the vertices of a platonic solid are all equivalent:
        //the graph looks the same from each position.
        equivalence.representative.push(0);
        for v in 0..graph.surface.nr_vertices() {
            equivalence.class[v] = 0;
        }
        //all edges of a platonic solid look the same (as all vertices look the same, duh),
        //however one can differentiate in a subdivided edge how far from the middle one is.
        //-> inner vertices of a subdivided edge will have this as their class distinction.
        {
            //divisions == 1 => one inner vertex => one class
            //divisions == 2 => two inner vertices => one class
            //divisions == 3 => three inner vertices => two classes
            //...
            let nr_edge_classes = (divisions + 1) / 2;
            for (v, class) in izip!(graph.edge_dividing_vertices[0], 1..=nr_edge_classes) {
                debug_assert_eq!(class, equivalence.representative.len());
                equivalence.representative.push(v);
            }
            for &(mut edge) in &graph.edge_dividing_vertices {
                for class in 1..=nr_edge_classes {
                    let v = edge.next().unwrap();
                    debug_assert!(equivalence.class[v] == u16::MAX || class == nr_edge_classes);
                    equivalence.class[v] = class as u16;
                }
            }
            debug_assert!({
                for &edge in &graph.edge_dividing_vertices {
                    for v in edge {
                        debug_assert_ne!(equivalence.class[v], u16::MAX);
                    }
                }
                true
            });
        }
        //vertices in a given face belong to a different class depending on how far away 
        //from a vertex subdividing an original edge and how far away from an original vertex they are.
        {
            let mut distance_to_og_vertices = vec![isize::MAX; graph.vertices.len()];
            let mut distance_to_og_edges = vec![isize::MAX; graph.vertices.len()];
            let mut queue = std::collections::VecDeque::new();
            for v in 0..graph.surface.nr_vertices() {
                distance_to_og_vertices[v] = 0;
                distance_to_og_edges[v] = 0;
                queue.push_back(v);
            }
            graph.edges.calc_distances_to(&mut queue, &mut distance_to_og_vertices);

            debug_assert!(queue.is_empty());
            for &edge in &graph.edge_dividing_vertices {
                for v in edge {
                    distance_to_og_edges[v] = 0;
                    queue.push_back(v);
                }
            }
            graph.edges.calc_distances_to(&mut queue, &mut distance_to_og_edges);

            let mut class_map = std::collections::HashMap::new();
            for v in 0..graph.vertices.len() {
                if equivalence.class[v] == u16::MAX {
                    let dists = (distance_to_og_edges[v], distance_to_og_vertices[v]);
                    if let Some(&class) = class_map.get(&dists) {
                        equivalence.class[v] = class;
                    }
                    else {
                        let new_class = equivalence.representative.len() as u16;
                        equivalence.representative.push(v);
                        class_map.insert(dists, new_class);
                        equivalence.class[v] = new_class;
                    }
                }
            }            
        }
        Some(equivalence)
    }

    pub fn classes(&self) -> &[u16] {
        &self.class
    } 

    /// if vertex is representative of it's class returns None, else
    /// the rotation (+ perhaps reflection) mapping vertex to it's representative
    pub fn transform_to_representative(&self, edges: &EdgeList, vertex: usize) -> Matrix3x3 {
        debug_assert_eq!(edges.nr_vertices(), self.vertex_directions.len());
        let class = self.class[vertex];
        let repr = self.representative[class as usize];
        if repr == vertex {
            return Matrix3x3::IDENTITY;
        }
        let pre_dir = self.vertex_directions[vertex];
        let post_dir = self.vertex_directions[repr];
        debug_assert!(pre_dir.is_normalized());
        debug_assert!(post_dir.is_normalized());
        let angle = Vec3::angle_between(pre_dir, post_dir);
        let axis = Vec3::cross(pre_dir, post_dir).normalized();
        let rot1 = Matrix3x3::new_rotation_from_axis_angle(axis, angle);
        {
            let det = rot1.determinant();
            debug_assert!((det - 1.0).abs() < 1e-4);
        }

        //TODO: make this more general. (currently only works for trianglulations)
        let n1 = edges.neighbors_of(vertex).next().unwrap();
        let class_n1 = self.class[n1];
        let repr_n1 = edges.neighbors_of(repr)
            .find(|&rn| self.class[rn] == class_n1)
            .unwrap();

        let n2 = edges.neighbors_of(vertex)
            .find(|&n| edges.neighbors_of(n1).contains(&n))
            .unwrap();
        let class_n2 = self.class[n2];
        let repr_n2 = edges.neighbors_of(repr)
            .find(|&rn| rn != repr_n1 && self.class[rn] == class_n2 && edges.neighbors_of(repr_n1).contains(&rn))
            .unwrap();

        let rot1_n1_dir = &rot1 * self.vertex_directions[n1];
        let post_n1_dir = self.vertex_directions[repr_n1];
        debug_assert!(post_n1_dir.is_normalized());
        debug_assert!(rot1_n1_dir.is_normalized());
        let angle_n1 = Vec3::angle_between(rot1_n1_dir, post_n1_dir);
        let axis_n1 = Vec3::cross(rot1_n1_dir, post_n1_dir).normalized();
        let rot2 = Matrix3x3::new_rotation_from_axis_angle(axis_n1, angle_n1);
        {
            let det = rot2.determinant();
            debug_assert!((det - 1.0).abs() < 1e-4);
        }

        let rot21 = &rot2 * &rot1;
        let rot21_n2_dir = &rot21 * self.vertex_directions[n2];
        let post_n2_dir = self.vertex_directions[repr_n2];
        if (rot21_n2_dir - post_n2_dir).length_sq() < 1e-4 {
            rot21
        }
        else {
            let normal = Vec3::cross(post_dir, post_dir - post_n1_dir).normalized();
            let flip = Matrix3x3::new_reflector(normal);
            {
                let det = flip.determinant();
                debug_assert!((det + 1.0).abs() < 1e-4);
            }
            let res = &flip * &rot21;
            debug_assert!((&res * self.vertex_directions[n2] - post_n2_dir).length_sq() < 1e-4);
            res
        }
    }

    pub fn apply_transform(&self, edges: &EdgeList, rot: &Matrix3x3, vertex: usize) -> usize {
        let pre_dir = self.vertex_directions[vertex];
        let post_dir = rot * pre_dir;
        let (res, error) = edges.find_local_minimum(|v| -self.vertex_directions[v].dot(post_dir), vertex);
        debug_assert!(error < 1e-4);
        res
    }
}

pub struct Embedding3D {
    /// all vertices are expected to lie on this surface.
    surface: ConvexTriangleHull,
    /// some surface vertices may only be there for technical reasons. these will not be drawn.
    nr_visible_surface_vertices: usize,
    /// allows faster rendering of triangles in interior of face
    is_regular_triangulation: bool, 

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

    equivalence: Option<EquivalenceClass>,
}

impl Embedding3D {

    pub fn is_3d(&self) -> bool {
        self.surface.vertices.len() > 0
    }

    pub fn edges(&self) -> &EdgeList {
        &self.edges
    }

    pub fn nr_vertices(&self) -> usize {
        self.vertices.len()
    }

    pub fn positions(&self) -> &[Pos3] {
        &self.vertices
    }

    pub fn equivalence(&self) -> Option<&EquivalenceClass> {
        self.equivalence.as_ref()
    }

    #[inline(always)]
    fn add_vertex(&mut self, pos: Pos3) -> usize {
        let new_index = self.nr_vertices();
        self.vertices.push(pos);
        self.edges.push();
        new_index
    }

    fn subdivide_platonic_with_triangles(surface: ConvexTriangleHull, divisions: usize) -> Self {
        let mut vertices = surface.vertices.clone();
        let mut edges = EdgeList::new(6, vertices.len());
        let mut edge_dividing_vertices = vec![BidirectionalRange::empty(); surface.edges.used_space()];
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
                let inner = BidirectionalRange::new_forward(fst_inner_edge_vertex, vertices.len());
                let fw_edge = surface.edges.directed_index(v1, v2);
                let bw_edge = surface.edges.directed_index(v2, v1);
                edge_dividing_vertices[fw_edge] = inner;
                edge_dividing_vertices[bw_edge] = inner.reversed();
                edges.add_path_edges(iter::once(v1).chain(inner).chain(iter::once(v2)));
            }
        }
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

        let nr_visible_surface_vertices = surface.nr_vertices();
        let mut res = Self { 
            surface,
            nr_visible_surface_vertices,
            is_regular_triangulation: true,
            edge_dividing_vertices,
            inner_vertices,
            vertices, 
            edges,
            equivalence: None,
        };
        res.equivalence = EquivalenceClass::new_for_subdivided_platonic(&res, divisions);
        res
    }

    pub fn new_subdivided_icosahedron(divisions: usize) -> Self {
        let ico = ConvexTriangleHull::new_icosahedron();
        Self::subdivide_platonic_with_triangles(ico, divisions)
    }

    pub fn new_subdivided_subdivided_icosahedron(d1: usize, d2: usize) -> Self {
        let ico = ConvexTriangleHull::new_subdivided_icosahedron(d1);
        Self::subdivide_platonic_with_triangles(ico, d2)
    }

    pub fn new_subdivided_tetrahedron(divisions: usize) -> Self {
        let tet = ConvexTriangleHull::new_tetrahedron();
        Self::subdivide_platonic_with_triangles(tet, divisions)
    }

    pub fn new_subdivided_octahedron(divisions: usize) -> Self {
        let oct = ConvexTriangleHull::new_octahedron();
        Self::subdivide_platonic_with_triangles(oct, divisions)
    }

    /// face_info stores circumference in .0 and weather to make the inner vertices of given face type visible in .1
    fn embed_archimedian_solid(mut vertices: Vec<Pos3>, mut edges: EdgeList, face_info: &[(usize, bool)]) -> (Self, Vec<Vec<usize>>) {
        //vertices should be centered around origin (among other things...)
        debug_assert!(Pos3::average_ref(vertices.iter()).to_vec3().length() < 1e-4);
        //all visible face inner thingies must be at the start, because of how the data structure works
        debug_assert!(face_info.iter().tuple_windows().all(|((_, a), (_, b))| a >= b));

        let mut faces = Vec::new();
        let mut hull = ConvexTriangleHull {
            vertices: vertices.clone(),
            edges: edges.clone(),
            face_normals: Vec::new(),
            triangles: Vec::new(),
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
            }
            else {
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
        
        let edge_dividing_vertices = vec![BidirectionalRange::empty(); hull.edges.used_space()];
        let inner_vertices = vec![0..0; hull.face_normals.len()];
        let res = Self {
            surface: hull,
            nr_visible_surface_vertices: vertices.len(),
            is_regular_triangulation: false,
            edge_dividing_vertices,
            inner_vertices,
            vertices,
            edges,
            equivalence: None,
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
            pos3(2.0,  (2.0 * phi + 1.0), phi),
            pos3(2.0,  (2.0 * phi + 1.0), -phi),
            pos3(2.0,  -(2.0 * phi + 1.0), phi),
            pos3(2.0,  -(2.0 * phi + 1.0), -phi),
            pos3(-2.0,  (2.0 * phi + 1.0), phi),
            pos3(-2.0,  (2.0 * phi + 1.0), -phi),
            pos3(-2.0,  -(2.0 * phi + 1.0), phi),
            pos3(-2.0,  -(2.0 * phi + 1.0), -phi),
        ];
        normalize_positions(&mut vertices);
        vertices
    }

    /// turns each edge into a path of length `nr_subdivisions + 1`, 
    /// e.g. `graph.subdivide_all_edges(0)` does nothing
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

    pub fn new_subdivided_dodecahedron(divisions: usize, show_divisions: bool, triangulate: bool) -> Self { 
        let vertices = Self::dodecahedron_vertices();
        let edges = edges_from_uniform_positions(&vertices);
        let face_info = [(5, show_divisions || triangulate)];

        let (mut res, _faces) = Self::embed_archimedian_solid(vertices, edges, &face_info);
        if triangulate {
            normalize_positions(&mut res.surface.vertices);
            res = Self::subdivide_platonic_with_triangles(res.surface, divisions);
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

    /// draws all visible edges, updates visible while doing it
    pub fn draw_visible_edges_3d(&self, to_screen: &geo::ToScreen, painter: &Painter, 
        stroke: Stroke, visible: &mut [bool]) 
    {
        debug_assert!(self.is_3d());
        debug_assert_eq!(visible.len(), self.nr_vertices());
        visible.iter_mut().for_each(|v| *v = false);

        let draw_line = |vertices: &[Pos3], v1, v2| {
            let edge = [
                to_screen.apply(vertices[v1]), 
                to_screen.apply(vertices[v2])];
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
            let mut mark_visible_at = |v| if show_v(v) { visible[v] = true; };
            mark_visible_at(v1);
            mark_visible_at(v2);
            mark_visible_at(v3);
            inner.clone().fold((), |(), v| visible[v] = true);
            edge_1_2.fold((), |(), v| visible[v] = true);
            edge_1_3.fold((), |(), v| visible[v] = true);
            edge_2_3.fold((), |(), v| visible[v] = true);
        }

        if !self.is_regular_triangulation {
            //TODO: draw interior edges
        }
    }

    pub fn draw_all_edges(&self, to_screen: &geo::ToScreen, painter: &Painter, stroke: Stroke) {
        self.edges.for_each_edge(|v1, v2| {
            let edge = [
                to_screen.apply(self.vertices[v1]), 
                to_screen.apply(self.vertices[v2])];
            let line = Shape::LineSegment { points: edge, stroke };
            painter.add(line);
        });
    }

    pub fn find_local_minimum(&self, mut potential: impl FnMut(usize, Pos3) -> f32, node_hint: usize) -> (usize, f32) {
        let pot = |v| potential(v, self.vertices[v]);
        self.edges.find_local_minimum(pot, node_hint)
    } 

    pub fn empty() -> Self {
        Self { 
            surface: ConvexTriangleHull::empty(), 
            nr_visible_surface_vertices: usize::MAX,
            is_regular_triangulation: false,
            edge_dividing_vertices: Vec::new(), 
            inner_vertices: Vec::new(), 
            vertices: Vec::new(), 
            edges: EdgeList::empty(),
            equivalence: None,
        }
    }

    pub fn from_2d(planar: Embedding2D) -> Self {
        let z = 0.5;
        let (positions_2d, edges) = planar.to_parts();
        let vertices = positions_2d.iter().map(|p| pos3(p.x, p.y, z)).collect_vec();
        Self { 
            surface: ConvexTriangleHull::empty(), 
            nr_visible_surface_vertices: usize::MAX,
            is_regular_triangulation: false,
            edge_dividing_vertices: Vec::new(), 
            inner_vertices: Vec::new(), 
            vertices, 
            edges,
            equivalence: None,
        }
    }
}






fn find_next_in_circle(edges: &EdgeList, path: &mut Vec<usize>, nr_left: usize) -> bool {
    debug_assert!(path.len() > 0);
    if nr_left == 0 {
        let first = *path.first().unwrap();
        let last = path.last().unwrap();
        return edges.neighbors_of(first).contains(last);
    }

    let curr = *path.last().unwrap();
    let last = if path.len() > 1 { path[path.len() - 2] } else { usize::MAX };
    for n in edges.neighbors_of(curr) {
        if n != last {
            path.push(n);
            if find_next_in_circle(edges, path, nr_left - 1) {
                return true;
            }
            path.pop();
        }
    }
    false
}

fn find_all_circles_at(edges: &EdgeList, start: usize, circumference: usize) -> Vec<Vec<usize>> {
    if circumference < 3 {
        return Vec::new();
    }
    let mut res = Vec::new();
    for n in edges.neighbors_of(start) {
        let mut path = vec![start, n];
        if find_next_in_circle(edges, &mut path, circumference - 2) {
            res.push(path);
        }
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















