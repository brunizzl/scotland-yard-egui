use super::*;

/// triangulation of a closed 2d surface embedded in 3d
pub struct ConvexTriangleHull {
    pub vertices: Vec<Pos3>,

    /// outher unit normal of each face
    /// this entry decides if a face is rendered or not: only camera-facing faces are.
    pub face_normals: Vec<Vec3>,

    /// indices of neighboring vertices
    pub edges: EdgeList,

    /// indices of vertices surrounding face
    pub triangles: Vec<[usize; 3]>,

    /// two faces are connected, iff they share a (primal) edge.
    pub dual_edges: EdgeList,
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
            dual_edges: EdgeList::empty(),
        }
    }

    /// adjusts each vertex position to have length 1,
    /// updates face normals
    pub fn normalize_positions(&mut self) {
        normalize_positions(&mut self.vertices);
        for (normal, [v1, v2, v3]) in izip!(&mut self.face_normals, &self.triangles) {
            let pos = |&v| self.vertices[v];
            *normal = geo::plane_normal(pos(v1), pos(v2), pos(v3));
        }
    }

    fn discover_faces(vertices: &[Pos3], edges: &EdgeList) -> (Vec<[usize; 3]>, Vec<Vec3>) {
        //enumerates each edge twice: once in each direction
        let mut unused_edges = edges.all_valid_edge_indices();

        let mut triangles = Vec::new();
        let mut normals = Vec::new();

        let mut next_search_start = 0;
        while let Some(edge_index) = unused_edges
            .iter()
            .skip(next_search_start)
            .position(|&e| e)
            .map(|p| p + next_search_start)
        {
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
            normals.push(geo::plane_normal(v1_pos, v2_pos, v3_pos));
            triangles.push([v1, v2, v3]);
        }

        //an actual 3d surface is assumed to be closed, e.g. there are no "outher" triangles,
        //meaning that every triangle is also surrounded by 3 other triangles.
        assert!(
            vertices.iter().all(|p| p.z == Z_OFFSET_2D) || !unused_edges.into_iter().any(|e| e)
        );

        (triangles, normals)
    }

    /// with faces indexed as in passed-in `triangles`, this returns the dual graph.
    pub fn discover_dual(triangles: &[[usize; 3]]) -> EdgeList {
        let mut res = EdgeList::new(3, triangles.len());

        fn share_edge(tri1: &[usize; 3], tri2: &[usize; 3]) -> bool {
            tri1.iter().filter(|&v| tri2.contains(v)).count() == 2
        }

        for (i1, tri1) in izip!(0.., triangles) {
            for (i2, tri2) in izip!(i1.., &triangles[i1..]) {
                if share_edge(tri1, tri2) {
                    res.add_edge(i1, i2);
                }
            }
        }
        res
    }

    pub fn new_from_graph(vertex_positions: Vec<Pos3>, vertex_neighbors: EdgeList) -> Self {
        let (triangles, face_normals) = Self::discover_faces(&vertex_positions, &vertex_neighbors);

        let dual_edges = Self::discover_dual(&triangles);

        Self {
            vertices: vertex_positions,
            edges: vertex_neighbors,
            triangles,
            face_normals,
            dual_edges,
        }
    }

    /// connects the closest vertices to have edges,
    /// positions are assumed to lie centered around the origin
    /// assumes all edges to have same length.
    pub fn new_uniform_from_positions(vertex_positions: Vec<Pos3>) -> Self {
        let vertex_neighbors = edges_from_uniform_positions(&vertex_positions);
        Self::new_from_graph(vertex_positions, vertex_neighbors)
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
        let embedding = Embedding3D::subdivide_surface_with_triangles(
            plat,
            divisions,
            false,
            Shape::Random2D(0),
        );
        let (mut vertices, edges) = (embedding.vertices, embedding.edges);

        //bring all vertices to sphere surface
        normalize_positions(&mut vertices);

        let (triangles, face_normals) = Self::discover_faces(&vertices, &edges);

        let dual_edges = Self::discover_dual(&triangles);

        Self {
            vertices,
            edges,
            triangles,
            face_normals,
            dual_edges,
        }
    }

    #[allow(dead_code)]
    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }
}
