

use itertools::izip;

use crate::geo::*;
use super::*;

/// represents a graph automorphism:
/// 
/// -> iff vertices `u` and `v` share an edge, then vertices `self.forward[u]` and `self.forward[v]` share an edge.
/// 
/// -> for any vertex `v` holds that `v == self.forward[self.backward[v]]`
#[derive(Clone)]
pub struct SymmetryTransform {
    /// forward[v] is vertex where v is mapped to
    forward: Vec<usize>,
    /// backward[v] is vertex which maps to v
    backward: Vec<usize>,
}

impl SymmetryTransform {
    pub fn identity(n: usize) -> Self {
        Self { 
            forward: (0..n).collect_vec(), 
            backward: (0..n).collect_vec(),
        }
    }

    pub fn forward(&self) -> &[usize] {
        &self.forward
    }

    pub fn backward(&self) -> &[usize] {
        &self.backward
    }

    pub fn new(edges: &EdgeList, positions: &[Pos3], matrix: Matrix3x3) -> Self {
        let mut forward = vec![usize::MAX; edges.nr_vertices()];
        let mut backward = vec![usize::MAX; edges.nr_vertices()];

        //idea: find image of vertex 0 initially. 
        //then use already mapped vertex to search image of it's neighbors only in the neighborhood of it's image
        let mut mapped = vec![false; edges.nr_vertices()];
        let mut queue = std::collections::VecDeque::new();
        {
            let mapped_v0_pos = (&matrix * positions[0].to_vec3()).to_pos3();
            let mut best_v = usize::MAX;
            let mut best_dist = f32::MAX;
            for (v, &pos) in izip!(0.., positions) {
                let new_dist = (pos - mapped_v0_pos).length();
                if new_dist < best_dist {
                    best_v = v;
                    best_dist = new_dist;
                }
            }
            debug_assert!(best_dist < 1e-4);
            mapped[0] = true;
            forward[0] = best_v;
            backward[best_v] = 0;
            for n in edges.neighbors_of(0) {
                queue.push_back((best_v, n));
                mapped[n] = true;
            }
        }
        while let Some((im_neigh, pre_v)) = queue.pop_front() {
            debug_assert!(mapped[pre_v]);
            let mapped_v_pos = (&matrix * positions[pre_v].to_vec3()).to_pos3();
            let mut best_im_v = usize::MAX;
            let mut best_dist = f32::MAX;
            for n in edges.neighbors_of(im_neigh) {
                let new_dist = (positions[n] - mapped_v_pos).length();
                if new_dist < best_dist {
                    best_im_v = n;
                    best_dist = new_dist;
                }
            }
            debug_assert!(best_dist < 1e-4);
            debug_assert!(forward[pre_v] == usize::MAX);
            debug_assert!(backward[best_im_v] == usize::MAX);
            forward[pre_v] = best_im_v;
            backward[best_im_v] = pre_v;
            for n in edges.neighbors_of(pre_v) {
                if !mapped[n] {
                    queue.push_back((best_im_v, n));
                    mapped[n] = true;
                }
            }
        }
        debug_assert!(forward.iter().all(|&v| v != usize::MAX));
        debug_assert!(backward.iter().all(|&v| v != usize::MAX));
        
        Self { forward, backward }
    }
}


/// in symmetric graphs, each vertex can be put into an equivalence class. 
#[derive(Clone)]
pub struct EquivalenceClasses {
    /// one entry per vertex, stores to which class vertex belongs
    class: Vec<u16>,

    /// one entry per vertex, logs to which vertex each vertex is mapped to
    vertex_representative: Vec<usize>,

    /// one entry epr class, keeps track which vertex represents a class
    class_representative: Vec<usize>,

    //one entry per element in symmetry group, starting with the identity
    symmetry_transforms: Vec<SymmetryTransform>,

    /// one entry per vertex, indexes in [`self.symmetry_maps`]. 
    /// this map transforms the vertex to it's class representative
    to_representative: Vec<usize>,
}

impl EquivalenceClasses {

    //only works for those platonic solids with threeangles as base shape
    pub fn enumerate_platonic_symmetry_transforms(plat: &ConvexTriangleHull) -> Vec<Matrix3x3> {
        //indicator for platonic, not guarantee
        let degree = plat.edges.max_degree();
        debug_assert_eq!(degree, plat.edges.min_degree());
        debug_assert!(Pos3::average_ref(plat.vertices.iter()).to_vec3().length() < 1e-4);

        let reflect = {
            let fst_tri_normal = plat.face_normals[0];
            let fst_tri_fst_vertex = plat.triangles[0][0];
            let v_dir = plat.vertices[fst_tri_fst_vertex].to_vec3();
            let reflect_normal = Vec3::cross(fst_tri_normal, v_dir).normalized();
            Matrix3x3::new_reflector(reflect_normal)
        };
        let mut rotations = vec![Matrix3x3::IDENTITY, reflect];
        let mut axes = Vec::<Vec3>::new();

        let mut is_new_axis = |mut axis: Vec3| {
            debug_assert!(axis.is_normalized());
            if axis.x < 0.0 || axis.x == 0.0 && (axis.y < 0.0 || axis.y == 0.0 && axis.z < 0.0) {
                axis *= -1.0;
            }
            if let Some(_) = axes.iter().find(|&&a| (a - axis).length() < 1e-4) {
                false
            }
            else {
                axes.push(axis);
                true
            }
        };

        let mut add_new_rotations = |axis: Vec3, angles: &[f32]| {
            if is_new_axis(axis) {
                for &angle in angles {
                    let rot = Matrix3x3::new_rotation_from_axis_angle(axis, angle);
                    rotations.push(&rot * &reflect);
                    rotations.push(rot);
                }
            }
        };

        use std::f32::consts::TAU;
        { //axes through faces
            let angles = [TAU / 3.0, 2.0 * TAU / 3.0];
            for &face_normal in &plat.face_normals {
                add_new_rotations(face_normal, &angles);
            }
        }
        { //axes through edges
            let angles = [TAU * 0.5];
            plat.edges.for_each_edge(|v1, v2| {
                let v1_dir = plat.vertices[v1].to_vec3();
                let v2_dir = plat.vertices[v2].to_vec3();
                let axis = (0.5 * v1_dir + 0.5 * v2_dir).normalized();
                add_new_rotations(axis, &angles);
            });
        }
        { //axes through vertices
            let angles = (1..degree).map(|i| TAU * (i as f32) / (degree as f32)).collect_vec();
            for &pos in &plat.vertices {
                let axis = pos.to_vec3().normalized();
                add_new_rotations(axis, &angles);
            }
        }

        rotations
    }

    fn find_symmetry_transforms(vertex_directions: &[Vec3], transforms: &[Matrix3x3], representative: &[usize]) -> Vec<usize> {
        let mut res = vec![usize::MAX; vertex_directions.len()];
        debug_assert!(transforms.len() > 3);

        for (v, &dir) in izip!(0.., vertex_directions) {
            debug_assert!(dir.is_normalized());
            let repr_dir = vertex_directions[representative[v]];
            let mut best_dist = f32::MAX;
            let mut best_mat = usize::MAX;
            for (m, mat) in izip!(0.., transforms) {
                let im = mat * dir;
                let im_dist = (im - repr_dir).length();
                if im_dist < best_dist {
                    best_dist = im_dist;
                    best_mat = m;
                }
            }
            debug_assert!(best_dist.abs() < 1e-4);
            res[v] = best_mat;
        }
        res
    }

    pub fn new_for_subdivided_platonic(graph: &Embedding3D, divisions: usize) -> Option<Self> {
        //indicator that passed in graph actually is a subdivision of a platonic (just nessecairy cond, not sufficient though)
        let degree = graph.surface.edges.max_degree();
        if degree != graph.surface.edges.min_degree() {
            return None;
        }

        let mut class = vec![u16::MAX; graph.vertices.len()];
        let mut class_representative = Vec::new(); //one entry per class, later used to compute vertex_representative
        
        //the vertices of a platonic solid are all equivalent:
        //the graph looks the same from each position.
        class_representative.push(0);
        for v in 0..graph.surface.nr_vertices() {
            class[v] = 0;
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
                debug_assert_eq!(class, class_representative.len());
                class_representative.push(v);
            }
            //this loop relies on each edge beeing iterated over once in each direction 
            for &(mut edge) in &graph.edge_dividing_vertices {
                for c in 1..=nr_edge_classes {
                    let v = edge.next().unwrap();
                    debug_assert!(class[v] == u16::MAX || c == nr_edge_classes);
                    class[v] = c as u16;
                }
            }
            debug_assert!({
                for &edge in &graph.edge_dividing_vertices {
                    for v in edge {
                        debug_assert_ne!(class[v], u16::MAX);
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
                if class[v] == u16::MAX {
                    let dists = (distance_to_og_edges[v], distance_to_og_vertices[v]);
                    if let Some(&c) = class_map.get(&dists) {
                        class[v] = c;
                    }
                    else {
                        let new_c = class_representative.len() as u16;
                        class_representative.push(v);
                        class_map.insert(dists, new_c);
                        class[v] = new_c;
                    }
                }
            }
        }

        let vertex_directions = Vec::from_iter(graph.positions().iter().map(|p| p.to_vec3().normalized()));
        let vertex_representative = Vec::from_iter(class.iter().map(|&c| class_representative[c as usize]));

        let symmetry_transform_matrices = 'try_reuse: {
            /// for a given platonic solid, this stores nr of vertices, vertex degree, all the transforms and to_representative.
            static KNOWN_PLATONIC_SYMMETRY_TRANSFORMS: 
                std::sync::Mutex<Vec<(usize, usize, Vec<Matrix3x3>)>> 
                = std::sync::Mutex::new(Vec::new());

            let nr_surface_vs = graph.surface.nr_vertices();
            if let Ok(guard) = KNOWN_PLATONIC_SYMMETRY_TRANSFORMS.try_lock() {
                for (nr_vs, deg, transforms) in guard.iter() {
                    if *nr_vs == nr_surface_vs && *deg == degree {
                        break 'try_reuse transforms.clone();
                    }
                }
            }
            let transforms = Self::enumerate_platonic_symmetry_transforms(&graph.surface);

            let new_stored_entry = (nr_surface_vs, degree, transforms.clone());
            if let Ok(mut lock) = KNOWN_PLATONIC_SYMMETRY_TRANSFORMS.lock() {
                lock.push(new_stored_entry);
            }
            transforms
        };
        let to_representative = Self::find_symmetry_transforms(
            &vertex_directions, 
            &symmetry_transform_matrices, 
            &vertex_representative
        );
        let symmetry_transforms = symmetry_transform_matrices.iter().map(|m| 
            SymmetryTransform::new(graph.edges(), graph.positions(), *m)
        ).collect_vec();

        Some(Self {
            class, 
            vertex_representative,
            class_representative,
            symmetry_transforms,
            to_representative,
        })
    }

    pub fn classes(&self) -> &[u16] {
        &self.class
    }

    pub fn class_representatives(&self) -> &[usize] {
        &self.class_representative
    }

    pub fn all_transforms(&self) -> &[SymmetryTransform] {
        &self.symmetry_transforms
    }

    #[allow(dead_code)]
    pub fn nr_classes(&self) -> usize {
        self.class_representative.len()
    }

    fn transform_of(&self, v: usize) -> (&SymmetryTransform, usize) {
        let transform_nr = self.to_representative[v];
        (
            &self.symmetry_transforms[transform_nr], 
            self.vertex_representative[v],
        )
    }

    /// chooses the rotation, such that in the rotated result the cop defining the rotation is moved
    /// to the smallest index.
    pub fn transform_all(&self, cops: &mut [usize]) -> &SymmetryTransform {
        if cops.len() == 0 {
            return &self.symmetry_transforms[0];
        }
        if cops.len() == 1 {
            let (rot, repr) = self.transform_of(cops[0]);
            cops[0] = repr;
            return rot;
        }
        for i in 0..cops.len() {
            let mut rotated = [0usize; 8];
            let rotated = &mut rotated[..cops.len()];
            for j in 0..cops.len() {
                rotated[j] = cops[j];
            }
            rotated.swap(0, i);
            let fst_cop = rotated[0];
            let rest_cops = &mut rotated[1..];
            let (rot, fst_cop_repr) = self.transform_of(fst_cop);
            debug_assert_eq!(fst_cop_repr, rot.forward[fst_cop]);
            for c in rest_cops.iter_mut() {
                *c = rot.forward[*c];
            }
            rest_cops.sort();
            if rest_cops[0] >= fst_cop_repr {
                cops[0] = fst_cop_repr;
                for j in 1..cops.len() {
                    cops[j] = rotated[j];
                }

                return rot;
            }
        }
        panic!("one always finds a cop that defines \
        a rotation and occupies the smallest vertex after rotating.");
    }
}
