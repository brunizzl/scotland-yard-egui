use super::*;

#[derive(Clone, Serialize, Deserialize)]
pub struct ExplicitAutomorphism {
    /// forward[v] is vertex where v is mapped to
    forward: Vec<usize>,
    /// backward[v] is vertex which maps to v
    backward: Vec<usize>,
}

impl Automorphism for ExplicitAutomorphism {
    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        self.forward.iter().copied()
    }

    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        self.backward.iter().copied()
    }

    fn nr_vertices(&self) -> usize {
        debug_assert_eq!(self.forward.len(), self.backward.len());
        self.forward.len()
    }
}

impl<T: Automorphism> From<&T> for ExplicitAutomorphism {
    fn from(auto: &T) -> Self {
        Self {
            forward: auto.forward().collect_vec(),
            backward: auto.backward().collect_vec(),
        }
    }
}

impl ExplicitAutomorphism {
    pub fn new(edges: &EdgeList, positions: &[Pos3], matrix: Matrix3x3) -> Self {
        let mut forward = vec![usize::MAX; edges.nr_vertices()];
        let mut backward = vec![usize::MAX; edges.nr_vertices()];

        //idea: find image of vertex 0 initially.
        //then use already mapped vertex to search image of it's neighbors only in the neighborhood of it's image
        //note: this requires the graph to be connected
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

/// stores every automorphism explicitly
#[derive(Clone, Serialize, Deserialize)]
pub struct ExplicitClasses {
    /// one entry per vertex, logs to which vertex each vertex is mapped to
    vertex_representative: Vec<usize>,

    //one entry per element in symmetry group, starting with the identity
    symmetry_transforms: Vec<ExplicitAutomorphism>,

    /// one row per vertex, indexes in [`self.symmetry_maps`].
    /// all entries in row are indices of transforms mapping vertex to it's representative
    to_representative: BoolCSR,
}

impl ExplicitClasses {
    pub fn enumerate_platonic_symmetry_transforms(plat: &ConvexHull) -> Vec<Matrix3x3> {
        //indicator for platonic, not guarantee
        let degree = plat.edges.max_degree();
        debug_assert_eq!(degree, plat.edges.min_degree());
        debug_assert!(Pos3::average(plat.vertices.iter().copied()).to_vec3().length() < 1e-4);

        let reflect = {
            let fst_tri_normal = plat.face_normals[0];
            let fst_tri_fst_vertex = plat.faces[0][0];
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
            if axes.iter().any(|&a| (a - axis).length() < 1e-4) {
                false
            } else {
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
        {
            //axes through faces
            let nf = plat.faces[0].len();
            debug_assert!(plat.faces.iter_rows().all(|r| r.len() == nf));
            let angles = (1..nf).map(|i| TAU * (i as f32) / (nf as f32)).collect_vec();
            for &face_normal in &plat.face_normals {
                add_new_rotations(face_normal, &angles);
            }
        }
        {
            //axes through edges
            let angles = [TAU * 0.5];
            plat.edges.for_each_edge(|v1, v2| {
                let v1_dir = plat.vertices[v1].to_vec3();
                let v2_dir = plat.vertices[v2].to_vec3();
                let axis = (0.5 * v1_dir + 0.5 * v2_dir).normalized();
                add_new_rotations(axis, &angles);
            });
        }
        {
            //axes through vertices
            let angles = (1..degree).map(|i| TAU * (i as f32) / (degree as f32)).collect_vec();
            for &pos in &plat.vertices {
                let axis = pos.to_vec3().normalized();
                add_new_rotations(axis, &angles);
            }
        }

        rotations
    }

    fn find_symmetry_transforms(
        vertex_directions: &[Vec3],
        transforms: &[Matrix3x3],
        representative: &[usize],
    ) -> BoolCSR {
        debug_assert!(!transforms.is_empty());
        let mut res = BoolCSR::new();

        for (v, &dir) in izip!(0.., vertex_directions) {
            res.start_new_row();
            debug_assert!(dir.is_normalized());
            let repr_dir = vertex_directions[representative[v]];
            for (m, mat) in izip!(0.., transforms) {
                let im = mat * dir;
                let im_dist = (im - repr_dir).length();
                if im_dist < 1e-5 {
                    res.add_entry_in_last_row(m);
                }
            }
            debug_assert!(!res.last_row().is_empty());
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
        let mut nr_classes = 0;

        //the vertices of a platonic solid are all equivalent:
        //the graph looks the same from each position.
        nr_classes += 1;
        for c in class.iter_mut().take(graph.surface.nr_vertices()) {
            *c = 0;
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
            nr_classes += nr_edge_classes;
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
                    } else {
                        let new_c = nr_classes as u16;
                        nr_classes += 1;
                        class_map.insert(dists, new_c);
                        class[v] = new_c;
                    }
                }
            }
        }

        let class_representative = (0..nr_classes)
            .map(|c| {
                //take first vertex of a class as the class' representative
                class
                    .iter()
                    .enumerate()
                    .filter_map(|(i, &cc)| (cc as usize == c).then_some(i))
                    .next()
                    .unwrap()
            })
            .collect_vec();
        //this is only guaranteed because we know the order in which a triangulated platonic solid
        //adds it's vertices during construction.
        //it is however needed in Self::transform_all.
        debug_assert!(class_representative.iter().tuple_windows().all(|(a, b)| a <= b));

        let vertex_directions =
            Vec::from_iter(graph.positions().iter().map(|p| p.to_vec3().normalized()));
        let vertex_representative =
            Vec::from_iter(class.iter().map(|&c| class_representative[c as usize]));

        let symmetry_transform_matrices = 'try_reuse: {
            /// for a given platonic solid, this stores nr of vertices, vertex degree, all the transforms and to_representative.
            static KNOWN_PLATONIC_SYMMETRY_TRANSFORMS: std::sync::Mutex<
                Vec<(usize, usize, Vec<Matrix3x3>)>,
            > = std::sync::Mutex::new(Vec::new());

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
            &vertex_representative,
        );
        let symmetry_transforms = symmetry_transform_matrices
            .iter()
            .map(|m| ExplicitAutomorphism::new(graph.edges(), graph.positions(), *m))
            .collect_vec();

        debug_assert!({
            let mut nr_rots = vec![0; nr_classes];
            for (&c, rots) in izip!(&class, to_representative.iter_rows()) {
                nr_rots[c as usize] += rots.len();
            }
            for &nr in &nr_rots {
                //each rotation must be able to map some vertex of any class to the class' representative
                assert_eq!(nr, symmetry_transforms.len());
            }
            true
        });

        Some(Self {
            vertex_representative,
            symmetry_transforms,
            to_representative,
        })
    }

    pub fn vertex_representatives(&self) -> &[usize] {
        &self.vertex_representative
    }

    /// returns all transforms mapping vertex v to it's class' representative
    fn transforms_of(
        &self,
        v: usize,
    ) -> impl ExactSizeIterator<Item = &'_ ExplicitAutomorphism> + Clone {
        let transform_indices = self.to_representative.row(v);
        debug_assert_eq!(
            self.vertex_representative[v],
            self.symmetry_transforms[transform_indices[0]].forward[v]
        );
        transform_indices.iter().map(|&i| &self.symmetry_transforms[i])
    }
}

impl SymmetryGroup for ExplicitClasses {
    type Auto = ExplicitAutomorphism;

    fn repr_automorphisms(&self, v: usize) -> impl Iterator<Item = &Self::Auto> + '_ + Clone {
        self.transforms_of(v)
    }

    fn repr_of(&self, v: usize) -> usize {
        self.vertex_representative[v]
    }

    fn class_representatives(&self) -> impl Iterator<Item = usize> + '_ + Clone {
        self.vertex_representative.iter().copied().unique()
    }

    fn all_automorphisms(&self) -> &[Self::Auto] {
        &self.symmetry_transforms
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::Explicit(self)
    }

    fn nr_vertices(&self) -> usize {
        self.vertex_representative.len()
    }
}

impl<S: SymmetryGroup> From<&S> for ExplicitClasses {
    fn from(sym: &S) -> Self {
        let symmetry_transforms = {
            let mut vec = sym
                .all_automorphisms()
                .iter()
                .map(ExplicitAutomorphism::from)
                .collect_vec();
            vec.sort_by(|a, b| compare(a.forward(), b.forward()));
            vec
        };

        let mut vertex_representative = Vec::new();
        let mut to_representative = BoolCSR::new();
        for v in 0..sym.nr_vertices() {
            vertex_representative.push(sym.repr_of(v));
            let autos = sym.repr_automorphisms(v);
            to_representative.add_row(autos.into_iter().map(|auto| {
                symmetry_transforms
                    .binary_search_by(|a| compare(a.forward(), auto.forward()))
                    .unwrap()
            }));
        }

        Self {
            vertex_representative,
            symmetry_transforms,
            to_representative,
        }
    }
}

impl From<SymGroup> for ExplicitClasses {
    fn from(value: SymGroup) -> Self {
        match value {
            SymGroup::Explicit(e) => e,
            SymGroup::Torus6(t) => ExplicitClasses::from(&t),
            SymGroup::Torus4(t) => ExplicitClasses::from(&t),
            SymGroup::None(n) => ExplicitClasses::from(&n),
        }
    }
}

#[cfg(test)]
pub mod test {
    use super::*;

    fn sym_converts_to_explicit(s: &SymGroup) {
        fn converts_impl(s: &impl SymmetryGroup) {
            use std::collections::BTreeSet;
            let e = ExplicitClasses::from(s);
            assert_eq!(s.all_automorphisms().len(), e.all_automorphisms().len());
            assert_eq!(s.nr_vertices(), e.nr_vertices());
            assert_eq!(
                s.class_representatives().collect::<BTreeSet<_>>(),
                e.class_representatives().collect::<BTreeSet<_>>()
            );
            for v in 0..s.nr_vertices() {
                assert_eq!(s.repr_of(v), e.repr_of(v));
                let mut s_autos = s.repr_automorphisms(v).collect_vec();
                let mut e_autos = e.repr_automorphisms(v).collect_vec();
                assert_eq!(s_autos.len(), e_autos.len());
                s_autos.sort_by(|a, b| compare(a.forward(), b.forward()));
                e_autos.sort_by(|a, b| compare(a.forward(), b.forward()));
                for (s_a, e_a) in izip!(s_autos, e_autos) {
                    assert!(compare(s_a.forward(), e_a.forward()).is_eq());
                }
            }
        }

        match s {
            SymGroup::Explicit(e) => converts_impl(e),
            SymGroup::None(n) => converts_impl(n),
            SymGroup::Torus4(t4) => converts_impl(t4),
            SymGroup::Torus6(t6) => converts_impl(t6),
        }
    }

    #[test]
    fn torus4_converts_to_explicit() {
        let g3 = Embedding3D::new_subdivided_squares_torus(3);
        sym_converts_to_explicit(g3.sym_group());

        let g5 = Embedding3D::new_subdivided_squares_torus(6);
        sym_converts_to_explicit(g5.sym_group());

        let g9 = Embedding3D::new_subdivided_squares_torus(9);
        sym_converts_to_explicit(g9.sym_group());
    }

    #[test]
    fn torus6_converts_to_explicit() {
        let g3 = Embedding3D::new_subdivided_triangle_torus(3);
        sym_converts_to_explicit(g3.sym_group());

        let g5 = Embedding3D::new_subdivided_triangle_torus(6);
        sym_converts_to_explicit(g5.sym_group());

        let g9 = Embedding3D::new_subdivided_triangle_torus(9);
        sym_converts_to_explicit(g9.sym_group());
    }

    #[test]
    fn expicit_converts_to_explicit() {
        let i10 = Embedding3D::new_subdivided_icosahedron(10);
        sym_converts_to_explicit(i10.sym_group());

        let o10 = Embedding3D::new_subdivided_octahedron(10);
        sym_converts_to_explicit(o10.sym_group());

        let c10 = Embedding3D::new_subdivided_cube(10);
        sym_converts_to_explicit(c10.sym_group());
    }
}
