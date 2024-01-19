

use itertools::izip;
use smallvec::SmallVec;

use crate::geo::*;
use self::bool_csr::BoolCSR;

use super::*;

/// automorphism of vertex set of a graph
/// vertices are named [`0..self.nr_vertices()`]
pub trait Automorphism {
    type Iter<'a>: ExactSizeIterator<Item = usize> + 'a + Clone where Self: 'a;

    fn forward<'a>(&'a self) -> Self::Iter<'a>;
    fn backward<'a>(&'a self) -> Self::Iter<'a>;

    fn nr_vertices(&self) -> usize;

    fn apply_forward(&self, v: usize) -> usize {
        debug_assert!(v < self.nr_vertices());
        self.forward().nth(v).unwrap()
    }

    fn apply_backward(&self, v: usize) -> usize {
        debug_assert!(v < self.nr_vertices());
        self.backward().nth(v).unwrap()
    }
}

pub struct Identity {
    nr_vertices: usize,
}

impl Identity {
    pub fn new(nr_vertices: usize) -> Self {
        Identity { nr_vertices }
    }
}

impl Automorphism for Identity {
    type Iter<'a> = std::ops::Range<usize>;

    fn forward<'a>(&'a self) -> Self::Iter<'a> {
        0..self.nr_vertices()
    }

    fn backward<'a>(&'a self) -> Self::Iter<'a> {
        0..self.nr_vertices()
    }

    fn nr_vertices(&self) -> usize {
        self.nr_vertices
    }
}

pub trait SymmetryGroup {
    type Auto: Automorphism;
    type AutoIter<'a>: IntoIterator<Item = &'a Self::Auto> + Clone where Self: 'a;

    /// applies an authomorphism to vertices in place, returns all authomorphisms which 
    /// map original vertices to result.
    /// order of vertices may be altered.
    /// 
    /// this function must guarantee to find a unique representative of vertices and 
    /// ALL autos bringing them there.
    fn to_representative<'a>(&'a self, vertices: & mut[usize]) -> Self::AutoIter<'a>;

    /// enumerates one vertex of each vertex class. this vertex will be result of [`Self::to_representative`]
    /// if a vertex of it's class is passed as only vertex.
    fn class_representatives<'a>(&'a self) -> <Self::Auto as Automorphism>::Iter<'a>;
}

pub struct NoSymmetry {
    identity: Identity,
}

impl NoSymmetry {
    pub fn new(nr_vertices: usize) -> Self {
        Self { identity: Identity::new(nr_vertices) }
    }
}

impl SymmetryGroup for NoSymmetry {
    type Auto = Identity;
    type AutoIter<'a> = std::iter::Once<&'a Identity>;

    fn to_representative<'a>(&'a self, vertices: & mut[usize]) -> Self::AutoIter<'a> {
        vertices.sort();
        std::iter::once(&self.identity)
    }

    fn class_representatives<'a>(&'a self) -> <Self::Auto as Automorphism>::Iter<'a> {
        self.identity.forward()
    }
}

/// represents a graph automorphism:
/// 
/// -> iff vertices `u` and `v` share an edge, then vertices `self.forward[u]` and `self.forward[v]` share an edge.
/// 
/// -> for any vertex `v` holds that `v == self.forward[self.backward[v]]`
#[derive(Clone)]
pub struct ExplicitAutomorphism {
    /// forward[v] is vertex where v is mapped to
    forward: Vec<usize>,
    /// backward[v] is vertex which maps to v
    backward: Vec<usize>,
}

impl Automorphism for ExplicitAutomorphism {
    type Iter<'a> = std::iter::Copied<std::slice::Iter<'a, usize>>;

    fn forward<'a>(&'a self) -> Self::Iter<'a> {
        self.forward.iter().copied()
    }

    fn backward<'a>(&'a self) -> Self::Iter<'a> {
        self.backward.iter().copied()
    }

    fn nr_vertices(&self) -> usize {
        debug_assert_eq!(self.forward.len(), self.backward.len());
        self.forward.len()
    }
}

impl ExplicitAutomorphism {
    pub fn identity(n: usize) -> Self {
        Self { 
            forward: (0..n).collect_vec(), 
            backward: (0..n).collect_vec(),
        }
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

/// stores every automorphism explicitly
#[derive(Clone)]
pub struct ExplicitClasses {
    /// one entry per vertex, stores to which class vertex belongs
    class: Vec<u16>,

    /// one entry per vertex, logs to which vertex each vertex is mapped to
    vertex_representative: Vec<usize>,

    /// one entry per class, keeps track which vertex represents a class
    class_representative: Vec<usize>,

    //one entry per element in symmetry group, starting with the identity
    symmetry_transforms: Vec<ExplicitAutomorphism>,

    /// one row per vertex, indexes in [`self.symmetry_maps`]. 
    /// all entries in row are indices of transforms mapping vertex to it's representative
    to_representative: BoolCSR,
}

impl ExplicitClasses {

    pub fn nr_vertices(&self) -> usize {
        let len = self.class.len();
        debug_assert_eq!(len, self.vertex_representative.len());
        debug_assert_eq!(len, self.to_representative.nr_rows());
        len
    }

    pub fn new_no_symmetry(nr_vertices: usize) -> Self {
        let mut to_representative = BoolCSR::new(nr_vertices);
        for _ in 0..nr_vertices {
            to_representative.add_row();
            to_representative.add_entry_in_last_row(0);
        }
        Self { 
            class: (0..(nr_vertices as u16)).collect_vec(), 
            vertex_representative: (0..nr_vertices).collect_vec(), 
            class_representative: (0..nr_vertices).collect_vec(), 
            symmetry_transforms: vec![ExplicitAutomorphism::identity(nr_vertices)], 
            to_representative
        }
    }

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

    fn find_symmetry_transforms(vertex_directions: &[Vec3], transforms: &[Matrix3x3], representative: &[usize]) -> BoolCSR {
        debug_assert!(transforms.len() > 0);
        let mut res = BoolCSR::new(transforms.len());

        for (v, &dir) in izip!(0.., vertex_directions) {
            res.add_row();
            debug_assert!(dir.is_normalized());
            let repr_dir = vertex_directions[representative[v]];
            for (m, mat) in izip!(0.., transforms) {
                let im = mat * dir;
                let im_dist = (im - repr_dir).length();
                if im_dist < 1e-5 {
                    res.add_entry_in_last_row(m);
                }
            }
            debug_assert!(res.last_row().len() > 0);
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
                    }
                    else {
                        let new_c = nr_classes as u16;
                        nr_classes += 1;
                        class_map.insert(dists, new_c);
                        class[v] = new_c;
                    }
                }
            }
        }

        let class_representative = (0..nr_classes).map(|c| {
            //take first vertex of a class as the class' representative
            class.iter().enumerate().filter_map(|(i, &cc)| (cc as usize == c).then_some(i)).next().unwrap()
        }).collect_vec();
        //this is only guaranteed because we know the order in which a triangulated platonic solid
        //adds it's vertices during construction.
        //it is however needed in Self::transform_all.
        debug_assert!(class_representative.iter().tuple_windows().all(|(a, b)| a <= b));

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
            ExplicitAutomorphism::new(graph.edges(), graph.positions(), *m)
        ).collect_vec();

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

    pub fn vertex_representatives(&self) -> &[usize] {
        &self.vertex_representative
    }

    pub fn all_transforms(&self) -> &[ExplicitAutomorphism] {
        &self.symmetry_transforms
    }

    #[allow(dead_code)]
    pub fn nr_classes(&self) -> usize {
        self.class_representative.len()
    }

    /// returns all transforms mapping vertex v to it's class' representative
    fn transforms_of<'a>(&'a self, v: usize) -> impl ExactSizeIterator<Item = &'a ExplicitAutomorphism> + Clone {
        let transform_indices = self.to_representative.row(v);
        debug_assert_eq!(
            self.vertex_representative[v], 
            self.symmetry_transforms[transform_indices[0]].forward[v]
        );
        transform_indices.iter().map(|&i| &self.symmetry_transforms[i])
    }

    /// chooses the rotation, such that in the rotated result the cop defining the rotation is moved
    /// to the smallest index.
    fn transform_all(&self, cops: &mut [usize]) -> SmallVec<[&ExplicitAutomorphism; 4]> {
        if cops.len() == 0 {
            return smallvec::smallvec![&self.symmetry_transforms[0]];
        }

        cops.sort_by_key(|&c| self.class[c]);
        let rotate_class = self.class[cops[0]];
        let mut rotations = cops
            .iter()
            .take_while(|&&c| self.class[c] == rotate_class)
            .flat_map(|&c| self.transforms_of(c));

        if cops.len() == 1 {
            let rot = rotations.next().unwrap();
            cops[0] = rot.forward[cops[0]];
            return smallvec::smallvec![rot];
        }

        //each configuration gets a value. the only important thing is that
        //config_hash_value is injective, because we choose the configuration with lowest value.
        //if two transforations of the same configuration can be chosen, the bruteforce algorithm will fail.
        //(because only one of those will be put in the update queue when they need to be updated)
        let config_hash_value = |rotated: &[_]| { 
            let mut acc = 0;
            for &c in rotated.iter().rev() {
                debug_assert!(c < self.nr_vertices());
                acc += c;
                acc *= self.nr_vertices();
            }
            acc
        };
        let mut best_is = SmallVec::<[usize; 4]>::new();
        let mut best_val = usize::MAX;
        for (i, rotation) in rotations.clone().enumerate() {
            let mut rotated = [0usize; bruteforce::MAX_COPS];
            let rotated = &mut rotated[..cops.len()];
            for (rotated, &c) in izip!(rotated.iter_mut(), cops.iter()) {
                *rotated = rotation.forward[c];
            }
            rotated.sort();
            //this relies on the classes beeing sorted by what the smallest vertex appearing in one is.
            debug_assert_eq!(self.class[rotated[0]], rotate_class);

            let new_val = config_hash_value(&rotated);
            if new_val == best_val {
                debug_assert!({
                    //two transformations yielding the same function value is only expected to
                    //happen, if both also yield the same configuration.
                    let prev_best_rot = rotations.clone().nth(best_is[0]).unwrap();
                    let mut hit = [false; bruteforce::MAX_COPS];
                    for &c in cops.iter() {
                        let prev_rotated = prev_best_rot.forward[c];
                        if let Some(j) = rotated
                            .iter()
                            .enumerate()
                            .position(|(j, &cc)| !hit[j] && cc == prev_rotated) 
                        {
                            hit[j] = true;
                        }
                        else { panic!() }
                    }
                    hit[..rotated.len()].into_iter().all(|&x| x)
                });
                best_is.push(i);
            }
            if new_val < best_val {
                best_val = new_val;
                best_is.clear();
                best_is.push(i);
            }
        }
        
        assert!(!best_is.is_empty());
        let mut best_rotations = SmallVec::<[&ExplicitAutomorphism; 4]>::new();
        let mut indexted_rotations = rotations.enumerate();
        for best_i in best_is {
            loop {
                let (i, rotation) = indexted_rotations.next().unwrap();
                if i == best_i {
                    best_rotations.push(rotation);
                    break;
                }
            }
        }
        for c in cops.iter_mut() {
            *c = best_rotations[0].forward[*c];
        }
        cops.sort();
        best_rotations
    }
}

impl SymmetryGroup for ExplicitClasses {
    type Auto = ExplicitAutomorphism;
    type AutoIter<'a> = SmallVec<[&'a ExplicitAutomorphism; 4]>;

    fn to_representative<'a>(&'a self, vertices: & mut[usize]) -> Self::AutoIter<'a> {
        self.transform_all(vertices)
    }

    fn class_representatives<'a>(&'a self) -> <Self::Auto as Automorphism>::Iter<'a> {
        self.class_representative.iter().copied()
    }
}
