

use itertools::izip;

use crate::geo::*;
use super::*;


/// in symmetric graphs, each vertex can be put into an equivalence class. 
#[derive(Clone)]
pub struct EquivalenceClasses {
    /// one entry per vertex, stores to which class vertex belongs
    class: Vec<u16>,

    /// one entry per vertex, logs to which vertex each vertex is mapped to
    vertex_representative: Vec<usize>,

    /// one entry epr class, keeps track which vertex represents a class
    class_representative: Vec<usize>,

    /// one entry per vertex, all positions normalized
    vertex_directions: Vec<Vec3>,

    /// lists all rotations / +reflections which respect the symmetry of the embedding
    symmetry_transforms: Vec<Matrix3x3>,

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

        let symmetry_transforms = 'try_reuse: {
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
        let to_representative = Self::find_symmetry_transforms(&vertex_directions, &symmetry_transforms, &vertex_representative);

        Some(Self {
            class, 
            vertex_representative,
            class_representative,
            vertex_directions,
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

    pub fn all_transforms(&self) -> &[Matrix3x3] {
        &self.symmetry_transforms
    }

    #[allow(dead_code)]
    pub fn nr_classes(&self) -> usize {
        self.class_representative.len()
    }

    pub fn apply_transform(&self, edges: &EdgeList, rot: &Matrix3x3, v: usize) -> usize {
        let pre_dir = self.vertex_directions[v];
        let post_dir = rot * pre_dir;
        let (res, error) = edges.find_local_minimum(|v| -self.vertex_directions[v].dot(post_dir), v);
        debug_assert!(error < 1e-4);
        debug_assert_eq!(self.class[v], self.class[res]);
        debug_assert_eq!(self.vertex_representative[v], self.vertex_representative[res]);
        res
    }

    fn transform_of(&self, v: usize) -> (&Matrix3x3, usize) {
        let transform_nr = self.to_representative[v];
        (
            &self.symmetry_transforms[transform_nr], 
            self.vertex_representative[v],
        )
    }

    /// chooses the rotation, such that in the rotated result the cop defining the rotation is moved
    /// to the smallest index.
    pub fn transform_all(&self, edges: &EdgeList, cops: &mut [usize]) -> Matrix3x3 {
        if cops.len() == 0 {
            return Matrix3x3::IDENTITY;
        }
        if cops.len() == 1 {
            let (rot, repr) = self.transform_of(cops[0]);
            cops[0] = repr;
            return *rot;
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
            debug_assert_eq!(
                fst_cop_repr, 
                self.apply_transform(edges, rot, fst_cop)
            );
            for c in rest_cops.iter_mut() {
                *c = self.apply_transform(edges, rot, *c);
            }
            rest_cops.sort();
            if rest_cops[0] >= fst_cop_repr {
                cops[0] = fst_cop_repr;
                for j in 1..cops.len() {
                    cops[j] = rotated[j];
                }

                return *rot;
            }
        }
        panic!("one always finds a cop that defines \
        a rotation and occupies the smallest vertex after rotating.");
    }
}
