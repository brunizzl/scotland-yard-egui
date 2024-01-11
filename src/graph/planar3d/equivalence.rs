

use itertools::izip;

use crate::geo::*;
use super::*;


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

    //only works for those platonic solids with threeangles as base shape
    fn enumerate_platonic_symmetry_maps(plat: &ConvexTriangleHull) -> Vec<Matrix3x3> {
        //indicator for platonic, not guarantee
        debug_assert_eq!(plat.edges.max_degree(), plat.edges.min_degree());
        let mut res = Vec::new();
        for (tri, normal) in izip!(&plat.triangles, &plat.face_normals) {

        }
        res
    }

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
            //this loop relies on each edge beeing iterated over once in each direction 
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

    pub fn apply_transform(&self, edges: &EdgeList, rot: &Matrix3x3, vertex: usize) -> usize {
        let pre_dir = self.vertex_directions[vertex];
        let post_dir = rot * pre_dir;
        let (res, error) = edges.find_local_minimum(|v| -self.vertex_directions[v].dot(post_dir), vertex);
        debug_assert!(error < 1e-4);
        res
    }
}