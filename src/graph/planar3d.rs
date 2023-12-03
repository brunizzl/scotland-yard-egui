


use crate::geo::{Pos3, Vec3, pos3};


pub struct ConvexPolyhedron {
    vertex_positions: Vec<Pos3>,

    /// indices of neighboring vertices 
    vertex_neighbors: Vec<Vec<usize>>, 

    /// indices of neighboring faces
    face_neighbors: Vec<Vec<usize>>, 

    /// indices of vertices surrounding face
    face_boundary_vertices: Vec<Vec<usize>>, 

    /// outher unit normal of each face
    /// this entry decides if a face is rendered or not: only camera-facing faces are.
    face_normals: Vec<Vec3>, 
}

macro_rules! find_related {
    ($xs:expr, $ys:expr, $f:expr) => {
        $xs
        .iter()
        .map(|&x| $ys
            .iter()
            .enumerate()
            .filter_map(|(i, &y)| if $f(x, y) { Some(i) } else { None })
            .collect()
        ).collect()
    };
}

impl ConvexPolyhedron {
    pub fn new_cube(scale: f32) -> Self {
        let p = scale;
        let n = -scale;
        let vertex_positions = vec![
            pos3(p, p, p),
            pos3(p, p, n),
            pos3(p, n, p),
            pos3(p, n, n),
            pos3(n, p, p),
            pos3(n, p, n),
            pos3(n, n, p),
            pos3(n, n, n),
        ];
        let face_normals = vec![
            Vec3::X,
            -Vec3::X,
            Vec3::Y,
            -Vec3::Y,
            Vec3::Z,
            -Vec3::Z,
        ];
        let vertex_neighbors = find_related!(vertex_positions, vertex_positions, 
            |p1: Pos3, p2: Pos3| (1.9 * scale .. 2.1 * scale).contains(&(p1 - p2).length()));

        let face_neighbors = find_related!(face_normals, face_normals, 
            |v1: Vec3, v2: Vec3| Vec3::dot(v1, v2) == 0.0);
            
        let face_boundary_vertices = find_related!(face_normals, vertex_positions,
            |v1: Vec3, p2: Pos3| Vec3::dot(v1, p2.to_vec3()) > 0.0);

        Self { vertex_positions, vertex_neighbors, face_neighbors, face_boundary_vertices, face_normals }
    }

}






















