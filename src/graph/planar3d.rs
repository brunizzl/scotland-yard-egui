
use egui::*;
use itertools::Itertools;

use crate::geo::{Pos3, Vec3, pos3, self};


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

/// expect boundary to form a circle, order vertices in circle s.t. neighboring vertices follow each other
fn order_face_boundary(boundary: &mut [usize], neighbors: &[Vec<usize>]) {
    debug_assert!(boundary.len() > 2);

    'outher: for i1 in 0..(boundary.len() - 1) {
        let v1 = boundary[i1];
        let v1_neighbors = &neighbors[v1][..];
        let i2 = i1 + 1;
        for search_i in i2..boundary.len() {
            let v = boundary[search_i];
            if v1_neighbors.contains(&v) {
                boundary.swap(i2, search_i);
                continue 'outher;
            }
        }
        panic!("vertex boundary is not closed");
    }
    debug_assert!(neighbors[boundary[0]].contains(boundary.last().unwrap()));
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
        let vertex_neighbors: Vec<Vec<usize>> = find_related!(vertex_positions, vertex_positions, 
            |p1: Pos3, p2: Pos3| (1.9 * scale .. 2.1 * scale).contains(&(p1 - p2).length()));

        let face_neighbors: Vec<Vec<usize>> = find_related!(face_normals, face_normals, 
            |v1: Vec3, v2: Vec3| Vec3::dot(v1, v2) == 0.0);
            
        let mut face_boundary_vertices: Vec<Vec<usize>> = find_related!(face_normals, vertex_positions,
            |v1: Vec3, p2: Pos3| Vec3::dot(v1, p2.to_vec3()) > 0.0);
        for boundary in &mut face_boundary_vertices {
            order_face_boundary(boundary, &vertex_neighbors);
        }

        Self { vertex_positions, vertex_neighbors, face_neighbors, face_boundary_vertices, face_normals }
    }

    pub fn draw_visible_edges(&self, to_screen: &geo::ToScreen, painter: &Painter, stroke: Stroke) 
    {
        for (&normal, boundary) in self.face_normals.iter().zip(self.face_boundary_vertices.iter()) {
            if to_screen.visible(normal) {
                for (&v1, &v2) in boundary.iter().circular_tuple_windows() {
                    let p1 = self.vertex_positions[v1];
                    let p2 = self.vertex_positions[v2];

                    let edge = [
                        to_screen.apply(p1), 
                        to_screen.apply(p2)];
                    let line = Shape::LineSegment { points: edge, stroke };
                    painter.add(line);
                }
            }
        }
    }
}






















