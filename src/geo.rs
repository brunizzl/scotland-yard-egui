
use egui::{Pos2, Vec2, pos2, vec2, emath::RectTransform};

pub mod vec3;
pub use vec3::*;

pub mod pos3;
pub use pos3::*;




type Line2 = (Pos2, Vec2);

pub fn line_from_to(from: Pos2, to: Pos2) -> Line2 {
    (from, to - from)
}

pub fn left_of_line((a, dir): Line2, p: Pos2) -> bool {
    signed_dist((a, dir), p) < 0.0
}

//assumes dir to be normalized.
pub fn signed_dist((a, dir): Line2, p: Pos2) -> f32 {
    dir.x * (a.y - p.y) - dir.y * (a.x - p.x)
}

pub fn project_to_line((a, dir): Line2, p: Pos2) -> Pos2 {
    //dividing by length squared normalized both occurences of dir at once.
    a + Vec2::dot(p - a, dir) / dir.length_sq() * dir
}

/// takes Vec3 to Vec2 where the subspace is spanned by new_x_axis and new_y_axis
pub struct Project3To2 {
    /// eigenvector with eigenvalue 1, portion in that direction will be returned as first Vec2 entry
    /// vector is expected to be normalized.
    pub new_x: Vec3,
    /// eigenvector with eigenvalue 1, portion in that direction will be returned as second Vec2 entry
    /// vector is expected to be normalized.
    pub new_y: Vec3,
    /// eigenvector with eigenvalue 0, portion in that direction will not be returned
    /// vector is expected to be normalized.
    pub new_z: Vec3,
}

impl Project3To2 {
    pub fn new([a, b, c]: &[Vec3; 3]) -> Self {
        // assume normalized
        debug_assert!(a.length().abs() - 1.0 < 1e-5);
        debug_assert!(b.length().abs() - 1.0 < 1e-5);
        debug_assert!(c.length().abs() - 1.0 < 1e-5);

        //assume orthogonal
        debug_assert!(a.dot(*b).abs() < 1e-5);
        debug_assert!(a.dot(*c).abs() < 1e-5);
        debug_assert!(b.dot(*c).abs() < 1e-5);
        Self { new_x: *a, new_y: *b, new_z: *c }
    }

    pub fn new_transposed([a, b, c]: &[Vec3; 3]) -> Self {
        let u = vec3(a.x, b.x, c.x);
        let v = vec3(a.y, b.y, c.y);
        let w = vec3(a.z, b.z, c.z);
        Self::new(&[u, v, w])
    }

    /// positive if vec is in front of plane, negative if behind, zero if on subspace
    pub fn signed_dist(&self, vec: Vec3) -> f32 {
        self.new_z.dot(vec)
    }

    pub fn project_vec(&self, vec: Vec3) -> Vec2 {
        let x = self.new_x.dot(vec);
        let y = self.new_y.dot(vec);
        vec2(x, y)
    }

    pub fn project_pos(&self, pos: Pos3) -> Pos2 {
        let as_vec = pos.to_vec3();
        let x = self.new_x.dot(as_vec);
        let y = self.new_y.dot(as_vec);
        pos2(x, y)
    }
}

pub struct ToScreen {
    pub to_plane: Project3To2,
    pub move_rect: RectTransform,
}

impl ToScreen {
    pub fn new(project: Project3To2, to_screen: RectTransform) -> Self {
        Self { to_plane: project, move_rect: to_screen }
    }

    pub fn apply(&self, pos: Pos3) -> Pos2 {
        let projected = self.to_plane.project_pos(pos);
        self.move_rect.transform_pos(projected)
    }

    pub fn faces_camera(&self, face_normal: Vec3) -> bool {
        self.to_plane.signed_dist(face_normal) > 0.0
    }
}

pub fn gram_schmidt_3d([v1, v2, v3]: &mut [Vec3; 3]) {
    *v1 = v1.normalized();
    *v2 = (*v2 - v1.dot(*v2) * *v1).normalized();
    *v3 = (*v3 - v1.dot(*v3) * *v1 - v2.dot(*v3) * *v2).normalized();
}

