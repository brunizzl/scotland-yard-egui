use egui::{emath::RectTransform, pos2, Pos2, Rect, Vec2};

pub mod vec3;
pub use vec3::*;

pub mod pos3;
pub use pos3::*;

pub mod mat;
pub use mat::*;

type Line2 = (Pos2, Vec2);

pub fn line_from_to(from: Pos2, to: Pos2) -> Line2 {
    (from, to - from)
}

pub fn left_of_line((a, dir): Line2, p: Pos2) -> bool {
    signed_dist((a, dir), p) < 0.0
}

/// assumes dir to be normalized.
pub fn signed_dist((a, dir): Line2, p: Pos2) -> f32 {
    dir.x * (a.y - p.y) - dir.y * (a.x - p.x)
}

#[allow(dead_code)]
pub fn project_to_line((a, dir): Line2, p: Pos2) -> Pos2 {
    //dividing by length squared normalized both occurences of dir at once.
    a + Vec2::dot(p - a, dir) / dir.length_sq() * dir
}

/// returns [`t`] in [`a + t * da = b + s * db`]
pub fn intersection_step((a, da): Line2, (b, db): Line2) -> f32 {
    //a, da == p, r
    //b, db == q, s
    let numerator = (a.y - b.y) * db.x - (a.x - b.x) * db.y;
    let denominator = da.x * db.y - da.y * db.x;
    numerator / denominator
}

pub fn lines_intersect(l1: Line2, l2: Line2) -> bool {
    (0.0..=1.0).contains(&intersection_step(l1, l2))
        && (0.0..=1.0).contains(&intersection_step(l2, l1))
}

/// takes Vec3 to Vec2 where the subspace is spanned by new_x_axis and new_y_axis
#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
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
        Self {
            new_x: *a,
            new_y: *b,
            new_z: *c,
        }
    }

    pub fn from_transposed([a, b, c]: &[Vec3; 3]) -> Self {
        let u = vec3(a.x, b.x, c.x);
        let v = vec3(a.y, b.y, c.y);
        let w = vec3(a.z, b.z, c.z);
        Self::new(&[u, v, w])
    }

    /// positive if vec is in front of plane, negative if behind, zero if on subspace
    pub fn signed_dist(&self, vec: Vec3) -> f32 {
        self.new_z.dot(vec)
    }

    pub fn project_pos(&self, pos: Pos3) -> Pos2 {
        let as_vec = pos.to_vec3();
        let x = self.new_x.dot(as_vec);
        let y = self.new_y.dot(as_vec);
        pos2(x, y)
    }
}

#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
pub struct ToScreen {
    pub to_plane: Project3To2,
    pub move_rect: RectTransform,

    screen_boundaries: [Line2; 4],
}

impl ToScreen {
    pub fn new(project: Project3To2, move_rect: RectTransform) -> Self {
        let screen = move_rect.to();
        let b1 = line_from_to(screen.left_bottom(), screen.left_top());
        let b2 = line_from_to(screen.left_top(), screen.right_top());
        let b3 = line_from_to(screen.right_top(), screen.right_bottom());
        let b4 = line_from_to(screen.right_bottom(), screen.left_bottom());
        Self {
            to_plane: project,
            move_rect,
            screen_boundaries: [b1, b2, b3, b4],
        }
    }

    pub fn screen(&self) -> &Rect {
        self.move_rect.to()
    }

    pub fn apply(&self, pos: Pos3) -> Pos2 {
        let projected = self.to_plane.project_pos(pos);
        self.move_rect.transform_pos(projected)
    }

    pub fn faces_camera(&self, face_normal: Vec3) -> bool {
        self.to_plane.signed_dist(face_normal) > 0.0
    }

    fn crosses_screen_boundary(&self, line: Line2) -> bool {
        self.screen_boundaries.iter().any(|&b| lines_intersect(b, line))
    }

    /// assumes a, b, c to be ordered counterclockwise
    pub fn triangle_visible(&self, a: Pos3, b: Pos3, c: Pos3) -> bool {
        let u = self.apply(a);
        let v = self.apply(b);
        let w = self.apply(c);
        let screen = self.move_rect.to();

        if screen.contains(u) || screen.contains(v) || screen.contains(w) {
            //at least one corner is visible
            return true;
        }

        let line_uv = line_from_to(u, v);
        let line_vw = line_from_to(v, w);
        let line_wu = line_from_to(w, u);
        if self.crosses_screen_boundary(line_uv)
            || self.crosses_screen_boundary(line_vw)
            || self.crosses_screen_boundary(line_wu)
        {
            //at least one edge is visible
            return true;
        }
        //if we get here, we know, that no part of the triangle an the screen boundary intersect,
        //and that no point of the triangle is contained in the screen.
        //thus the only option left is for the screen to be completely contained in the triangle.
        //which screen point we test to lie inside the triangle is thus irrelevant.
        //just to be save numerically, we use the center.
        let screen_center = screen.center();
        if left_of_line(line_uv, screen_center)
            && left_of_line(line_vw, screen_center)
            && left_of_line(line_wu, screen_center)
        {
            return true;
        }

        false
    }

    pub fn pos_visible(&self, pos: Pos3) -> bool {
        let mapped = self.apply(pos);
        self.move_rect.to().contains(mapped)
    }
}

pub fn gram_schmidt_3d([v1, v2, v3]: &mut [Vec3; 3]) {
    *v1 = v1.normalized();
    *v2 = (*v2 - v1.dot(*v2) * *v1).normalized();
    *v3 = (*v3 - v1.dot(*v3) * *v1 - v2.dot(*v3) * *v2).normalized();
}

#[inline]
pub fn plane_normal(p1: Pos3, p2: Pos3, p3: Pos3) -> Vec3 {
    Vec3::cross(p2 - p1, p3 - p1).normalized()
}
