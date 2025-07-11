use egui::{Pos2, Rect, Vec2, emath::RectTransform, pos2};

pub mod vec3;
pub use vec3::*;

pub mod pos3;
pub use pos3::*;

pub mod mat;
pub use mat::*;

#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
pub struct Line2(pub Pos2, pub Vec2);

impl From<Line2> for [Pos2; 2] {
    fn from(Line2(a, da): Line2) -> Self {
        [a, a + da]
    }
}

pub fn line_from_to(from: Pos2, to: Pos2) -> Line2 {
    Line2(from, to - from)
}

pub fn left_of_line(line: Line2, p: Pos2) -> bool {
    signed_dist(line, p) < 0.0
}

/// assumes dir to be normalized.
pub fn signed_dist(Line2(a, dir): Line2, p: Pos2) -> f32 {
    dir.x * (a.y - p.y) - dir.y * (a.x - p.x)
}

#[allow(dead_code)]
pub fn project_to_line(Line2(a, dir): Line2, p: Pos2) -> Pos2 {
    //dividing by length squared normalized both occurences of dir at once.
    a + Vec2::dot(p - a, dir) / dir.length_sq() * dir
}

/// returns [`t`] in [`a + t * da = b + s * db`]
pub fn intersection_step(Line2(a, da): Line2, Line2(b, db): Line2) -> f32 {
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
pub struct BoundedRect {
    pub rect: Rect,
    pub boundary: [Line2; 4],
}

impl BoundedRect {
    pub fn from_rect(rect: Rect) -> Self {
        let b1 = line_from_to(rect.left_bottom(), rect.left_top());
        let b2 = line_from_to(rect.left_top(), rect.right_top());
        let b3 = line_from_to(rect.right_top(), rect.right_bottom());
        let b4 = line_from_to(rect.right_bottom(), rect.left_bottom());
        Self {
            rect,
            boundary: [b1, b2, b3, b4],
        }
    }

    pub fn crosses_boundary(&self, line: Line2) -> bool {
        self.boundary.iter().any(|&b| lines_intersect(b, line))
    }

    /// if line intersects self, returns part of line that intersects,
    /// else returns None.
    pub fn trim(&self, mut line: Line2) -> Option<Line2> {
        if self.crosses_boundary(line) {
            for &b in &self.boundary {
                let step = intersection_step(line, b);
                if (0.0..1.0).contains(&step) && (0.0..1.0).contains(&intersection_step(b, line)) {
                    let Line2(a, da) = line;
                    let fst_dir = da * step;
                    if self.rect.contains(a + 0.9999 * fst_dir) {
                        line = Line2(a, fst_dir);
                    } else {
                        debug_assert!(self.rect.contains(a + 1.0001 * fst_dir));
                        let snd_dir = da * (1.0 - step);
                        line = Line2(a + fst_dir, snd_dir);
                    }
                }
            }
            Some(line)
        } else if self.rect.contains(line.0) {
            Some(line)
        } else {
            None
        }
    }
}

#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
pub struct ToScreen {
    pub to_plane: Project3To2,
    pub move_rect: RectTransform,

    screen: BoundedRect,
}

impl ToScreen {
    pub fn new(project: Project3To2, move_rect: RectTransform) -> Self {
        let screen = BoundedRect::from_rect(*move_rect.to());
        Self {
            to_plane: project,
            move_rect,
            screen,
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

    /// assumes a, b, c to be ordered counterclockwise
    pub fn face_maybe_visible(&self, corners: impl Iterator<Item = Pos3>) -> bool {
        let corners_on_screen =
            smallvec::SmallVec::<[Pos2; 8]>::from_iter(corners.map(|c| self.apply(c)));
        let bounding_box = egui::Rect::from_points(&corners_on_screen);
        let screen = *self.move_rect.to();
        Rect::intersects(bounding_box, screen)
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
