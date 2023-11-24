
use egui::{Pos2, Vec2};

type Line = (Pos2, Vec2);

pub fn line_from_to(from: Pos2, to: Pos2) -> Line {
    (from, to - from)
}

pub fn left_of_line((a, dir): Line, p: Pos2) -> bool {
    dir.x * (p.y - a.y) - dir.y * (p.x - a.x) > 0.0
}

//assumes dir to be normalized.
pub fn signed_dist((a, dir): Line, p: Pos2) -> f32 {
    dir.x * (a.y - p.y) - dir.y * (a.x - p.x)
}

pub fn project_to_line((a, dir): Line, p: Pos2) -> Pos2 {
    //dividing by length squared normalized both occurences of dir at once.
    a + Vec2::dot(p - a, dir) / dir.length_sq() * dir
}