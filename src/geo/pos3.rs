use std::ops::{Add, AddAssign, Sub, SubAssign};

use super::vec3::*;

/// copy of egui's Pos2, but with one extra dimension
#[repr(C)]
#[derive(Clone, Copy, Default, PartialEq)]
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Pos3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[inline(always)]
pub const fn pos3(x: f32, y: f32, z: f32) -> Pos3 {
    Pos3 { x, y, z }
}

// ----------------------------------------------------------------------------

impl Pos3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// The vector from origin to this position
    #[inline(always)]
    pub fn to_vec3(self) -> Vec3 {
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    #[inline]
    pub fn distance(self, other: Self) -> f32 {
        (self - other).length()
    }

    #[inline]
    pub fn distance_sq(self, other: Self) -> f32 {
        (self - other).length_sq()
    }

    /// True if all members are also finite.
    #[inline(always)]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// True if any member is NaN.
    #[inline(always)]
    pub fn any_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    #[must_use]
    #[inline]
    pub fn min(self, other: Self) -> Self {
        pos3(self.x.min(other.x), self.y.min(other.y), self.z.min(other.z))
    }

    #[must_use]
    #[inline]
    pub fn max(self, other: Self) -> Self {
        pos3(self.x.max(other.x), self.y.max(other.y), self.z.max(other.z))
    }

    #[must_use]
    #[inline]
    pub fn clamp(self, min: Self, max: Self) -> Self {
        Self {
            x: self.x.clamp(min.x, max.x),
            y: self.y.clamp(min.y, max.y),
            z: self.z.clamp(min.z, max.z),
        }
    }

    /// Linearly interpolate towards another point, so that `0.0 => self, 1.0 => other`.
    pub fn lerp(&self, other: Pos3, t: f32) -> Pos3 {
        Pos3 {
            x: egui::lerp(self.x..=other.x, t),
            y: egui::lerp(self.y..=other.y, t),
            z: egui::lerp(self.z..=other.z, t),
        }
    }

    pub fn average(positions: impl ExactSizeIterator<Item = Self>) -> Pos3 {
        let mut acc = Vec3::ZERO;
        let len = positions.len() as f32;
        for x in positions {
            acc += x.to_vec3();
        }
        Pos3::ZERO + acc / len
    }

    pub fn average_ref<'a>(positions: impl ExactSizeIterator<Item = &'a Self>) -> Pos3 {
        Self::average(positions.map(|&p| p))
    }
}

use egui::{Pos2, Vec2};
#[allow(dead_code)]
pub fn average(positions: impl ExactSizeIterator<Item = Pos2>) -> Pos2 {
    let mut acc = Vec2::ZERO;
    let len = positions.len() as f32;
    for x in positions {
        acc += x.to_vec2();
    }
    Pos2::ZERO + acc / len
}

impl std::ops::Index<usize> for Pos3 {
    type Output = f32;

    #[inline(always)]
    fn index(&self, index: usize) -> &f32 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Pos3 index out of bounds: {index}"),
        }
    }
}

impl std::ops::IndexMut<usize> for Pos3 {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Pos3 index out of bounds: {index}"),
        }
    }
}

impl Eq for Pos3 {}

impl AddAssign<Vec3> for Pos3 {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Vec3) {
        *self = Pos3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

impl SubAssign<Vec3> for Pos3 {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Vec3) {
        *self = Pos3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}

impl Add<Vec3> for Pos3 {
    type Output = Pos3;

    #[inline(always)]
    fn add(self, rhs: Vec3) -> Pos3 {
        Pos3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Pos3 {
    type Output = Vec3;

    #[inline(always)]
    fn sub(self, rhs: Pos3) -> Vec3 {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Sub<Vec3> for Pos3 {
    type Output = Pos3;

    #[inline(always)]
    fn sub(self, rhs: Vec3) -> Pos3 {
        Pos3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl std::fmt::Debug for Pos3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{:.1} {:.1} {:.1}]", self.x, self.y, self.z)
    }
}
