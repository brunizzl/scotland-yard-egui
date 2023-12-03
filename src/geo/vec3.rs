use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

use super::pos3::*;
use egui::vec2;

/// copy of egui's Vec2, but with one extra dimension
#[repr(C)]
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[inline(always)]
pub const fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

// ----------------------------------------------------------------------------

impl Vec3 {
    pub const X: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Self = Self { x: 0.0, y: 0.0, z: 1.0 };

    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    pub const INFINITY: Self = Self::splat(f32::INFINITY);

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// set x, y, z to the given value
    #[inline(always)]
    pub const fn splat(v: f32) -> Self {
        Self { x: v, y: v, z: v }
    }

    /// Treat this vector as a position.
    #[inline(always)]
    pub fn to_pos3(self) -> Pos3 {
        pos3(self.x, self.y, self.z)
    }

    /// Safe normalize: returns zero if input is zero.
    #[must_use]
    #[inline(always)]
    pub fn normalized(self) -> Self {
        let len = self.length();
        if len <= 0.0 {
            self
        } else {
            self / len
        }
    }

    #[inline(always)]
    pub fn length(self) -> f32 {
        self.x.hypot(self.y).hypot(self.z)
    }

    #[inline(always)]
    pub fn length_sq(self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// angle in spherical coordinates in radiants
    /// first is latitude (how far from equator) in -pi/2 (south pole) to pi/2 (north pole)
    /// second is longitude (how far on equator) from -pi to pi
    /// this assumes the x-y plane to intersect the sphere in the equator (positive x-axis has longitude 0) 
    /// and the z-axis to go through the poles (positive z is northern hemisphere)
    #[inline(always)]
    pub fn angle(self) -> (f32, f32) {
        let xy_plane_len = vec2(self.x, self.y).length();
        let latitude = xy_plane_len.atan2(self.z);
        let longitude = self.y.atan2(self.x);
        (latitude, longitude)
    }

    #[must_use]
    #[inline]
    pub fn abs(self) -> Self {
        vec3(self.x.abs(), self.y.abs(), self.z.abs())
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
        vec3(self.x.min(other.x), self.y.min(other.y), self.z.min(other.z))
    }

    #[must_use]
    #[inline]
    pub fn max(self, other: Self) -> Self {
        vec3(self.x.max(other.x), self.y.max(other.y), self.z.max(other.z))
    }

    /// The dot-product of two vectors.
    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Returns the minimum of `self.x` and `self.y`.
    #[must_use]
    #[inline(always)]
    pub fn min_elem(self) -> f32 {
        self.x.min(self.y).min(self.z)
    }

    /// Returns the maximum of `self.x` and `self.y`.
    #[inline(always)]
    #[must_use]
    pub fn max_elem(self) -> f32 {
        self.x.max(self.y).max(self.z)
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
}

impl std::ops::Index<usize> for Vec3 {
    type Output = f32;

    #[inline(always)]
    fn index(&self, index: usize) -> &f32 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Vec3 index out of bounds: {index}"),
        }
    }
}

impl std::ops::IndexMut<usize> for Vec3 {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Vec3 index out of bounds: {index}"),
        }
    }
}

impl Eq for Vec3 {}

impl Neg for Vec3 {
    type Output = Vec3;

    #[inline(always)]
    fn neg(self) -> Vec3 {
        vec3(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Vec3 {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Vec3) {
        *self = Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

impl SubAssign for Vec3 {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Vec3) {
        *self = Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}

impl Add for Vec3 {
    type Output = Vec3;

    #[inline(always)]
    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Vec3 {
    type Output = Vec3;

    #[inline(always)]
    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl MulAssign<f32> for Vec3 {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;

    #[inline(always)]
    fn mul(self, factor: f32) -> Vec3 {
        Vec3 {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;

    #[inline(always)]
    fn mul(self, vec: Vec3) -> Vec3 {
        Vec3 {
            x: self * vec.x,
            y: self * vec.y,
            z: self * vec.z,
        }
    }
}

impl Div<f32> for Vec3 {
    type Output = Vec3;

    #[inline(always)]
    fn div(self, factor: f32) -> Vec3 {
        Vec3 {
            x: self.x / factor,
            y: self.y / factor,
            z: self.z / factor,
        }
    }
}

impl std::fmt::Debug for Vec3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{:.1} {:.1} {:.1}]", self.x, self.y, self.z)
    }
}
