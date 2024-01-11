

use super::*;

pub struct Matrix3x3 {
    pub x_row: Vec3,
    pub y_row: Vec3,
    pub z_row: Vec3,
}

impl Matrix3x3 {
    #[must_use]
    #[inline(always)]
    fn fst_col(&self) -> Vec3 {
        Vec3 { x: self.x_row.x, y: self.y_row.x, z: self.z_row.x }
    }

    #[must_use]
    #[inline(always)]
    fn snd_col(&self) -> Vec3 {
        Vec3 { x: self.x_row.y, y: self.y_row.y, z: self.z_row.y }
    }

    #[must_use]
    #[inline(always)]
    fn trd_col(&self) -> Vec3 {
        Vec3 { x: self.x_row.z, y: self.y_row.z, z: self.z_row.z }
    }

    pub const IDENTITY: Self = Self {
        x_row: Vec3::X,
        y_row: Vec3::Y,
        z_row: Vec3::Z,
    };

    pub fn new_rotation_from_axis_angle(axis: Vec3, angle: f32) -> Self {
        debug_assert!(axis.is_normalized());
        //taken from wikipedia: https://en.wikipedia.org/wiki/Rotation_matrix
        let cos = angle.cos();
        let sin = angle.sin();
        let one_cos = 1.0 - cos;
        let Vec3 { x: ux, y: uy, z: uz } = axis;
        Self {
            x_row: vec3(cos + ux * ux * one_cos,        ux * uy * one_cos - uz * sin,   ux * uz * one_cos + uy * sin),
            y_row: vec3(uy * ux * one_cos + uz * sin,   cos + uy * uy * one_cos,        uy * uz * one_cos - ux * sin),
            z_row: vec3(uz * ux * one_cos - uy * sin,   uz * uy * one_cos + ux * sin,   cos + uz * uz * one_cos),
        }
    }

    pub fn new_reflector(normal: Vec3) -> Self {
        debug_assert!(normal.is_normalized());
        let Vec3 { x, y, z } = normal;
        Self {
            x_row: vec3(1.0 - 2.0 * x * x,       -2.0 * x * y,       -2.0 * x * z), 
            y_row: vec3(     -2.0 * y * x,  1.0 - 2.0 * y * y,       -2.0 * y * z),
            z_row: vec3(     -2.0 * z * x,       -2.0 * z * y,  1.0 - 2.0 * z * z),
        }
    }

    pub fn determinant(&self) -> f32 {
        let Self { 
            x_row: Vec3 { x: a, y: b, z: c }, 
            y_row: Vec3 { x: d, y: e, z: f }, 
            z_row: Vec3 { x: g, y: h, z: i }, 
        } = *self;
        //rule of sarrus
        a * e * i 
        + b * f * g 
        + c * d * h 
        - c * e * g 
        - b * d * i 
        - a * f * h
    }
}

impl std::ops::Mul<Vec3> for &Matrix3x3 {
    type Output = Vec3;
    #[must_use]
    #[inline(always)]
    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x_row.dot(rhs),
            y: self.y_row.dot(rhs),
            z: self.z_row.dot(rhs),
        }
    }
}

impl std::ops::Mul<&Matrix3x3> for &Matrix3x3 {
    type Output = Matrix3x3;
    #[must_use]
    #[inline(always)]
    fn mul(self, rhs: &Matrix3x3) -> Self::Output {
        Matrix3x3 {
            x_row: self * rhs.fst_col(),
            y_row: self * rhs.snd_col(),
            z_row: self * rhs.trd_col(),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn matrix_vec_prod() {
        let mat = Matrix3x3 {
            x_row: vec3(1.0, 2.0, 3.0),
            y_row: vec3(4.0, 5.0, 6.0),
            z_row: vec3(9.0, 8.0, 7.0),
        };

        let vec = vec3(100.0, 1000.0, 10.0);
        assert_eq!(&mat * vec, vec3(2130.0, 5460.0, 8970.0));
    }
}
