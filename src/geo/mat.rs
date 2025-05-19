use super::*;

#[derive(Clone, Copy, PartialEq)]
pub struct Matrix3x3 {
    pub x_row: Vec3,
    pub y_row: Vec3,
    pub z_row: Vec3,
}

impl Matrix3x3 {
    #[must_use]
    #[inline(always)]
    fn fst_col(&self) -> Vec3 {
        Vec3 {
            x: self.x_row.x,
            y: self.y_row.x,
            z: self.z_row.x,
        }
    }

    #[must_use]
    #[inline(always)]
    fn snd_col(&self) -> Vec3 {
        Vec3 {
            x: self.x_row.y,
            y: self.y_row.y,
            z: self.z_row.y,
        }
    }

    #[must_use]
    #[inline(always)]
    fn trd_col(&self) -> Vec3 {
        Vec3 {
            x: self.x_row.z,
            y: self.y_row.z,
            z: self.z_row.z,
        }
    }

    pub const IDENTITY: Self = Self {
        x_row: Vec3::X,
        y_row: Vec3::Y,
        z_row: Vec3::Z,
    };

    pub fn transposed(&self) -> Self {
        Self {
            x_row: self.fst_col(),
            y_row: self.snd_col(),
            z_row: self.trd_col(),
        }
    }

    pub fn new_rotation_from_axis_angle(axis: Vec3, angle: f32) -> Self {
        debug_assert!(axis.is_normalized());
        //taken from wikipedia: https://en.wikipedia.org/wiki/Rotation_matrix
        let cos = angle.cos();
        let sin = angle.sin();
        let one_cos = 1.0 - cos;
        let Vec3 { x: ux, y: uy, z: uz } = axis;
        Self {
            x_row: vec3(
                cos + ux * ux * one_cos,
                ux * uy * one_cos - uz * sin,
                ux * uz * one_cos + uy * sin,
            ),
            y_row: vec3(
                uy * ux * one_cos + uz * sin,
                cos + uy * uy * one_cos,
                uy * uz * one_cos - ux * sin,
            ),
            z_row: vec3(
                uz * ux * one_cos - uy * sin,
                uz * uy * one_cos + ux * sin,
                cos + uz * uz * one_cos,
            ),
        }
    }

    pub fn new_reflector(normal: Vec3) -> Self {
        debug_assert!(normal.is_normalized());
        let Vec3 { x, y, z } = normal;
        Self {
            x_row: vec3(1.0 - 2.0 * x * x, -2.0 * x * y, -2.0 * x * z),
            y_row: vec3(-2.0 * y * x, 1.0 - 2.0 * y * y, -2.0 * y * z),
            z_row: vec3(-2.0 * z * x, -2.0 * z * y, 1.0 - 2.0 * z * z),
        }
    }

    pub fn determinant(&self) -> f32 {
        let Self {
            x_row: Vec3 { x: a, y: b, z: c },
            y_row: Vec3 { x: d, y: e, z: f },
            z_row: Vec3 { x: g, y: h, z: i },
        } = *self;
        //rule of sarrus
        a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h
    }
}

impl std::ops::Mul<Vec3> for &Matrix3x3 {
    type Output = Vec3;
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

use egui::vec2;

#[derive(Clone, Copy, PartialEq)]
pub struct Matrix2x2 {
    x_row: Vec2,
    y_row: Vec2,
}

impl Matrix2x2 {
    pub const fn from_array(a: [f32; 4]) -> Self {
        Self::from_rows(vec2(a[0], a[1]), vec2(a[2], a[3]))
    }

    pub const fn from_rows(x_row: Vec2, y_row: Vec2) -> Self {
        Self { x_row, y_row }
    }

    pub const fn from_cols(col_1: Vec2, col_2: Vec2) -> Self {
        let x_row = vec2(col_1.x, col_2.x);
        let y_row = vec2(col_1.y, col_2.y);
        Self { x_row, y_row }
    }

    pub const fn to_array(self) -> [f32; 4] {
        [self.x_row.x, self.x_row.y, self.y_row.x, self.y_row.y]
    }

    pub fn determinant(&self) -> f32 {
        let [a, b, c, d] = self.to_array();
        a * d - b * c
    }

    pub fn inverse(self) -> Self {
        let [a, b, c, d] = self.to_array();
        let det = self.determinant();
        (1.0 / det) * Self::from_array([d, -b, -c, a])
    }
}

impl std::ops::Mul<Vec2> for Matrix2x2 {
    type Output = Vec2;
    #[inline(always)]
    fn mul(self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: self.x_row.dot(rhs),
            y: self.y_row.dot(rhs),
        }
    }
}

impl std::ops::Mul<Matrix2x2> for f32 {
    type Output = Matrix2x2;
    #[inline(always)]
    fn mul(self, rhs: Matrix2x2) -> Self::Output {
        let [a, b, c, d] = rhs.to_array();
        Matrix2x2::from_array([self * a, self * b, self * c, self * d])
    }
}
