use itertools::iproduct;

use super::{grid::*, *};

pub trait Rotation<const COUNT: usize>
where
    Self: Sized + Copy,
{
    fn rotate_forward(self, x: isize, y: isize) -> (isize, isize);

    fn rotate_backward(self, x: isize, y: isize) -> (isize, isize);

    const ALL: [Self; COUNT];
}

/// there are 6 possible turns of triangulated torus embedded in [0, 1]^2.
/// some of them are a bit weird, because the embedding is not ideal for triangle symmetry.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub enum Turn6 {
    None,
    /// not actually just a turn when embedded in [0, 1]^2:
    /// for the continuous torus, turning any angle is an automorphism.
    /// this one maps the x-axis of the embedding to x = y (stretched by sqrt 2)
    /// while x = y is mapped to the y-axis (shrunk by sqrt 2)
    ///
    /// we call this transformation S. as stated, S maps (1, 0) to (1, 1) and (1, 1) to (0, 1).
    /// -> S is matrix [1, -1; 1, 0] (written row-wise)
    /// note: inverse of S is [0, 1; -1, 1].
    Skewed60,
    /// apply Skewed60 twice
    Skewed120,
    /// apply Skewed60 three times
    Half,
    /// apply Skewed60 four times
    Skewed240,
    /// apply Skewed60 five times
    Skewed300,
}

impl Rotation<6> for Turn6 {
    #[inline(always)]
    fn rotate_forward(self, x: isize, y: isize) -> (isize, isize) {
        match self {
            Turn6::None => (x, y),            // apply S^0
            Turn6::Skewed60 => (x - y, x),    // apply S^1
            Turn6::Skewed120 => (-y, x - y),  // apply S^2
            Turn6::Half => (-x, -y),          // apply S^3
            Turn6::Skewed240 => (-x + y, -x), // apply S^4
            Turn6::Skewed300 => (y, -x + y),  // apply S^5
        }
    }

    #[inline(always)]
    fn rotate_backward(self, x: isize, y: isize) -> (isize, isize) {
        match self {
            Turn6::None => (x, y),            // apply S^-0 = S^0
            Turn6::Skewed60 => (y, -x + y),   // apply S^-1 = S^5
            Turn6::Skewed120 => (-x + y, -x), // apply S^-2 = S^4
            Turn6::Half => (-x, -y),          // apply S^-3 = S^3
            Turn6::Skewed240 => (-y, x - y),  // apply S^-4 = S^2
            Turn6::Skewed300 => (x - y, x),   // apply S^-5 = S^1
        }
    }

    const ALL: [Self; 6] = [
        Turn6::None,
        Turn6::Skewed60,
        Turn6::Skewed120,
        Turn6::Half,
        Turn6::Skewed240,
        Turn6::Skewed300,
    ];
}

/// there are 4 possible turns of torus with squares embedded in [0, 1]^2.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub enum Turn4 {
    None,
    Rot90,
    Rot180,
    Rot240,
}

impl Rotation<4> for Turn4 {
    #[inline(always)]
    fn rotate_forward(self, x: isize, y: isize) -> (isize, isize) {
        match self {
            Turn4::None => (x, y),
            Turn4::Rot90 => (-y, x),
            Turn4::Rot180 => (-x, -y),
            Turn4::Rot240 => (y, -x),
        }
    }

    #[inline(always)]
    fn rotate_backward(self, x: isize, y: isize) -> (isize, isize) {
        match self {
            Turn4::None => (x, y),
            Turn4::Rot90 => (y, -x),
            Turn4::Rot180 => (-x, -y),
            Turn4::Rot240 => (-y, x),
        }
    }

    const ALL: [Self; 4] = [Turn4::None, Turn4::Rot90, Turn4::Rot180, Turn4::Rot240];
}

const BOOL: [bool; 2] = [false, true];

#[derive(Serialize, Deserialize, Clone)]
pub struct TorusAutomorphism<const N: usize, R: Rotation<N>> {
    colwise: OrderedColWise,
    new_origin: Coords,
    turn: R,
    flip: bool,
}
pub type TorusAutomorphism6 = TorusAutomorphism<6, Turn6>;
pub type TorusAutomorphism4 = TorusAutomorphism<4, Turn4>;

impl<const N: usize, R: Rotation<N>> Automorphism for TorusAutomorphism<N, R> {
    fn apply_forward(&self, v: usize) -> usize {
        let Coords { mut x, mut y } = self.colwise.coordinates_of(v);

        x -= self.new_origin.x;
        y -= self.new_origin.y;
        (x, y) = self.turn.rotate_forward(x, y);
        if self.flip {
            std::mem::swap(&mut x, &mut y);
        }

        let new_coords = self.colwise.pack_small_coordinates(x, y);
        self.colwise.index_of(new_coords)
    }

    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        (0..self.nr_vertices()).map(|v| self.apply_forward(v))
    }

    fn apply_backward(&self, v: usize) -> usize {
        let Coords { mut x, mut y } = self.colwise.coordinates_of(v);

        // reversed order of apply_forward
        if self.flip {
            std::mem::swap(&mut x, &mut y);
        }
        (x, y) = self.turn.rotate_backward(x, y);
        x += self.new_origin.x;
        y += self.new_origin.y;

        let new_coords = self.colwise.pack_small_coordinates(x, y);
        self.colwise.index_of(new_coords)
    }

    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        (0..self.nr_vertices()).map(|v| self.apply_backward(v))
    }

    fn nr_vertices(&self) -> usize {
        self.colwise.nr_vertices()
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct TorusSymmetry<const N: usize, R: Rotation<N>> {
    nr_vertices: usize,
    autos: Vec<TorusAutomorphism<N, R>>,
}
pub type TorusSymmetry6 = TorusSymmetry<6, Turn6>;
pub type TorusSymmetry4 = TorusSymmetry<4, Turn4>;

impl<const N: usize, R: Rotation<N>> TorusSymmetry<N, R> {
    pub fn new(nr_vertices: usize) -> Self {
        //counts how many vertices are in one row / col
        let len = (nr_vertices as f32).sqrt().round() as usize;
        debug_assert_eq!(len * len, nr_vertices);

        let colwise = OrderedColWise::new(len);
        let autos = iproduct!(0..nr_vertices, R::ALL, BOOL)
            .map(|(v, turn, flip)| {
                let new_origin = colwise.coordinates_of(v);
                TorusAutomorphism {
                    colwise,
                    new_origin,
                    turn,
                    flip,
                }
            })
            .collect_vec();
        Self { nr_vertices, autos }
    }

    fn autos_of(&self, v: usize) -> &[TorusAutomorphism<N, R>] {
        debug_assert_eq!(self.nr_vertices * N * 2, self.autos.len());
        let start = v * N * 2;
        let end = start + N * 2;
        &self.autos[start..end]
    }
}

impl SymmetryGroup for TorusSymmetry6 {
    type Auto = TorusAutomorphism6;

    fn all_automorphisms(&self) -> &[Self::Auto] {
        &self.autos
    }

    fn repr_automorphisms(&self, v: usize) -> impl Iterator<Item = &Self::Auto> + '_ + Clone {
        self.autos_of(v).iter()
    }

    fn repr_of(&self, _: usize) -> usize {
        0
    }

    fn class_representatives(&self) -> impl Iterator<Item = usize> + '_ + Clone {
        std::iter::once(0)
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::Torus6(self)
    }

    fn nr_vertices(&self) -> usize {
        self.nr_vertices
    }
}

impl SymmetryGroup for TorusSymmetry4 {
    type Auto = TorusAutomorphism4;

    fn all_automorphisms(&self) -> &[Self::Auto] {
        &self.autos
    }

    fn repr_automorphisms(&self, v: usize) -> impl Iterator<Item = &Self::Auto> + '_ + Clone {
        self.autos_of(v).iter()
    }

    fn repr_of(&self, _: usize) -> usize {
        0
    }

    fn class_representatives(&self) -> impl Iterator<Item = usize> + '_ + Clone {
        std::iter::once(0)
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::Torus4(self)
    }

    fn nr_vertices(&self) -> usize {
        self.nr_vertices
    }
}
