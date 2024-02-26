use itertools::{iproduct, izip};
use smallvec::SmallVec;

use super::*;

mod ordered_colwise {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct SquareCoords {
        x: isize,
        y: isize,
    }

    impl SquareCoords {
        pub fn x(self) -> isize {
            self.x
        }
        pub fn y(self) -> isize {
            self.y
        }
    }

    #[derive(Serialize, Deserialize, Clone, Copy)]
    pub struct OrderedColWise {
        /// square root of number of vertices, nr of vertices per row / column in unit square
        len: usize,
    }

    impl OrderedColWise {
        pub fn new(len: usize) -> Self {
            Self { len }
        }

        pub fn nr_vertices(&self) -> usize {
            self.len * self.len
        }

        pub fn pack_coordinates(&self, x: isize, y: isize) -> SquareCoords {
            let ilen = self.len as isize;
            let quad_ilen = 4 * ilen;
            debug_assert!(x >= -quad_ilen);
            debug_assert!(y >= -quad_ilen);
            let x = (x + quad_ilen) % ilen;
            let y = (y + quad_ilen) % ilen;
            SquareCoords { x, y }
        }

        pub fn to_ordered_coordinates(&self, v: usize) -> SquareCoords {
            let x = (v / self.len) as isize;
            let y = (v % self.len) as isize;
            SquareCoords { x, y }
        }

        pub fn from_ordered_coordinates(&self, coords: SquareCoords) -> usize {
            let v = coords.x * (self.len as isize) + coords.y;
            v as usize
        }
    }
}
use ordered_colwise::*;

/// there are 6 possible turns of triangulated torus embedded in [0, 1]^2.
/// some of them are a bit weird, because the embedding is not ideal for triangle symmetry.
#[derive(Clone, Copy, Serialize, Deserialize)]
enum Turn3 {
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

const TURN: [Turn3; 6] = [
    Turn3::None,
    Turn3::Skewed60,
    Turn3::Skewed120,
    Turn3::Half,
    Turn3::Skewed240,
    Turn3::Skewed300,
];
const BOOL: [bool; 2] = [false, true];

#[derive(Serialize, Deserialize, Clone)]
pub struct TorusAutomorphism {
    colwise: OrderedColWise,
    new_origin: SquareCoords,
    turn: Turn3,
    flip: bool,
}

impl Automorphism for TorusAutomorphism {
    fn apply_forward(&self, v: usize) -> usize {
        let old_coords = self.colwise.to_ordered_coordinates(v);
        let mut x = old_coords.x();
        let mut y = old_coords.y();

        x -= self.new_origin.x();
        y -= self.new_origin.y();
        (x, y) = match self.turn {
            Turn3::None => (x, y),            // apply S^0
            Turn3::Skewed60 => (x - y, x),    // apply S^1
            Turn3::Skewed120 => (-y, x - y),  // apply S^2
            Turn3::Half => (-x, -y),          // apply S^3
            Turn3::Skewed240 => (-x + y, -x), // apply S^4
            Turn3::Skewed300 => (y, -x + y),  // apply S^5
        };
        if self.flip {
            std::mem::swap(&mut x, &mut y);
        }

        let new_coords = self.colwise.pack_coordinates(x, y);
        self.colwise.from_ordered_coordinates(new_coords)
    }

    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        (0..self.nr_vertices()).map(|v| self.apply_forward(v))
    }

    fn apply_backward(&self, v: usize) -> usize {
        let old_coords = self.colwise.to_ordered_coordinates(v);
        let mut x = old_coords.x();
        let mut y = old_coords.y();

        // reversed order of apply_forward
        if self.flip {
            std::mem::swap(&mut x, &mut y);
        }
        (x, y) = match self.turn {
            Turn3::None => (x, y),            // apply S^-0 = S^0
            Turn3::Skewed60 => (y, -x + y),   // apply S^-1 = S^5
            Turn3::Skewed120 => (-x + y, -x), // apply S^-2 = S^4
            Turn3::Half => (-x, -y),          // apply S^-3 = S^3
            Turn3::Skewed240 => (-y, x - y),  // apply S^-4 = S^2
            Turn3::Skewed300 => (x - y, x),   // apply S^-5 = S^1
        };
        x += self.new_origin.x();
        y += self.new_origin.y();

        let new_coords = self.colwise.pack_coordinates(x, y);
        self.colwise.from_ordered_coordinates(new_coords)
    }

    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        (0..self.nr_vertices()).map(|v| self.apply_backward(v))
    }

    fn nr_vertices(&self) -> usize {
        self.colwise.nr_vertices()
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct TorusSymmetry {
    nr_vertices: usize,
    autos: Vec<TorusAutomorphism>,
}

impl TorusSymmetry {
    pub fn new(nr_vertices: usize) -> Self {
        //counts how many vertices are in one row / col
        let len = (nr_vertices as f32).sqrt().round() as usize;
        debug_assert_eq!(len * len, nr_vertices);

        let colwise = OrderedColWise::new(len);
        let autos = iproduct!(0..nr_vertices, TURN, BOOL)
            .map(|(v, turn, flip)| {
                let new_origin = colwise.to_ordered_coordinates(v);
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

    fn autos_of(&self, v: usize) -> &[TorusAutomorphism] {
        debug_assert_eq!(self.nr_vertices * 12, self.autos.len());
        let start = v * 12;
        let end = start + 12;
        &self.autos[start..end]
    }
}

impl SymmetryGroup for TorusSymmetry {
    type Auto = TorusAutomorphism;
    type AutoIter<'a> = SmallVec<[&'a TorusAutomorphism; 4]>;

    fn all_automorphisms(&self) -> std::slice::Iter<'_, Self::Auto> {
        self.autos.iter()
    }

    fn class_representatives(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        std::iter::once(0)
    }

    fn to_representative<'a>(&'a self, cops: &mut [usize]) -> Self::AutoIter<'_> {
        if cops.is_empty() {
            //the first entry in self.autos is always the identity
            debug_assert!(self.autos[0]
                .forward()
                .enumerate()
                .take(20)
                .all(|(i, j)| i == j));
            return smallvec![&self.autos[0]];
        }
        //each configuration gets a value. the only important thing is that
        //config_hash_value is injective, because we choose the configuration with lowest value.
        //if multiple transformations yield the same (best) configuration,
        //we have to return all these best transformations at once.
        let config_hash_value = |rotated: &[_]| {
            let mut acc = 0;
            for &c in rotated.iter() {
                debug_assert!(c < self.nr_vertices);
                acc += c;
                acc *= self.nr_vertices;
            }
            acc
        };
        let mut best_autos = SmallVec::<[&TorusAutomorphism; 4]>::new();
        let mut best_val = usize::MAX;
        for &cop in cops.iter() {
            for auto in self.autos_of(cop) {
                let mut rotated = [0usize; bruteforce::MAX_COPS];
                let rotated = &mut rotated[..cops.len()];
                for (rc, &c) in izip!(rotated.iter_mut(), cops.iter()) {
                    *rc = auto.apply_forward(c);
                }
                rotated.sort();
                debug_assert_eq!(rotated[0], 0);
                let new_val = config_hash_value(rotated);
                if new_val == best_val {
                    best_autos.push(auto);
                }
                if new_val < best_val {
                    best_val = new_val;
                    best_autos.clear();
                    best_autos.push(auto);
                }
            }
        }
        assert!(!best_autos.is_empty());
        for c in cops.iter_mut() {
            *c = best_autos[0].apply_forward(*c);
        }
        cops.sort();

        best_autos
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::Torus(self)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn autos() {
        let res = 8; //8 divisions -> original edge now has len 10 -> 100 vertices
        let torus = Embedding3D::new_subdivided_triangle_torus(res);
        assert_eq!(torus.nr_vertices(), 100);
        let sym = TorusSymmetry::new(100);
        let mut mapped_neighs = Vec::new();
        let mut neighs_mapped = Vec::new();
        for auto in sym.all_automorphisms() {
            let fw = |v| auto.apply_forward(v);
            let bw = |v| auto.apply_backward(v);
            for v in 0..torus.nr_vertices() {
                let mapped = fw(v);
                mapped_neighs.clear();
                mapped_neighs.extend(torus.edges().neighbors_of(mapped));
                mapped_neighs.sort();

                neighs_mapped.clear();
                neighs_mapped.extend(torus.edges().neighbors_of(v).map(fw));
                neighs_mapped.sort();

                assert_eq!(mapped_neighs, neighs_mapped);
                assert_eq!(bw(mapped), v);
            }
        }
    }
}
