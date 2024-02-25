use egui::*;

use itertools::izip;
use smallvec::SmallVec;

use crate::geo::{Matrix2x2, Pos3};

use super::*;

const TORUS_EXTREMES_2D: [Pos2; 4] = {
    let orig = &Embedding3D::TRIANGLE_TORUS_EXTREMES;
    assert!(orig.len() == 4);
    assert!(orig[0].z == Z_OFFSET_2D);
    assert!(orig[1].z == Z_OFFSET_2D);
    assert!(orig[2].z == Z_OFFSET_2D);
    assert!(orig[3].z == Z_OFFSET_2D);

    [orig[0].xy(), orig[1].xy(), orig[2].xy(), orig[3].xy()]
};
const NEW_ORIGIN: Pos2 = TORUS_EXTREMES_2D[0];

/// transforms unit square [0, 1]^2 to parallelogram with extremes `TORUS_EXTREMES_2D - NEW_ORIGIN`,
fn skew_unit_square() -> Matrix2x2 {
    let col_1 = TORUS_EXTREMES_2D[1] - NEW_ORIGIN;
    let col_2 = TORUS_EXTREMES_2D[3] - NEW_ORIGIN;
    Matrix2x2::from_array([col_1.x, col_2.x, col_1.y, col_2.y])
}

/// transforms parallelogram `TORUS_EXTREMES_2D` with `TORUS_EXTREMES_2D[0]` shifted into origin
/// to unit square [0, 1]^2
fn unskew_unit_square() -> Matrix2x2 {
    skew_unit_square().inverse()
}

/// transforms [`TORUS_EXTREMES_2D`] to unit square [0, 1]^2
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
struct PosInSquare(Vec2);

impl From<Pos2> for PosInSquare {
    fn from(value: Pos2) -> Self {
        let shifted = value - NEW_ORIGIN;
        PosInSquare(unskew_unit_square() * shifted)
    }
}

impl From<PosInSquare> for Pos2 {
    fn from(PosInSquare(val): PosInSquare) -> Self {
        let skewed = skew_unit_square() * val;
        NEW_ORIGIN + skewed
    }
}

mod order_colwise {
    use super::*;

    /// vertex index but ordered row-wise when torus embedding is mapped to unit square
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    struct InSquare(usize);

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

    /// for a given triangle torus, the vertices are reordered such that
    /// their positions transformed to [`PosInSquare`] coordinates are ordered
    /// first by `.x`, then by `.y` value.
    /// for fast 2d iteration thus choose `x` in outher loop, `y` in inner loop.
    #[derive(Serialize, Deserialize, Clone)]
    pub struct OrderColWise {
        /// square root of number of vertices, nr of vertices per row / column in unit square
        len: usize,
        to_ordered: Vec<InSquare>,
        from_ordered: Vec<usize>,
    }

    impl OrderColWise {
        pub fn side_len(&self) -> isize {
            self.len as isize
        }

        pub fn nr_vertices(&self) -> usize {
            self.len * self.len
        }

        pub fn test_edges(&self, torus_edges: &EdgeList) -> bool {
            let mut neighs_from_pos = Vec::new();
            let mut neighs_from_edges = Vec::new();
            for x in 0..self.side_len() {
                for y in 0..self.side_len() {
                    let coords = SquareCoords { x, y };
                    let original_i = self.from_ordered_coordinates(coords);
                    neighs_from_pos.extend(
                        [
                            (x - 1, y),
                            (x + 1, y),
                            (x, y - 1),
                            (x, y + 1),
                            (x - 1, y - 1),
                            (x + 1, y + 1),
                        ]
                        .into_iter()
                        .map(|(neigh_x, neigh_y)| {
                            let neigh_coords = self.pack_coordinates(neigh_x, neigh_y);
                            self.from_ordered_coordinates(neigh_coords)
                        }),
                    );
                    neighs_from_edges.extend(torus_edges.neighbors_of(original_i));

                    neighs_from_pos.sort();
                    neighs_from_pos.dedup();
                    neighs_from_edges.sort();

                    assert_eq!(neighs_from_pos, neighs_from_edges);
                    neighs_from_pos.clear();
                    neighs_from_edges.clear();
                }
            }
            true
        }

        pub fn new(torus_edges: &EdgeList, torus_positions: &[Pos3]) -> Self {
            let nr_vertices = torus_edges.nr_vertices();
            debug_assert_eq!(nr_vertices, torus_positions.len());

            let mut to_ordered = vec![InSquare(usize::MAX); nr_vertices];
            let mut from_ordered = vec![usize::MAX; nr_vertices];

            let f_len = (nr_vertices as f32).sqrt();
            let len = f_len.round() as usize; //counts how many vertices are in one row / col
            debug_assert_eq!(len * len, nr_vertices);
            let max_coord = f_len - 1.0; //min coord == 0 -> n-1 later is max coord (for both x and y)

            for (original_i, pos) in izip!(0.., torus_positions) {
                debug_assert_eq!(pos.z, Z_OFFSET_2D);
                let square_pos: PosInSquare = pos.xy().into();
                let x_index = (square_pos.0.x * max_coord).round() as usize;
                let y_index = (square_pos.0.y * max_coord).round() as usize;
                let ordered_i = x_index * len + y_index;

                debug_assert_eq!(to_ordered[original_i], InSquare(usize::MAX));
                debug_assert_eq!(from_ordered[ordered_i], usize::MAX);
                to_ordered[original_i] = InSquare(ordered_i);
                from_ordered[ordered_i] = original_i;
            }
            let res = Self { len, to_ordered, from_ordered };
            debug_assert!(res.test_edges(torus_edges));
            res
        }

        pub fn pack_coordinates(&self, x: isize, y: isize) -> SquareCoords {
            let ilen = self.len as isize;
            debug_assert!(x >= -ilen);
            debug_assert!(y >= -ilen);
            let x = (x + ilen) % ilen;
            let y = (y + ilen) % ilen;
            SquareCoords { x, y }
        }

        pub fn to_ordered_coordinates(&self, v: usize) -> SquareCoords {
            let InSquare(ord) = self.to_ordered[v];
            let x = (ord / self.len) as isize;
            let y = (ord % self.len) as isize;
            SquareCoords { x, y }
        }

        pub fn from_ordered_coordinates(&self, coords: SquareCoords) -> usize {
            let index = coords.x * (self.len as isize) + coords.y;
            self.from_ordered[index as usize]
        }
    }
}
use order_colwise::*;

#[derive(Clone, Copy, Serialize, Deserialize)]
struct AutoData {
    new_origin: SquareCoords,
    turn: bool,
    flip: bool,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct TorusAutomorphism {
    colwise: OrderColWise,
    data: std::cell::Cell<AutoData>,
}

impl TorusAutomorphism {
    fn new(colwise: OrderColWise) -> Self {
        let data = AutoData {
            new_origin: colwise.pack_coordinates(0, 0).into(),
            turn: false,
            flip: false,
        }
        .into();
        Self { colwise, data }
    }

    fn change_data_to(&self, new_data: AutoData) {
        self.data.set(new_data);
    }

    fn change_to(&self, new_origin: usize, turn: bool, flip: bool) {
        let new_origin = self.colwise.to_ordered_coordinates(new_origin);
        self.data.set(AutoData { new_origin, turn, flip });
    }
}

impl Automorphism for TorusAutomorphism {
    fn apply_forward(&self, v: usize) -> usize {
        let old_coords = self.colwise.to_ordered_coordinates(v);
        let mut new_x = old_coords.x();
        let mut new_y = old_coords.y();
        let data = self.data.get();
        if data.turn {
            new_x = self.colwise.side_len() - new_x;
            new_y = self.colwise.side_len() - new_y;
        }
        if data.flip {
            std::mem::swap(&mut new_x, &mut new_y);
        }
        new_x -= data.new_origin.x();
        new_y -= data.new_origin.y();
        let new_coords = self.colwise.pack_coordinates(new_x, new_y);
        self.colwise.from_ordered_coordinates(new_coords)
    }

    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        (0..self.colwise.nr_vertices()).map(|v| self.apply_forward(v))
    }

    fn apply_backward(&self, v: usize) -> usize {
        let old_coords = self.colwise.to_ordered_coordinates(v);
        let mut new_x = old_coords.x();
        let mut new_y = old_coords.y();
        let data = self.data.get();
        if data.turn {
            new_x = self.colwise.side_len() - new_x;
            new_y = self.colwise.side_len() - new_y;
        }
        if data.flip {
            std::mem::swap(&mut new_x, &mut new_y);
        }
        new_x += data.new_origin.x();
        new_y += data.new_origin.y();
        let new_coords = self.colwise.pack_coordinates(new_x, new_y);
        self.colwise.from_ordered_coordinates(new_coords)
    }

    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        (0..self.colwise.nr_vertices()).map(|v| self.apply_backward(v))
    }

    fn nr_vertices(&self) -> usize {
        self.colwise.nr_vertices()
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct TorusSymmetry {
    auto: TorusAutomorphism,
}

impl TorusSymmetry {
    pub fn new(torus_edges: &EdgeList, torus_positions: &[Pos3]) -> Self {
        let colwise = OrderColWise::new(torus_edges, torus_positions);
        let auto = TorusAutomorphism::new(colwise);
        Self { auto }
    }
}

#[derive(Clone)]
pub struct TorusSymmetryIter<'a> {
    data: SmallVec<[AutoData; 4]>,
    auto: &'a TorusAutomorphism,
    i: usize,
}

impl<'a> Iterator for TorusSymmetryIter<'a> {
    type Item = &'a TorusAutomorphism;
    fn next(&mut self) -> Option<Self::Item> {
        if self.i < self.data.len() {
            self.auto.change_data_to(self.data[self.i]);
            self.i += 1;
            Some(&self.auto)
        } else {
            None
        }
    }
}

impl SymmetryGroup for TorusSymmetry {
    type Auto = TorusAutomorphism;
    type AutoIter<'a> = TorusSymmetryIter<'a>;

    fn all_automorphisms(&self) -> impl Iterator<Item = &Self::Auto> {
        (0..(4 * self.auto.nr_vertices())).map(|i| {
            let v = i / 4;
            let flip1 = matches!(i % 4, 2 | 3);
            let flip2 = matches!(i % 4, 1 | 2);
            self.auto.change_to(v, flip1, flip2);
            &self.auto
        })
    }

    fn class_representatives(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        std::iter::once(0)
    }

    fn to_representative<'a>(&'a self, cops: &mut [usize]) -> Self::AutoIter<'_> {
        //each configuration gets a value. the only important thing is that
        //config_hash_value is injective, because we choose the configuration with lowest value.
        //if multiple transformations yield the same (best) configuration, 
        //we have to return all these best transformations at once.
        let nr_vertices = self.auto.nr_vertices();
        let config_hash_value = |rotated: &[_]| {
            let mut acc = 0;
            for &c in rotated.iter() {
                debug_assert!(c < nr_vertices);
                acc += c;
                acc *= nr_vertices;
            }
            acc
        };
        let mut best_autos = SmallVec::<[AutoData; 4]>::new();
        let mut best_val = usize::MAX;
        const BOOL: [bool; 2] = [false, true];
        for (&cop, turn, flip) in itertools::iproduct!(cops.iter(), BOOL, BOOL) {
            self.auto.change_to(cop, turn, flip);                
            let mut rotated = [0usize; bruteforce::MAX_COPS];
            let rotated = &mut rotated[..cops.len()];
            for (rc, &c) in izip!(rotated.iter_mut(), cops.iter()) {
                *rc = self.auto.apply_forward(c);
            }
            rotated.sort();                
            let new_val = config_hash_value(rotated);
            if new_val == best_val {
                best_autos.push(self.auto.data.get());
            }
            if new_val < best_val {
                best_val = new_val;
                best_autos.clear();
                best_autos.push(self.auto.data.get());
            }
        }
        assert!(!best_autos.is_empty());
        self.auto.change_data_to(best_autos[0]);
        for c in cops.iter_mut() {
            *c = self.auto.apply_forward(*c);
        }
        cops.sort();
        
        TorusSymmetryIter {
            data: best_autos,
            auto: &self.auto,
            i: 0,
        }
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::Torus(self)
    }
}

#[cfg(test)]
mod test {
    use itertools::Itertools;

    use super::*;

    #[test]
    fn map_extremes() {
        let orig = &TORUS_EXTREMES_2D;
        let unit_square: Vec<PosInSquare> = orig.iter().copied().map_into().collect_vec();
        assert!((unit_square[0].0 - vec2(0.0, 0.0)).length() < 1e-4);
        assert!((unit_square[1].0 - vec2(1.0, 0.0)).length() < 1e-4);
        assert!((unit_square[2].0 - vec2(1.0, 1.0)).length() < 1e-4);
        assert!((unit_square[3].0 - vec2(0.0, 1.0)).length() < 1e-4);

        let undone: Vec<Pos2> = unit_square.iter().copied().map_into().collect_vec();
        assert!((undone[0] - orig[0]).length() < 1e-4);
        assert!((undone[1] - orig[1]).length() < 1e-4);
        assert!((undone[2] - orig[2]).length() < 1e-4);
        assert!((undone[3] - orig[3]).length() < 1e-4);
    }

    #[test]
    fn build_colwise() {
        for resolution in [0, 2, 10, 30, 64, 101] {
            let torus = Embedding3D::new_subdivided_triangle_torus(resolution);
            let colwise = OrderColWise::new(torus.edges(), torus.positions());
            colwise.test_edges(torus.edges());
        }
    }
}
