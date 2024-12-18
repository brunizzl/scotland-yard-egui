use serde::{Deserialize, Serialize};

use super::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Coords {
    pub x: isize,
    pub y: isize,
}

impl Coords {
    /// only valid on [`Norm::Hex`], SAME COORDINATE SYSTEM AS THESIS
    pub const fn line_e1_index(&self) -> isize {
        -self.y
    }

    /// only valid on [`Norm::Hex`], SAME COORDINATE SYSTEM AS THESIS
    pub const fn line_e2_index(&self) -> isize {
        -self.x
    }

    /// only valid on [`Norm::Hex`], SAME COORDINATE SYSTEM AS THESIS
    pub const fn line_e3_index(&self) -> isize {
        -self.x + self.y
    }

    /// these three coordinates are not line coordinates.
    /// instead they say how many steps in each direction one needs to
    /// take on a shortest path from the origin to self.
    /// the returned value is (e1, e2, e3) in non-thesis coorinates.
    pub const fn shortest_path_dirs(self, norm: Norm) -> (isize, isize, isize) {
        let Coords { x, y } = self;
        match norm {
            Norm::Hex => {
                // again: the coordinate system in use here has
                // the y-axis flipped when compared to the system
                // chosen in the thesis.
                if x * y <= 0 {
                    (x, y, 0)
                } else if y.abs() > x.abs() {
                    (0, y - x, -x)
                } else {
                    (x - y, 0, -y)
                }
            },
            Norm::Quad => (x, y, 0),
        }
    }

    pub const fn dirs(self, norm: Norm) -> Dirs {
        let (e1, e2, e3) = self.shortest_path_dirs(norm);
        // this works for both hex and quad, as in the quad case we have `e3 == 0`.
        let e1pos = (e1 > 0) as u8;
        let e3neg = (e3 < 0) as u8 * (1 << 1);
        let e2pos = (e2 > 0) as u8 * (1 << 2);
        let e1neg = (e1 < 0) as u8 * (1 << 3);
        let e3pos = (e3 > 0) as u8 * (1 << 4);
        let e2neg = (e2 < 0) as u8 * (1 << 5);
        Dirs(e1pos | e3neg | e2pos | e1neg | e3pos | e2neg)
    }
}

impl std::ops::Add<Coords> for Coords {
    type Output = Coords;
    fn add(self, rhs: Coords) -> Self::Output {
        Coords {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl std::ops::Sub<Coords> for Coords {
    type Output = Coords;
    fn sub(self, rhs: Coords) -> Self::Output {
        Coords {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl std::ops::Neg for Coords {
    type Output = Coords;
    fn neg(self) -> Self::Output {
        Coords { x: -self.x, y: -self.y }
    }
}

impl std::ops::Mul<Coords> for isize {
    type Output = Coords;
    fn mul(self, rhs: Coords) -> Self::Output {
        Coords {
            x: rhs.x * self,
            y: rhs.y * self,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq)]
pub struct OrderedColWise {
    /// square root of number of vertices, nr of vertices per row / column in unit square
    pub len: usize,
}

impl OrderedColWise {
    pub fn new(len: usize) -> Self {
        Self { len }
    }

    pub fn nr_vertices(&self) -> usize {
        self.len * self.len
    }

    /// works for every combination of x and y,
    /// but is only fast for coordinates close to their representatives.
    /// (representatives are those where both x and y are contained in `0..self.len`)
    pub fn pack_small_coordinates(&self, mut x: isize, mut y: isize) -> Coords {
        // computing modulo is faster this way for values just outside len (as is the case for x and y)
        // also: remember that `%` is not the euclidean remainder (aka modulo)
        let ilen = self.len as isize;
        while x < 0 {
            x += ilen;
        }
        while y < 0 {
            y += ilen;
        }
        while x >= ilen {
            x -= ilen;
        }
        while y >= ilen {
            y -= ilen;
        }
        Coords { x, y }
    }

    pub fn coordinates_of(&self, v: usize) -> Coords {
        // note: both computations rely on `v >= 0`, which we now to be the case, cause `usize`
        let x = (v / self.len) as isize;
        let y = (v % self.len) as isize;
        Coords { x, y }
    }

    pub fn index_of(&self, coords: Coords) -> usize {
        let v = coords.x * (self.len as isize) + coords.y;
        v as usize
    }
}

/// on hexagonal coordinates, the first 6 bits are in use and represent the different sectors as follows:
/// - positive e1 coordiante (x in [`Coords`])
/// - negative e3 coordinate
/// - positive e2 coordinate (-y in [`Coords`])
/// - negative e1 coordinate (-x in [`Coords`])
/// - positive e3 coordinate
/// - negative e2 coordinate (y in [`Coords`])
///
/// The same is done with square coordinates, therefore bits 1 and 4 are always 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Dirs(pub u8);

impl Dirs {
    const MASK: u8 = (1 << 6) - 1;
    const FST_BIT: u8 = 1 << 0;
    const LST_BIT: u8 = 1 << 5;

    pub const fn all(norm: Norm) -> Self {
        match norm {
            Norm::Hex => Self(0b00111111),
            Norm::Quad => Self(0b00101101),
        }
    }

    pub const EMPTY: Self = Dirs(0);

    #[inline(always)]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    #[inline(always)]
    pub const fn nonempty(self) -> bool {
        self.0 != 0
    }

    /// cardinatity of set
    #[inline(always)]
    pub const fn count(self) -> u32 {
        self.0.count_ones()
    }

    #[inline(always)]
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    #[inline(always)]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    #[inline(always)]
    pub fn unionize(&mut self, other: Self) {
        self.0 |= other.0;
    }

    #[inline(always)]
    pub const fn intersection(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    #[inline(always)]
    pub fn intersect(&mut self, other: Self) {
        self.0 &= other.0;
    }

    #[inline(always)]
    pub const fn setminus(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    #[inline(always)]
    pub const fn rotate_right_hex(self) -> Self {
        let mut raw = self.0 >> 1;
        if self.0 & Self::FST_BIT != 0 {
            raw |= Self::LST_BIT;
        }
        debug_assert!(raw & Self::MASK == raw);
        Self(raw)
    }

    #[inline(always)]
    pub const fn rotate_left_hex(self) -> Self {
        let mut raw = self.0 << 1;
        if self.0 & Self::LST_BIT != 0 {
            raw |= Self::FST_BIT;
            raw &= Self::MASK;
        }
        debug_assert!(raw & Self::MASK == raw);
        Self(raw)
    }

    pub const fn add_adjacent_on_hex(self) -> Self {
        self.union(self.rotate_left_hex()).union(self.rotate_right_hex())
    }

    pub const fn keep_inner_on_hex(self) -> Self {
        self.intersection(self.rotate_left_hex())
            .intersection(self.rotate_right_hex())
    }

    pub const fn connected_on(self, norm: Norm) -> bool {
        const fn compute_connected(square: bool) -> [bool; 64] {
            let mut res = [false; 64];
            let mut union: u8 = 0;
            while union < 64 {
                let mut nr_flips = 0;
                let fst_bit = union & 1 != 0;
                let mut last_bit = fst_bit;
                let mut i = 1;
                while i < 6 {
                    // bits 1 and 4 on a square grid are not used, because
                    // this is where we store how any steps we took in direction e3.
                    // -> skip these
                    if !square || (i != 1 && i != 4) {
                        let curr_bit = (union & (1 << i)) != 0;
                        if curr_bit != last_bit {
                            nr_flips += 1;
                        }
                        last_bit = curr_bit;
                    }
                    i += 1;
                }
                assert!((fst_bit == last_bit) == (nr_flips % 2 == 0));
                res[union as usize] = nr_flips <= 2;
                union += 1;
            }
            res
        }
        const HEX_CONNECTED: [bool; 64] = compute_connected(false);
        const QUAD_CONNECTED: [bool; 64] = compute_connected(true);
        match norm {
            Norm::Hex => HEX_CONNECTED[self.0 as usize],
            Norm::Quad => QUAD_CONNECTED[self.0 as usize],
        }
    }

    /// same order as [`Norm::unit_directions`]
    pub const fn unit_bits(norm: Norm) -> &'static [Dirs] {
        match norm {
            Norm::Hex => &[
                Dirs(1 << 0),
                Dirs(1 << 1),
                Dirs(1 << 2),
                Dirs(1 << 3),
                Dirs(1 << 4),
                Dirs(1 << 5),
            ],
            Norm::Quad => &[
                Dirs(1 << 0),
                // bit 1 is unused (would be direction 3)
                Dirs(1 << 2),
                Dirs(1 << 3),
                // bit 4 is unused (would be negative direction 3)
                Dirs(1 << 5),
            ],
        }
    }

    pub fn all_bits_and_directions(
        norm: Norm,
    ) -> impl Iterator<Item = (&'static Self, &'static Coords)> + Clone {
        izip!(Self::unit_bits(norm), norm.unit_directions())
    }

    pub fn directions(self, norm: Norm) -> impl Iterator<Item = Coords> {
        let it = Self::all_bits_and_directions(norm);
        it.filter_map(move |(&dir, &coords)| self.intersection(dir).nonempty().then_some(coords))
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Norm {
    /// measures distance to `Coords { x: 0, y: 0 }` on a triangle grid
    Hex,
    /// measures distance to `Coords { x: 0, y: 0 }` on a square grid
    Quad,
}

impl Norm {
    pub fn apply(self, v: Coords) -> isize {
        let x = v.x.abs();
        let y = v.y.abs();
        match self {
            // note: due to stupidity, the program uses a different coordinate system than the thesis.
            // thus: `v.x - v.y`, not `v.x + v.y`
            Self::Hex => (v.x - v.y).abs().max(x).max(y),
            Self::Quad => x + y,
        }
    }

    /// returns all unit directions along grid lines (two directions per line)
    pub const fn unit_directions(self) -> &'static [Coords] {
        match self {
            Self::Hex => &[
                Coords { x: 1, y: 0 },
                Coords { x: 1, y: 1 },
                Coords { x: 0, y: 1 },
                Coords { x: -1, y: 0 },
                Coords { x: -1, y: -1 },
                Coords { x: 0, y: -1 },
            ],
            Self::Quad => &[
                Coords { x: 1, y: 0 },
                Coords { x: 0, y: 1 },
                Coords { x: -1, y: 0 },
                Coords { x: 0, y: -1 },
            ],
        }
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub struct GridGraph {
    pub grid: OrderedColWise,
    pub norm: Norm,
    /// if true: grid is wrapped to torus
    pub wrap: bool,
}

impl GridGraph {
    pub fn new(len: usize, norm: Norm, wrap: bool) -> Self {
        Self {
            grid: OrderedColWise::new(len),
            norm,
            wrap,
        }
    }

    pub fn try_from(g: &Embedding3D) -> Option<Self> {
        let (norm, wrap) = match g.shape() {
            Shape::SquareTorus => (Norm::Quad, true),
            Shape::TriangTorus => (Norm::Hex, true),
            Shape::SquareGrid => (Norm::Quad, false),
            Shape::TriangGrid => (Norm::Hex, false),
            _ => return None,
        };
        let len = f64::sqrt(g.nr_vertices() as f64) as usize;
        debug_assert_eq!(len * len, g.nr_vertices());
        Some(Self::new(len, norm, wrap))
    }

    pub fn side_len(&self) -> isize {
        self.grid.len as isize
    }

    pub fn unchecked_index_of(&self, v: Coords) -> usize {
        self.grid.index_of(v)
    }

    pub fn index_of(&self, v: Coords) -> Option<usize> {
        self.try_wrap(v).map(|w| self.grid.index_of(w))
    }

    pub fn coordinates_of(&self, v: usize) -> Coords {
        self.grid.coordinates_of(v)
    }

    /// try to bring coordinate in "normal form", e.g. how it maps to a vertex index.
    /// this is only guaranteed to work on tori, where wrapping is allowed.
    /// on non-tori, the input must already be in normal form in order to be returned.
    pub fn try_wrap(&self, v: Coords) -> Option<Coords> {
        let wrapped = self.grid.pack_small_coordinates(v.x, v.y);
        (self.wrap || wrapped == v).then_some(wrapped)
    }

    pub fn neighbors_of(&self, v: Coords) -> impl Iterator<Item = Coords> + '_ {
        self.norm
            .unit_directions()
            .iter()
            .filter_map(move |&dir| self.try_wrap(v + dir))
    }

    /// returns all vertices with distance `radius`.
    /// funkiness caused by tori is ignored. only non-wrapping distance is considered.
    #[allow(dead_code)]
    pub fn circle_around(&self, v: Coords, radius: usize) -> impl Iterator<Item = Coords> + '_ {
        let dist = radius as isize;
        self.norm.unit_directions().iter().circular_tuple_windows().flat_map(
            move |(&dir1, &dir2)| {
                (0..dist).filter_map(move |i| {
                    let n = v + i * dir1 + (dist - i) * dir2;
                    debug_assert_eq!(dist, self.norm.apply(n - v));
                    self.try_wrap(n)
                })
            },
        )
    }

    pub fn neighbor_indices_of(&self, v: Coords) -> impl Iterator<Item = usize> + '_ {
        self.neighbors_of(v).map(|v| self.unchecked_index_of(v))
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn quad_unit_directions_are_unit() {
        assert_eq!(Norm::Quad.unit_directions().len(), 4);
        for &dir in Norm::Quad.unit_directions() {
            assert_eq!(1, Norm::Quad.apply(dir));
        }
    }

    #[test]
    fn hex_unit_directions_are_unit() {
        assert_eq!(Norm::Hex.unit_directions().len(), 6);
        for &dir in Norm::Hex.unit_directions() {
            assert_eq!(1, Norm::Hex.apply(dir));
        }
    }

    #[test]
    fn shift_round_trip() {
        for raw in 0..Dirs::MASK {
            let dirs = Dirs(raw);
            assert_eq!(dirs.rotate_left_hex().rotate_right_hex(), dirs);
            assert_eq!(dirs.rotate_right_hex().rotate_left_hex(), dirs);
        }
    }
}
