use serde::{Deserialize, Serialize};

use super::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Coords {
    pub x: isize,
    pub y: isize,
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

/// on a hexagonal grid, every direction can be expressed in
/// any combination of two (linearly independent) unit directions.
/// The combination with the shortest sum of absolute values is meant to be stored here.
/// e.g. one entry is always zero, on square grids always [`Self::e3`].
/// on hex grids, if in a [`Coords`] value, x and y have different signs (in thesis: same signs),
/// then `Coords{ x, y }` is the same as `CanonicalForm{ e1: x, e2: y, e3: 0 }`.
///
/// The purpose of this struct is to have a form of coordinates on the torus,
/// which directly tell which directions to take to get somewhere on a shortest path.
pub struct CanonicalCoords {
    /// same coodinate as is thesis, follows first coordinate exis
    pub e1: isize,
    /// negative coordinate compared to thesis, follows snd coodinate axis
    pub e2: isize,
    /// same coodinate as is thesis
    pub e3: isize,
}

impl CanonicalCoords {
    /// points east in screen directions
    pub const E1: Coords = Coords { x: 1, y: 0 };
    /// points south-south-west in screen directions
    pub const E2: Coords = Coords { x: 0, y: 1 };
    /// points nord-nord-west in screen directions
    pub const E3: Coords = Coords { x: -1, y: -1 };

    #[allow(dead_code)]
    pub fn as_coords(&self) -> Coords {
        self.e1 * Self::E1 + self.e2 * Self::E2 + self.e3 * Self::E3
    }

    /// only valid on [`Norm::Hex`]
    pub fn e1_line_index(&self) -> isize {
        self.e3 - self.e2
    }

    /// only valid on [`Norm::Hex`]
    pub fn e2_line_index(&self) -> isize {
        self.e1 - self.e3
    }

    /// only valid on [`Norm::Hex`]
    pub fn e3_line_index(&self) -> isize {
        self.e1 - self.e2
    }

    pub const fn dirs(&self) -> Dirs {
        // this works for both hex and quad, as in the quad case we just have `self.e3 == 0`.
        let e1pos = (self.e1 > 0) as u8;
        let e3neg = (self.e3 < 0) as u8 * (1 << 1);
        let e2pos = (self.e2 > 0) as u8 * (1 << 2);
        let e1neg = (self.e1 < 0) as u8 * (1 << 3);
        let e3pos = (self.e3 > 0) as u8 * (1 << 4);
        let e2neg = (self.e2 < 0) as u8 * (1 << 5);
        Dirs(e1pos | e3neg | e2pos | e1neg | e3pos | e2neg)
    }
}

/// given the unit directions of [`CanonicalCoords`],
/// this type differentiates which coordinate sign combo is present.
/// on hexagonal coordinates, the first 6 bits are in use and represent the different sectors as follows:
/// - positive e1 coordiante
/// - negative e3 coordinate
/// - positive e2 coordinate
/// - negative e1 coordinate
/// - positive e3 coordinate
/// - negative e2 coordinate
///
/// therefore for any single vector in this format:
/// not on grid line -> two bits are set.
/// on grid line but nonzero -> one bit is set.
/// zero -> no bit is set.
///
/// The same is done with square coordinates, therefore bits 1 and 4 are always 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Dirs(pub u8);

#[allow(dead_code)]
impl Dirs {
    const MASK: u8 = (1 << 6) - 1;

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

    #[inline(always)]
    pub const fn contain(self, cc: &CanonicalCoords) -> bool {
        let dirs = cc.dirs();
        (self.0 | !dirs.0) == u8::MAX
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

    pub const fn add_adjacent_on_hex(self) -> Self {
        let mut res = (self.0 << 1) | self.0 | (self.0 >> 1);
        const FST_BIT: u8 = 1 << 0;
        const LST_BIT: u8 = 1 << 5;
        if self.0 & FST_BIT != 0 {
            res |= LST_BIT;
        }
        if self.0 & LST_BIT != 0 {
            res |= FST_BIT;
        }
        Self(res & Self::MASK)
    }

    pub const fn keep_inner_on_hex(self) -> Self {
        let mut shift_right = self.0 >> 1;
        let mut shift_left = self.0 << 1;
        const FST_BIT: u8 = 1 << 0;
        const LST_BIT: u8 = 1 << 5;
        if self.0 & FST_BIT != 0 {
            shift_right |= LST_BIT;
        }
        if self.0 & LST_BIT != 0 {
            shift_left |= FST_BIT;
        }
        let res = shift_right & self.0 & shift_left;
        Self(res & Self::MASK)
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

    pub const fn half_rotation(self) -> Self {
        // this works for both hex and quad,
        // as in the quad case we just have some more bits guaranteed to be zero.
        let new_high = self.0 << 3;
        let new_low = self.0 >> 3;
        Self((new_high | new_low) & Self::MASK)
    }

    pub fn all_bits_and_directions(
        norm: Norm,
    ) -> impl Iterator<Item = (&'static Self, &'static Coords)> {
        izip!(Self::unit_bits(norm), norm.unit_directions())
    }

    pub fn directions(self, norm: Norm) -> impl Iterator<Item = Coords> {
        let it = Self::all_bits_and_directions(norm);
        it.filter_map(move |(&dir, &coords)| (self.intersection(dir).nonempty()).then_some(coords))
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

    pub const fn canonical_coords(self, vec: Coords) -> CanonicalCoords {
        const fn e123(e1: isize, e2: isize, e3: isize) -> CanonicalCoords {
            CanonicalCoords { e1, e2, e3 }
        }
        let Coords { x, y } = vec;
        match self {
            Self::Hex => {
                // again: the coordinate system in use here has
                // the y-axis flipped when compared to the system
                // chosen in the thesis.
                if x * y <= 0 {
                    e123(x, y, 0)
                } else if y.abs() > x.abs() {
                    e123(0, y - x, -x)
                } else {
                    e123(x - y, 0, -y)
                }
            },
            Self::Quad => e123(x, y, 0),
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
    fn round_trip_canonical_hex() {
        let norm = Norm::Hex;
        for x in -5..=5 {
            for y in -5..=5 {
                let vec = Coords { x, y };
                let cc = norm.canonical_coords(vec);
                let back = cc.as_coords();
                assert_eq!(vec, back);
                let len = cc.e1.abs() + cc.e2.abs() + cc.e3.abs();
                assert_eq!(len, norm.apply(vec));
            }
        }
    }

    #[test]
    fn round_trip_canonical_quad() {
        let norm = Norm::Quad;
        for x in -5..=5 {
            for y in -5..=5 {
                let vec = Coords { x, y };
                let cc = norm.canonical_coords(vec);
                let back = cc.as_coords();
                assert_eq!(vec, back);
                let len = cc.e1.abs() + cc.e2.abs() + cc.e3.abs();
                assert_eq!(len, norm.apply(vec));
            }
        }
    }

    #[test]
    fn round_trip_sector_combos_dirs_hex() {
        use itertools::Itertools;
        let norm = Norm::Hex;
        for mut active_dirs in norm.unit_directions().iter().copied().powerset() {
            let sectors = active_dirs
                .iter()
                .map(|c| norm.canonical_coords(*c).dirs())
                .fold(Dirs::EMPTY, Dirs::union);
            let mut sectors_dirs = sectors.directions(norm).collect_vec();
            active_dirs.sort();
            sectors_dirs.sort();
            assert_eq!(active_dirs, sectors_dirs);
        }
    }

    #[test]
    fn round_trip_sector_combos_dirs_quad() {
        use itertools::Itertools;
        let norm = Norm::Quad;
        for mut active_dirs in norm.unit_directions().iter().copied().powerset() {
            let sectors = active_dirs
                .iter()
                .map(|c| norm.canonical_coords(*c).dirs())
                .fold(Dirs::EMPTY, Dirs::union);
            let mut sectors_dirs = sectors.directions(norm).collect_vec();
            active_dirs.sort();
            sectors_dirs.sort();
            assert_eq!(active_dirs, sectors_dirs);
        }
    }

    #[test]
    fn connected_sections() {
        {
            let norm = Norm::Hex;
            let [a, b, c, d, e, f] = [1, 2, 4, 8, 16, 32];
            assert!(Dirs(a | b | d | e | f).connected_on(norm));
            assert!(Dirs(a | b | c).connected_on(norm));
            assert!(Dirs(a | b).connected_on(norm));
            assert!(Dirs(c | d | e).connected_on(norm));
            assert!(Dirs(f | a).connected_on(norm));
            assert!(Dirs::EMPTY.connected_on(norm));
            assert!(Dirs(d).connected_on(norm));

            assert!(!Dirs(a | c | d).connected_on(norm));
            assert!(!Dirs(a | c | d | f).connected_on(norm));
            assert!(!Dirs(a | d).connected_on(norm));
        }
        {
            let norm = Norm::Quad;
            let [a, b, c, d] = [1, 4, 8, 32];
            assert!(Dirs(a | c | d).connected_on(norm));
            assert!(Dirs(a | b | d).connected_on(norm));
            assert!(Dirs(a | b | c).connected_on(norm));
            assert!(Dirs(a | b).connected_on(norm));
            assert!(Dirs(c | d).connected_on(norm));
            assert!(Dirs(d | a).connected_on(norm));
            assert!(Dirs::EMPTY.connected_on(norm));
            assert!(Dirs(d).connected_on(norm));

            assert!(!Dirs(a | c).connected_on(norm));
            assert!(!Dirs(b | d).connected_on(norm));
        }
    }

    #[test]
    fn adjacent_directions_are_connected() {
        for i in 0..6 {
            let single_dir = Dirs(1 << i);
            let three_dirs = single_dir.add_adjacent_on_hex();
            assert!(three_dirs.0.count_ones() == 3);
            assert!(three_dirs.connected_on(Norm::Hex));
        }
    }

    #[test]
    fn unit_directions_in_same_order() {
        for norm in [Norm::Hex, Norm::Quad] {
            for (&dirs, &v) in Dirs::all_bits_and_directions(norm) {
                let v_dirs = norm.canonical_coords(v).dirs();
                assert_eq!(dirs, v_dirs);
            }
        }
    }
}
