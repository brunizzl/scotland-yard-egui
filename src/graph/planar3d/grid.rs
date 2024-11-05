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
/// - positive e1 coordiante (x in [``Coords`])
/// - negative e3 coordinate
/// - positive e2 coordinate (-y in [``Coords`])
/// - negative e1 coordinate (-x in [``Coords`])
/// - positive e3 coordinate
/// - negative e2 coordinate (y in [``Coords`])
///
/// The same is done with square coordinates, therefore bits 1 and 4 are always 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Dirs(pub u8);

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

    pub fn all_bits_and_directions(
        norm: Norm,
    ) -> impl Iterator<Item = (&'static Self, &'static Coords)> {
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
}
