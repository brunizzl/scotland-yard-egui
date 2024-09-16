use serde::{Deserialize, Serialize};

use super::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
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

    pub fn pack_small_coordinates(&self, mut x: isize, mut y: isize) -> Coords {
        // computing modulo is faster this way for values just outside len (as is the case for x and y)
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
        let x = (v / self.len) as isize;
        let y = (v % self.len) as isize;
        Coords { x, y }
    }

    pub fn index_of(&self, coords: Coords) -> usize {
        let v = coords.x * (self.len as isize) + coords.y;
        v as usize
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Norm {
    Hex,
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
    pub fn unit_directions(self) -> &'static [Coords] {
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
    pub columns: OrderedColWise,
    pub norm: Norm,
    /// if true: grid is wrapped to torus
    pub wrap: bool,
}

impl GridGraph {
    pub fn new(len: usize, norm: Norm, wrap: bool) -> Self {
        Self {
            columns: OrderedColWise::new(len),
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
        self.columns.len as isize
    }

    pub fn index_of(&self, v: Coords) -> usize {
        self.columns.index_of(v)
    }

    pub fn coordinates_of(&self, v: usize) -> Coords {
        self.columns.coordinates_of(v)
    }

    /// try to bring coordinate in "normal form", e.g. how it maps to a vertex index.
    /// this is only guaranteed to work on tori, where wrapping is allowed.
    /// on non-tori, the input must already be in normal form in order to be returned.
    pub fn try_wrap(&self, v: Coords) -> Option<Coords> {
        let wrapped = self.columns.pack_small_coordinates(v.x, v.y);
        (self.wrap || wrapped == v).then_some(wrapped)
    }

    pub fn neighbors_of(&self, v: Coords) -> impl Iterator<Item = Coords> + '_ {
        self.norm
            .unit_directions()
            .iter()
            .filter_map(move |&dir| self.try_wrap(v + dir))
    }

    /// returns all vertices with distance `dist`.
    /// funkiness caused by tori is ignored. only non-wrapping distance is considered.
    #[allow(dead_code)]
    pub fn dist_neighbors_of(&self, v: Coords, dist: usize) -> impl Iterator<Item = Coords> + '_ {
        let dist = dist as isize;
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
        self.neighbors_of(v).map(|v| self.index_of(v))
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
