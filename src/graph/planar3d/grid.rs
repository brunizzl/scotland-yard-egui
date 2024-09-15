use std::ops::Deref;

use serde::{Deserialize, Serialize};

use super::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Coords {
    pub x: isize,
    pub y: isize,
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
    pub fn nr_neighs(self) -> usize {
        match self {
            Self::Hex => 6,
            Self::Quad => 4,
        }
    }
}

pub struct GridGraph {
    vertices: OrderedColWise,
    norm: Norm,
    wrap: bool,
}

impl GridGraph {
    pub fn new(len: usize, norm: Norm, wrap: bool) -> Self {
        Self {
            vertices: OrderedColWise::new(len),
            norm,
            wrap,
        }
    }

    #[allow(dead_code)]
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

    pub fn neighbors_of(&self, v: Coords) -> impl Iterator<Item = Coords> + '_ {
        let Coords { x, y } = v;
        let raw = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
            (x - 1, y - 1),
            (x + 1, y + 1),
        ];
        raw.into_iter().take(self.norm.nr_neighs()).filter_map(|(nx, ny)| {
            let n = self.vertices.pack_small_coordinates(nx, ny);
            (self.wrap || (nx == n.x && ny == n.y)).then_some(n)
        })
    }

    pub fn neighbor_indices_of(&self, v: Coords) -> impl Iterator<Item = usize> + '_ {
        self.neighbors_of(v).map(|coords| self.index_of(coords))
    }
}

impl Deref for GridGraph {
    type Target = OrderedColWise;
    fn deref(&self) -> &Self::Target {
        &self.vertices
    }
}
