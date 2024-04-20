use itertools::izip;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use self::bool_csr::BoolCSR;
use crate::geo::*;

use super::*;

pub mod explicit;
pub use explicit::*;
pub mod torus;

/// represents a graph automorphism
/// where vertices are named [`0..self.nr_vertices()`]
///
/// -> iff vertices `u` and `v` share an edge, then vertices `self.forward[u]` and `self.forward[v]` share an edge.
///
/// -> for any vertex `v` holds that `v == self.forward[self.backward[v]]`
pub trait Automorphism {
    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone;
    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone;

    fn nr_vertices(&self) -> usize;

    fn apply_forward(&self, v: usize) -> usize {
        debug_assert!(v < self.nr_vertices());
        self.forward().nth(v).unwrap()
    }

    fn apply_backward(&self, v: usize) -> usize {
        debug_assert!(v < self.nr_vertices());
        self.backward().nth(v).unwrap()
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct Identity {
    nr_vertices: usize,
}

impl Identity {
    pub fn new(nr_vertices: usize) -> Self {
        Identity { nr_vertices }
    }
}

impl Automorphism for Identity {
    fn forward(&self) -> std::ops::Range<usize> {
        0..self.nr_vertices()
    }

    fn backward(&self) -> std::ops::Range<usize> {
        0..self.nr_vertices()
    }

    fn nr_vertices(&self) -> usize {
        self.nr_vertices
    }
}

/// representation of all [`Automorphism`]'s of a given graph
pub trait SymmetryGroup {
    type Auto: Automorphism;
    type AutoIter<'a>: IntoIterator<Item = &'a Self::Auto> + Clone
    where
        Self: 'a;

    /// applies an authomorphism to vertices in place, returns all authomorphisms which
    /// map original vertices to result.
    /// order of vertices may be altered.
    ///
    /// this function must guarantee to find a unique representative of vertices and
    /// ALL autos bringing them there.
    fn to_representative<'a>(&'a self, vertices: &mut [usize]) -> Self::AutoIter<'a>;

    /// enumerates one vertex of each vertex class. this vertex will be result of [`Self::to_representative`]
    /// if a vertex of it's class is passed as only vertex.
    fn class_representatives(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone;

    fn all_automorphisms(&self) -> impl Iterator<Item = &Self::Auto>;

    const HAS_SYMMETRY: bool = true;

    fn into_enum(self) -> SymGroup;
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct NoSymmetry {
    identity: Identity,
}

impl NoSymmetry {
    pub fn new(nr_vertices: usize) -> Self {
        Self {
            identity: Identity::new(nr_vertices),
        }
    }
}

impl SymmetryGroup for NoSymmetry {
    type Auto = Identity;
    type AutoIter<'a> = std::iter::Once<&'a Identity>;

    fn to_representative<'a>(&'a self, vertices: &mut [usize]) -> Self::AutoIter<'a> {
        vertices.sort();
        std::iter::once(&self.identity)
    }

    fn class_representatives(&self) -> std::ops::Range<usize> {
        self.identity.forward()
    }

    fn all_automorphisms(&self) -> impl Iterator<Item = &Self::Auto> {
        std::iter::once(&self.identity)
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::None(self)
    }

    const HAS_SYMMETRY: bool = false;
}

pub trait DynAutomorphism {
    fn dyn_nr_vertices(&self) -> usize;
    fn dyn_apply_forward(&self, v: usize) -> usize;
    fn dyn_apply_backward(&self, v: usize) -> usize;
}

impl<T: Automorphism> DynAutomorphism for T {
    fn dyn_nr_vertices(&self) -> usize {
        <Self as Automorphism>::nr_vertices(self)
    }

    fn dyn_apply_forward(&self, v: usize) -> usize {
        <Self as Automorphism>::apply_forward(self, v)
    }

    fn dyn_apply_backward(&self, v: usize) -> usize {
        <Self as Automorphism>::apply_backward(self, v)
    }
}

pub trait DynSymmetryGroup {
    fn dyn_to_representative<'a>(
        &'a self,
        vertices: &mut [usize],
    ) -> SmallVec<[&'a dyn DynAutomorphism; 4]>;

    fn for_each_transform(&self, f: &mut dyn FnMut(&dyn DynAutomorphism));
}

impl<T: SymmetryGroup> DynSymmetryGroup for T {
    fn dyn_to_representative<'a>(
        &'a self,
        vertices: &mut [usize],
    ) -> SmallVec<[&'a dyn DynAutomorphism; 4]> {
        let mut res = SmallVec::new();
        let iter = <Self as SymmetryGroup>::to_representative(self, vertices);
        for auto in iter {
            res.push(auto as &dyn DynAutomorphism);
        }
        res
    }

    fn for_each_transform(&self, f: &mut dyn FnMut(&dyn DynAutomorphism)) {
        for auto in self.all_automorphisms() {
            f(auto as &dyn DynAutomorphism);
        }
    }
}

#[derive(Serialize, Deserialize)]
pub enum SymGroup {
    Explicit(ExplicitClasses),
    Torus6(torus::TorusSymmetry6),
    Torus4(torus::TorusSymmetry4),
    None(NoSymmetry),
}

impl SymGroup {
    pub fn to_dyn(&self) -> &dyn DynSymmetryGroup {
        match self {
            Self::Explicit(e) => e as &dyn DynSymmetryGroup,
            Self::Torus6(t) => t as &dyn DynSymmetryGroup,
            Self::Torus4(t) => t as &dyn DynSymmetryGroup,
            Self::None(n) => n as &dyn DynSymmetryGroup,
        }
    }
}
