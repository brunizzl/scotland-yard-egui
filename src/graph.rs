use super::geo;

/// purpose build data structure for very regular neighbor relations.
/// the same space is kept for each node's neighbors. this is only efficient if most nodes have
/// close to the maximum graph degree. even a single outliar
/// with larger degree will greatly diminish the efficiency of this approach.
pub mod edgelist;
pub use edgelist::*;

/// 2D embeddings of planar graphs
pub mod planar;
pub use planar::*;

pub mod algos;
pub use algos::*;

/// convex 3D embeddings of planar graphs
pub mod planar3d;
pub use planar3d::*;

/// bruteforce algortithms to compute the lazy-cop number of a graph
pub mod bruteforce;

#[allow(dead_code)]
/// compressed sparse row matrix for bool entries, e.g. no actual value array exists
/// great to e.g. neighbors in a sparse (for example planar) graph
pub mod bool_csr;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InSet {
    No = 0,
    Interieur = 1,
    OnBoundary = 2,
    Unknown = 4,
    NewlyAdded = 8,
}
const _: () = assert!(std::mem::size_of::<InSet>() == 1);

impl InSet {
    const fn finished_construction(self) -> bool {
        matches!(self, InSet::No | InSet::Interieur | InSet::OnBoundary)
    }

    #[inline(always)]
    pub const fn in_set(self) -> bool {
        debug_assert!(self.finished_construction());
        !self.outside()
    }

    #[inline(always)]
    pub const fn on_boundary(self) -> bool {
        debug_assert!(self.finished_construction());
        matches!(self, InSet::OnBoundary)
    }

    #[inline(always)]
    pub const fn in_interieur(self) -> bool {
        debug_assert!(self.finished_construction());
        matches!(self, InSet::Interieur)
    }

    #[inline(always)]
    pub const fn outside(self) -> bool {
        debug_assert!(self.finished_construction());
        matches!(self, InSet::No)
    }
}
