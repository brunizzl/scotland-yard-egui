
use super::geo;

pub mod edgelist;
pub use edgelist::*;

pub mod planar;
pub use planar::*;

pub mod algos;
pub use algos::*;

pub mod planar3d;
pub use planar3d::*;

/// bruteforce algrotithms to compute the lazy-cop number of a graph
pub mod bruteforce;
pub use bruteforce::*;



#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InSet { 
    No = 0, 
    Yes = 1, 
    OnBoundary = 2, 
    Perhaps = 4, 
    NewlyAdded = 8, 
}
const _: () = assert!(std::mem::size_of::<InSet>() == 1);

impl InSet {
    const fn finished_construction(self) -> bool {
        matches!(self, InSet::No | InSet::Yes | InSet::OnBoundary)
    }

    #[inline(always)]
    pub const fn inside(self) -> bool { 
        debug_assert!(self.finished_construction());
        !self.outside()
    }

    #[inline(always)]
    pub const fn on_boundary(self) -> bool {
        debug_assert!(self.finished_construction());
        matches!(self, InSet::OnBoundary)
    }

    #[inline(always)]
    pub const fn outside(self) -> bool {
        debug_assert!(self.finished_construction());
        matches!(self, InSet::No)
    }
}

