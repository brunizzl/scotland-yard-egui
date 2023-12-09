
use super::geo;

pub mod edgelist;
pub use edgelist::*;

pub mod planar;
pub use planar::*;

#[allow(dead_code)]
pub mod planar3d;
pub use planar3d::*;



#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InSet { No, Perhaps, Yes, NewlyAdded }
impl InSet {
    pub fn yes(self) -> bool { 
        debug_assert!(matches!(self, InSet::No | InSet::Yes));
        self == InSet::Yes 
    }
}

