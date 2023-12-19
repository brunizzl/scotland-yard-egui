
use super::geo;

pub mod edgelist;
pub use edgelist::*;

pub mod planar;
pub use planar::*;

pub mod algos;
pub use algos::*;

pub mod planar3d;
pub use planar3d::*;



#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[derive(serde::Deserialize, serde::Serialize)]
pub enum InSet { No, Perhaps, Yes, NewlyAdded }
impl InSet {
    pub fn yes(self) -> bool { 
        debug_assert!(matches!(self, InSet::No | InSet::Yes));
        self == InSet::Yes 
    }
}

