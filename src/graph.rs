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

pub mod hull;
pub use hull::*;

/// convex 3D embeddings of planar graphs
pub mod planar3d;
pub use planar3d::*;

/// bruteforce algortithms to compute the lazy-cop number of a graph
pub mod bruteforce;

pub mod shape;
pub use shape::{Resolution, Shape, Wrap};

/// compressed sparse row matrix for bool entries, e.g. no actual value array exists
/// great to e.g. store neighbors in a sparse (for example planar) graph
pub mod bool_csr;
