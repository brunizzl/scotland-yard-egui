#![warn(clippy::all, rust_2018_idioms)]

/// the management side of the program
mod app;
pub use app::State;

/// the logic / math side of the program
mod graph;

/// everything has special cases for some graphs, so
/// shapes are known by __all__ of the program
mod graph_shape;
use graph_shape::GraphShape;

/// general purpose math concepts
mod geo;

mod rand;

mod rle;
