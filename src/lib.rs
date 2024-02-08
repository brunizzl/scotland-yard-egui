#![warn(clippy::all, rust_2018_idioms)]

/// the management side of the program
mod app;
pub use app::State;

/// the logic / math side of the program
mod graph;

/// general purpose math concepts
mod geo;

#[allow(dead_code)]
mod rand;
