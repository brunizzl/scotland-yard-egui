#![warn(clippy::all, rust_2018_idioms)]

mod app;
pub use app::State;

mod graph;

mod geo;

#[allow(dead_code)]
mod rand;
