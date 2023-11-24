#![warn(clippy::all, rust_2018_idioms)]

mod app;
pub use app::State;

mod graph;

#[allow(dead_code)]
mod geo;

mod rand;

