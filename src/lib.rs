#![warn(clippy::all, rust_2018_idioms)]


#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Bool3 { No, Yes, Perhaps }

mod app;
pub use app::State;

mod graph;

