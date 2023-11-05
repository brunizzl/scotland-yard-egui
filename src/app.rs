
use egui::Align2;

use super::graph;


pub struct VisualizedGraph {
    game_map: Option<graph::PlanarGraph>
}

impl VisualizedGraph {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        VisualizedGraph { game_map: None }
    }
}

impl eframe::App for VisualizedGraph {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            egui::widgets::global_dark_light_mode_buttons(ui);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            
        });
    }
}
