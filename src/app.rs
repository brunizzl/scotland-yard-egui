
use egui::{Align2, Pos2};

use super::graph;


pub struct VisualizedGraph {
    game_map: Option<graph::PlanarGraph>
}

impl VisualizedGraph {
    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        VisualizedGraph { game_map: Some(
            graph::PlanarGraph::new_plane_tiles_hexagon(3, Pos2::new(500.0, 500.0), 400.0)) }
    }
}

impl eframe::App for VisualizedGraph {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            egui::widgets::global_dark_light_mode_buttons(ui);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let points = [Pos2::new(100.0, 100.0), Pos2::new(1000.0, 1000.0)];
            let color = egui::Color32::from_rgb(255, 0, 200);
            let stroke = egui::Stroke::new(5.0, color);
            let line = egui::Shape::LineSegment { points, stroke };
            
            let (_response, painter) = ui.allocate_painter(
                egui::Vec2::new(ui.available_width(), 300.0), egui::Sense::hover());
            painter.add(line);
        });
    }
}
