
use std::default;

use egui::*;

use super::graph;


pub enum GraphShape { Hexagon, Pentagon, }

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
struct CharacterPos {
    nearest_node: usize,
    pos: Pos2,
}

impl Default for CharacterPos {
    fn default() -> Self {
        CharacterPos { nearest_node: 0, pos: pos2(0.1, 0.1) }
    }
}

impl CharacterPos {
    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, change to that neighbor
    //(converges to globally nearest node only for "convex" graphs)
    fn fast_update_pos(&mut self, map: &graph::Graph) -> f32 {
        let nearest_pos = map.nodes()[self.nearest_node].pos;
        let mut nearest_dist = (nearest_pos - self.pos).length_sq();
        for (i, neigh) in map.neigbors_of(self.nearest_node) {
            let neigh_dist = (neigh.pos - self.pos).length_sq();
            if neigh_dist < nearest_dist {
                self.nearest_node = i;
                nearest_dist = neigh_dist;
            }
        }
        f32::sqrt(nearest_dist)
    }
}

pub struct State {
    map: graph::Graph,
    map_shape: GraphShape,
    map_radius: usize,

    nr_cops: usize,
    cops: Vec<CharacterPos>,
    robber: CharacterPos,
}

impl State {
    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let radius = 5;

        State { 
            map: graph::Graph::new_plane_tiles_hexagon(radius, Pos2::new(1.0, 1.0), 1.0),
            map_shape: GraphShape::Hexagon,
            map_radius: radius,

            nr_cops: 0,
            cops: Vec::new(),
            robber: CharacterPos::default(),
         }
    }
}

impl eframe::App for State {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        // Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        TopBottomPanel::top("top_panel").show(ctx, |ui| {
            widgets::global_dark_light_mode_buttons(ui);
        });



        SidePanel::left("left_panel").show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.heading("Optionen");

                { //adjust radius
                    let mut new_radius = self.map_radius;
                    ui.horizontal(|ui| {
                        ui.label("Radius: ");
                        ui.add(DragValue::new(&mut new_radius));
                    });
                    new_radius = usize::max(2, usize::min(new_radius, 100));
                    if new_radius != self.map_radius {
                        self.map_radius = new_radius;
                        self.map = graph::Graph::new_plane_tiles_hexagon(self.map_radius, Pos2::new(1.0, 1.0), 1.0);
                        self.robber.nearest_node = 0;
                        for cop in &mut self.cops {
                            cop.nearest_node = 0;
                        }
                    }
                }
                if ui.button("Neuer Cop").clicked() {
                    self.cops.push(CharacterPos::default());
                }
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            let (response, painter) = ui.allocate_painter(
                Vec2::new(ui.available_width(), ui.available_height()), Sense::hover());            

            let to_screen = {
                let from = Rect{ min: Pos2::ZERO, max: Pos2::new(2.0, 2.0) };

                let rect_len = f32::min(response.rect.height(), response.rect.width());
                let to_middle = (response.rect.width() - rect_len) / 2.0;
                let screen_min = response.rect.min + vec2(to_middle, 0.0);
                let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

                emath::RectTransform::from_to(from, to)
            };
            
            let edge_stroke = Stroke::new(1.0, Color32::from_rgb(150, 150, 150));
            for (v1, v2) in self.map.edges() {                   
                let edge = [
                    to_screen.transform_pos(v1.pos), 
                    to_screen.transform_pos(v2.pos)];
                let line = Shape::LineSegment { points: edge, stroke: edge_stroke };
                painter.add(line);
            }

            let characters = std::iter::once(&mut self.robber).chain(self.cops.iter_mut());
            for (i, character) in characters.enumerate() {                
                let character_size = 8.0;

                let drawn_pos = to_screen.transform_pos(character.pos);
                let point_rect = Rect::from_center_size(drawn_pos, vec2(character_size * 3.0, character_size * 3.0));
                let character_id = response.id.with(i);
                let point_response = ui.interact(point_rect, character_id, Sense::drag());
                character.pos = to_screen.inverse().transform_pos(drawn_pos + point_response.drag_delta());

                let dist_closest = character.fast_update_pos(&self.map);
                if dist_closest <= 0.05 {
                    let nearest_node_pos = to_screen.transform_pos(self.map.nodes()[character.nearest_node].pos);
                    let marker_circle = Shape::circle_stroke(nearest_node_pos, character_size + 4.0, edge_stroke);
                    painter.add(marker_circle);
                }

                let fill_color = if i == 0 {
                    Color32::from_rgb(200, 50, 50) //robber
                } else {
                    Color32::from_rgb(10, 50, 190) //cop
                };
                let character_circle = Shape::circle_filled(drawn_pos, character_size, fill_color);
                painter.add(character_circle);
            }
        });
    }
}
