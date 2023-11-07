
use egui::*;

use super::graph;


pub enum GraphShape { Hexagon(Vec<graph::hex_pos::HexPos>), Pentagon, }

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct CharacterPos {
    pub nearest_node: usize,
    on_node: bool,
    pos: Pos2,
}

impl Default for CharacterPos {
    fn default() -> Self {
        CharacterPos { nearest_node: 0, on_node: false, pos: pos2(0.1, 0.1) }
    }
}

impl CharacterPos {
    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, change to that neighbor
    //(converges to globally nearest node only for "convex" graphs)
    fn fast_update_pos(&mut self, map: &graph::Graph) -> bool {
        let nearest_pos = map.nodes()[self.nearest_node].pos;
        let mut nearest_dist = (nearest_pos - self.pos).length_sq();
        let mut change = false;
        for (i, neigh) in map.neigbors_of(self.nearest_node) {
            let neigh_dist = (neigh.pos - self.pos).length_sq();
            if neigh_dist < nearest_dist {
                self.nearest_node = i;
                nearest_dist = neigh_dist;
                change = true;
            }
        }
        let now_on_node = nearest_dist <= 0.01;
        change |= now_on_node != self.on_node;
        self.on_node = now_on_node;
        change
    }
}

pub struct State {
    map: graph::Graph,
    map_shape: GraphShape,
    map_radius: usize,

    show_robber_closer: bool,
    show_convex_hull: bool,
    show_escapeable_nodes: bool, //all nodes closer to the outside of the convex hull than to the next cop

    cops: Vec<CharacterPos>,
    robber: CharacterPos,
}

impl State {

    fn resize(&mut self, radius: usize) {
        let (map, names) = graph::Graph::new_plane_tiles_hexagon(radius, Pos2::new(1.0, 1.0), 1.0);
        self.map_shape = GraphShape::Hexagon(names);
        self.map = map;
        self.map_radius = radius;        
        self.robber.nearest_node = 0;
        for cop in &mut self.cops {
            cop.nearest_node = 0;
        }
    }

    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = State { 
            map: graph::Graph::empty(),
            map_shape: GraphShape::Hexagon(Vec::new()),
            map_radius: 0,

            show_robber_closer: true,
            show_convex_hull: true,
            show_escapeable_nodes: true,

            cops: Vec::new(),
            robber: CharacterPos::default(),
         };
         res.resize(5);
         res
    }

    fn active_cops(&self) -> Vec<usize> {
        self.cops.iter().filter(|c| c.on_node).map(|c| c.nearest_node).collect()
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
            ui.vertical(|ui| {
                ui.heading("Optionen");

                { //adjust radius
                    let mut new_radius = self.map_radius;
                    ui.horizontal(|ui| {
                        ui.label("Radius: ");
                        ui.add(DragValue::new(&mut new_radius));
                    });
                    new_radius = usize::max(1, usize::min(new_radius, 100));
                    if new_radius != self.map_radius {
                        self.resize(new_radius);
                    }
                }
                if ui.button("neuer Cop").clicked() {
                    self.cops.push(CharacterPos::default());
                }
                ui.add(Checkbox::new(&mut self.show_robber_closer, "markiere für Räuber\n nähere Knoten"));
                ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige Convexe Hüll\n um Cops"));
                ui.add(Checkbox::new(&mut self.show_escapeable_nodes, "zeige Punkte näher an\n Hüllenrand als an Cops"));
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

            if self.show_robber_closer {
                let robber_closer_color = Color32::from_rgb(80, 210, 80);
                for v in self.map.nodes() {
                    if v.robber_advantage > 0 {
                        let draw_pos = to_screen.transform_pos(v.pos);
                        let marker_circle = Shape::circle_filled(draw_pos, 4.0, robber_closer_color);
                        painter.add(marker_circle);
                    }
                }
            }
            if self.show_convex_hull {
                let stroke = Stroke::new(2.0, Color32::from_rgb(100, 100, 220));
                for v in self.map.nodes() {
                    if graph::Bool3::Yes == v.in_convex_hull  {
                        let draw_pos = to_screen.transform_pos(v.pos);
                        let marker_circle = Shape::circle_stroke(draw_pos, 6.0, stroke);
                        painter.add(marker_circle);
                    }
                }
            }

            let characters = std::iter::once(&mut self.robber).chain(self.cops.iter_mut());
            let mut change = false;
            for (i, character) in characters.enumerate() {                
                let character_size = 8.0;

                let real_screen_pos = to_screen.transform_pos(character.pos);
                let draw_pos = if character.on_node {
                    let nearest_node_pos = to_screen.transform_pos(self.map.nodes()[character.nearest_node].pos);
                    let marker_circle = Shape::circle_stroke(nearest_node_pos, character_size + 4.0, edge_stroke);
                    painter.add(marker_circle);
                    nearest_node_pos
                }
                else {
                    real_screen_pos
                };
                let point_rect = Rect::from_center_size(draw_pos, vec2(character_size * 3.0, character_size * 3.0));
                let character_id = response.id.with(i);
                let point_response = ui.interact(point_rect, character_id, Sense::drag());
                character.pos = to_screen.inverse().transform_pos(real_screen_pos + point_response.drag_delta());

                change |= character.fast_update_pos(&self.map);

                let fill_color = if i == 0 {
                    Color32::from_rgb(200, 50, 50) //robber
                } else {
                    Color32::from_rgb(10, 50, 190) //cop
                };
                let character_circle = Shape::circle_filled(draw_pos, character_size, fill_color);
                painter.add(character_circle);
            }
            if change {
                let active_cops = self.active_cops();
                self.map.calc_robber_advantage(self.robber.nearest_node, &active_cops);
                if let GraphShape::Hexagon(names) = &self.map_shape {
                    self.map.calc_hex_convex_police_hull(&names, &active_cops);
                }
            }
        });
    }
}
