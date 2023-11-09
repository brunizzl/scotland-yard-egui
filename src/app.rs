
use std::collections::VecDeque;

use egui::*;

use super::{ graph, Bool3 };


pub enum GraphShape { Hexagon, Pentagon, }

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Character {
    is_cop: bool, //else is robber
    on_node: bool,
    pub nearest_node: usize,
    pos: Pos2,
    distancs: Vec<usize>,
}

impl Character {
    fn new(is_cop: bool) -> Character {
        Character { is_cop, on_node: false, nearest_node: 0, pos: pos2(0.1, 0.1), distancs: Vec::new() }
    }

    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, 
    //  change to that neighbor
    //(converges to globally nearest node only for "convex" graphs, 
    //  e.g. planar graphs, where each inside face is convex and the complement of the outside face is convex)
    fn update(&mut self, tolerance: f32, map: &graph::Graph, queue: &mut VecDeque<usize>) {
        let nearest_pos = map.nodes()[self.nearest_node];
        let mut nearest_dist = (nearest_pos - self.pos).length_sq();
        let mut change = false;
        let mut maybe_neighbor_closer = true;
        while maybe_neighbor_closer {
            maybe_neighbor_closer = false;
            for (i, &neigh) in map.neigbors_of(self.nearest_node) {
                let neigh_dist = (neigh - self.pos).length_sq();
                if neigh_dist < nearest_dist {
                    self.nearest_node = i;
                    nearest_dist = neigh_dist;
                    maybe_neighbor_closer = true;
                    change = true;
                }
            }
        }
        self.on_node = nearest_dist <= tolerance * tolerance;

        if change && self.on_node {
            map.calc_distances_to(self.nearest_node, &mut self.distancs, queue);
        }
    }
}

pub struct State {
    map: graph::Graph,
    map_shape: GraphShape,
    map_radius: usize, //(approx.) shortest path length from origin to rim

    //state kept for each node in map
    robber_closer: Vec<bool>,
    in_convex_cop_hull: Vec<Bool3>, 
    dist_to_outside_hull: Vec<usize>, //minimum dist to nearest point in convex cop hull
    min_cop_dist: Vec<usize>, //elementwise minimum of .distance of active cops in self.characters

    show_robber_closer: bool,
    show_convex_hull: bool,
    show_escapeable_nodes: bool, //all nodes closer to the outside of the convex hull than to the next cop

    //first is robber, rest are cops
    characters: Vec<Character>,
    tolerance: f32, //how close must a character be to a vertex to count as beeing on thet vertex

    queue: VecDeque<usize>, //kept permanentely to reduce allocations when a character update is computed.
}

impl State {

    fn resize(&mut self, radius: usize) {
        let map = graph::Graph::new_plane_tiles_hexagon(radius, Pos2::new(1.0, 1.0), 1.0);
        self.map_shape = GraphShape::Hexagon;
        self.map = map;
        self.map_radius = radius;      
        for char in &mut self.characters {
            char.update(self.tolerance, &self.map, &mut self.queue);
        }
    }

    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = State { 
            map: graph::Graph::empty(),
            map_shape: GraphShape::Hexagon,
            map_radius: 0,

            robber_closer: Vec::new(),
            in_convex_cop_hull: Vec::new(),
            dist_to_outside_hull: Vec::new(),
            min_cop_dist: Vec::new(),

            show_robber_closer: true,
            show_convex_hull: true,
            show_escapeable_nodes: true,
            
            characters: vec![Character::new(false)],
            tolerance: 0.1,
            queue: VecDeque::new(),
         };
         res.resize(5);
         res
    }

    pub fn robber(&self) -> &Character {
        & self.characters[0]
    }

    pub fn police(&self) -> &[Character] {
        & self.characters[1..]
    }

    fn active_cops(&self) -> Vec<usize> {
        self.characters.iter().filter(|c| c.is_cop && c.on_node).map(|c| c.nearest_node).collect()
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
                    let mut new = Character::new(true);
                    new.update(0.01, &self.map, &mut self.queue);
                    self.characters.push(new);
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
            for (&v1, &v2) in self.map.edges() {                   
                let edge = [
                    to_screen.transform_pos(v1), 
                    to_screen.transform_pos(v2)];
                let line = Shape::LineSegment { points: edge, stroke: edge_stroke };
                painter.add(line);
            }

            for (i, character) in self.characters.iter_mut().enumerate() {                
                let character_size = 8.0;

                let real_screen_pos = to_screen.transform_pos(character.pos);
                let draw_pos = if character.on_node {
                    let nearest_node_pos = to_screen.transform_pos(self.map.nodes()[character.nearest_node]);
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

                character.update(self.tolerance, &self.map, &mut self.queue);

                let fill_color = if character.is_cop {
                    Color32::from_rgb(10, 50, 190)
                } else {
                    Color32::from_rgb(200, 50, 50)
                };
                let character_circle = Shape::circle_filled(draw_pos, character_size, fill_color);
                painter.add(character_circle);
            }
        });
    }
}
