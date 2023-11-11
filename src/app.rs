
use std::collections::VecDeque;

use egui::*;

use super::{ graph::Graph, graph };



#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InSet { No, Perhaps, Yes, NewlyAdded }

#[derive(Clone, Copy, PartialEq)]
pub enum GraphShape { RegularPolygon(usize), Random }

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Character {
    is_cop: bool, //else is robber
    on_node: bool,
    pub nearest_node: usize,
    pos: Pos2,
    distances: Vec<usize>,
}

impl Character {
    fn new_cop(pos: Pos2) -> Self {
        Character { is_cop: true, on_node: false, nearest_node: 0, pos, distances: Vec::new() }
    }

    fn new_robber(pos: Pos2) -> Self {
        Character { is_cop: false, on_node: false, nearest_node: 0, pos, distances: Vec::new() }
    }

    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, 
    //  change to that neighbor
    //(converges to globally nearest node only for "convex" graphs, 
    //  e.g. planar graphs, where each inside face is convex and the complement of the outside face is convex)
    fn update(&mut self, tolerance: f32, map: &Graph, queue: &mut VecDeque<usize>) {
        let (nearest_node, nearest_dist_sq) = map.find_nearest_node(self.pos, self.nearest_node);

        if nearest_node != self.nearest_node || self.distances.len() != map.len() {
            queue.clear();
            queue.push_back(nearest_node);
            self.distances.clear();
            self.distances.resize(map.len(), usize::MAX);
            self.distances[nearest_node] = 0;
            map.calc_distances_to(queue, &mut self.distances);
        }
        self.nearest_node = nearest_node;
        self.on_node = nearest_dist_sq <= tolerance * tolerance;
    }
}

pub struct State {
    map: Graph,

    //overall map shape: == 2 is line, == 3 triangle, == 4 square, == 5 pentagon etc.
    map_shape: GraphShape,
    map_radius: usize, //(approx.) shortest path length from origin to rim

    extreme_vertices: [usize; 4], //indices of vertices with extreme coodinates

    //state kept for each node in map
    in_convex_cop_hull: Vec<InSet>, 
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

    fn compute_extreme_vertices(points: &[Pos2]) -> [usize; 4] {
        let mut res = [0; 4];
        let mut res_vals = [points[0]; 4];
        for (i, p) in points.iter().enumerate() {
            if p.x < res_vals[0].x {
                res[0] = i;
                res_vals[0] = *p;
            }
            if p.y < res_vals[1].y {
                res[1] = i;
                res_vals[1] = *p;
            }
            if p.x > res_vals[2].x {
                res[2] = i;
                res_vals[2] = *p;
            }
            if p.y > res_vals[3].y {
                res[3] = i;
                res_vals[3] = *p;
            }
        }
        res
    }

    fn recompute_graph(&mut self) {
        self.map = match self.map_shape {
            GraphShape::RegularPolygon(n) => graph::triangulated_regular_polygon(
                n, 
                self.map_radius),
            //TODO: implement function to build random graph
            GraphShape::Random => graph::random_triangulated(4 * self.map_radius * self.map_radius),
        };
        for char in &mut self.characters {
            char.nearest_node = 0;
            char.update(self.tolerance, &self.map, &mut self.queue);
        }
        self.extreme_vertices = Self::compute_extreme_vertices(self.map.positions());
        self.update_min_cop_dist();
        self.update_convex_cop_hull();
        self.update_dist_to_outside_hull();
    }

    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = State { 
            map: Graph::empty(),
            map_shape: GraphShape::RegularPolygon(6),
            map_radius: 8,

            extreme_vertices: [0; 4],

            in_convex_cop_hull: Vec::new(),
            dist_to_outside_hull: Vec::new(),
            min_cop_dist: Vec::new(),

            show_robber_closer: false,
            show_convex_hull: false,
            show_escapeable_nodes: false,
            
            characters: vec![
                Character::new_robber(Pos2::ZERO),
                Character::new_cop(pos2(0.25, 0.0)),
                Character::new_cop(pos2(-0.25, 0.0))],
            tolerance: 0.1,
            queue: VecDeque::new(),
         };
         res.recompute_graph();
         res
    }

    pub fn robber(&self) -> &Character {
        & self.characters[0]
    }

    pub fn active_cops(&self) -> impl Iterator<Item = &Character> {
        self.characters[1..].iter().filter(|c| c.on_node)
    }

    fn update_min_cop_dist(&mut self) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        min_cop_dist.clear();
        min_cop_dist.resize(self.map.len(), usize::MAX);
        for cop in self.active_cops() {
            for (this, curr_min) in cop.distances.iter().zip(min_cop_dist.iter_mut()) {
                if this < curr_min {
                    *curr_min = *this;
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
    }

    fn update_dist_to_outside_hull(&mut self) {
        let in_hull = &self.in_convex_cop_hull;
        let mut hull_depth = std::mem::take(&mut self.dist_to_outside_hull);
        hull_depth.resize(self.map.len(), usize::MAX);
        for (depth, &in_) in hull_depth.iter_mut().zip(in_hull.iter()) {
            *depth = if in_ == InSet::No { 0 } else { usize::MAX }
        }


        let mut queue = std::mem::take(&mut self.queue);
        queue.clear();
        for (node, &in_set) in in_hull.iter().enumerate() {
            if in_set == InSet::Yes && self.map.neighbors(node).iter().any(|&n| in_hull[n] == InSet::No) {
                queue.push_back(node);
                hull_depth[node] = 1;
            }
        }
        self.map.calc_distances_to(&mut queue, &mut hull_depth);
        self.dist_to_outside_hull = hull_depth;
        self.queue = queue;
    }

    fn update_convex_cop_hull(&mut self) {
        self.queue.clear();

        let in_hull = &mut self.in_convex_cop_hull;
        in_hull.clear();
        in_hull.resize(self.map.len(), InSet::Perhaps);
        for i in 1..self.characters.len() {
            let cop_i = &self.characters[i];
            if !cop_i.on_node {
                continue;
            }
            for cop_j in self.characters[(i + 1)..].iter().filter(|c| c.on_node) {                
                //walk from cop i to cop j on all shortest paths
                in_hull[cop_i.nearest_node] = InSet::NewlyAdded;
                self.queue.push_back(cop_i.nearest_node);
                while let Some(node) = self.queue.pop_front() {
                    let curr_dist_to_j = cop_j.distances[node];
                    for &neigh in self.map.neighbors(node) {
                        if cop_j.distances[neigh] < curr_dist_to_j && in_hull[neigh] != InSet::NewlyAdded {
                            in_hull[neigh] = InSet::NewlyAdded;
                            self.queue.push_back(neigh);
                        }
                    }
                }
                //change these paths from InSet::NewlyAdded to InSet::Yes
                //(to allow new paths to go through the current one)
                self.queue.push_back(cop_i.nearest_node);
                in_hull[cop_i.nearest_node] = InSet::Yes;
                self.map.recolor_region((InSet::NewlyAdded, InSet::Yes), in_hull, &mut self.queue);
            }
        }
        //color outside as InSet::No (note: this might miss some edge cases; best to not place cops at rim)
        for p in self.extreme_vertices {
            if in_hull[p] == InSet::Perhaps {
                in_hull[p] = InSet::No;
                self.queue.push_back(p);
            }
        }
        self.map.recolor_region((InSet::Perhaps, InSet::No), in_hull, &mut self.queue);

        //color remaining InSet::Perhaps as InSet::Yes
        for x in in_hull {
            if *x == InSet::Perhaps {
                *x = InSet::Yes;
            }
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

        SidePanel::left("left_panel").show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.heading("Optionen");
                widgets::global_dark_light_mode_buttons(ui);

                //adjust underlying graph
                ui.collapsing("Form", |ui| {
                    let prev_shape = self.map_shape;
                    let is_n_gon = matches!(self.map_shape, GraphShape::RegularPolygon(_));
                    if ui.add(RadioButton::new(is_n_gon, "ReguläresPolygon")).clicked() {                        
                        self.map_shape = GraphShape::RegularPolygon(6);
                    }
                    if let GraphShape::RegularPolygon(n) = &mut self.map_shape {
                        ui.add(egui::Slider::new(n, 3..=10).text("Seiten"));
                    }
                    ui.radio_value(&mut self.map_shape, GraphShape::Random, "Zufallsverteilt");
                    if prev_shape != self.map_shape {
                        self.recompute_graph();
                    }
                });
                { //adjust radius
                    let prev_radius = self.map_radius;    
                    ui.add(egui::Slider::new(&mut self.map_radius, 1..=100).text("Radius"));
                    if prev_radius != self.map_radius {
                        self.recompute_graph();
                    }
                }
                //adjust nr of cops
                ui.horizontal(|ui| {
                    if ui.button("- Cop").clicked() && self.characters.len() > 1 {
                        self.characters.pop();
                    }
                    if ui.button("+ Cop").clicked() {
                        let mut new = Character::new_cop(Pos2::ZERO);
                        new.update(self.tolerance, &self.map, &mut self.queue);
                        self.characters.push(new);
                    }
                });

                //settings to draw extra information
                ui.radio_value(&mut self.show_robber_closer, true, 
                    "markiere für Räuber\n nähere Knoten");
                if self.show_robber_closer {
                    self.show_escapeable_nodes = false;
                }
                ui.radio_value(&mut self.show_escapeable_nodes, true, 
                    "zeige Punkte näher an\n Hüllenrand als an Cops");
                if self.show_escapeable_nodes {
                    self.show_robber_closer = false;
                }
                ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige \"Konvexe Hülle\"\n um Cops"));
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            let (response, painter) = ui.allocate_painter(
                Vec2::new(ui.available_width(), ui.available_height()), Sense::hover());            

            let (to_screen, scale) = {
                let from = Rect::from_min_max(pos2(-1.05, -1.05), pos2(1.05, 1.05));

                let rect_len = f32::min(response.rect.height(), response.rect.width());
                let to_middle = (response.rect.width() - rect_len) / 2.0;
                let screen_min = response.rect.min + vec2(to_middle, 0.0);
                let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

                let to_screen = emath::RectTransform::from_to(from, to);
                let scale = f32::min(rect_len / self.map_radius as f32 * 0.015, 4.0);
                (to_screen, scale)
            };
            
            let edge_stroke = Stroke::new(scale, Color32::from_rgb(150, 150, 150));
            for (&v1, &v2) in self.map.edges() {                   
                let edge = [
                    to_screen.transform_pos(v1), 
                    to_screen.transform_pos(v2)];
                let line = Shape::LineSegment { points: edge, stroke: edge_stroke };
                painter.add(line);
            }

            if self.show_convex_hull {
                let hull_color = Color32::from_rgb(100, 100, 230);
                for (&in_hull, &pos) in self.in_convex_cop_hull.iter().zip(self.map.positions()) {
                    if in_hull == InSet::Yes  {
                        let draw_pos = to_screen.transform_pos(pos);
                        let marker_circle = Shape::circle_filled(draw_pos, scale * 10.0, hull_color);
                        painter.add(marker_circle);
                    }
                }
            }
            if self.show_robber_closer {                
                let robber_closer_color = Color32::from_rgb(80, 210, 80);
                let dist_vs = self.robber().distances.iter().zip(self.min_cop_dist.iter());
                for ((r_dist, c_dist), &pos) in dist_vs.zip(self.map.positions()) {
                    if r_dist < c_dist  {
                        let draw_pos = to_screen.transform_pos(pos);
                        let marker_circle = Shape::circle_filled(draw_pos, scale * 4.0, robber_closer_color);
                        painter.add(marker_circle);
                    }
                }
            }
            if self.show_escapeable_nodes {
                let escapable_color = Color32::from_rgb(150, 210, 50);
                let dist_vs = self.dist_to_outside_hull.iter().zip(self.min_cop_dist.iter());
                for ((o_dist, c_dist), &pos) in dist_vs.zip(self.map.positions()) {
                    if o_dist < c_dist  {
                        let draw_pos = to_screen.transform_pos(pos);
                        let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, escapable_color);
                        painter.add(marker_circle);
                    }
                }
            }

            for (i, character) in self.characters.iter_mut().enumerate() {                
                let character_size = f32::max(8.0, scale * 8.0);
                let marker_size = character_size + 4.0 * scale;

                let real_screen_pos = to_screen.transform_pos(character.pos);                
                let draw_pos = if character.on_node {
                    let nearest_node_pos = to_screen.transform_pos(self.map.positions()[character.nearest_node]);
                    let marker_circle = Shape::circle_stroke(nearest_node_pos, marker_size, edge_stroke);
                    painter.add(marker_circle);
                    nearest_node_pos
                }
                else {
                    real_screen_pos
                };

                let point_rect = Rect::from_center_size(draw_pos, vec2(marker_size, marker_size));
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
            if self.show_convex_hull || self.show_escapeable_nodes || self.show_robber_closer {
                self.update_min_cop_dist();
            }
            if self.show_convex_hull || self.show_escapeable_nodes {
                self.update_convex_cop_hull();
            }
            if self.show_escapeable_nodes {
                self.update_dist_to_outside_hull();
            }
        });
    }
}
