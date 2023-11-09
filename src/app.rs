
use std::collections::VecDeque;

use egui::*;

use super::graph;



#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InSet { No, Perhaps, Yes, NewlyAdded }

pub enum GraphShape { Hexagon, }

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
    fn new(is_cop: bool, pos: Pos2) -> Character {
        Character { is_cop, on_node: false, nearest_node: 0, pos, distances: Vec::new() }
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

        if change || self.distances.len() != map.len() {
            queue.clear();
            queue.push_back(self.nearest_node);
            self.distances.clear();
            self.distances.resize(map.len(), usize::MAX);
            self.distances[self.nearest_node] = 0;
            map.calc_distances_to(queue, &mut self.distances);
        }
    }
}

pub struct State {
    map: graph::Graph,
    map_shape: GraphShape,
    map_radius: usize, //(approx.) shortest path length from origin to rim

    extreme_points: [usize; 4], //indices of vertices with extreme coodinates

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

    fn compute_extreme_points(points: &[Pos2]) -> [usize; 4] {
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

    fn resize(&mut self, radius: usize) {
        let map = graph::Graph::new_plane_tiles_hexagon(radius, Pos2::new(1.0, 1.0), 1.0);
        self.map_shape = GraphShape::Hexagon;
        self.map = map;
        self.map_radius = radius;      
        for char in &mut self.characters {
            char.nearest_node = 0;
            char.update(self.tolerance, &self.map, &mut self.queue);
        }
        self.extreme_points = Self::compute_extreme_points(self.map.nodes());
        self.update_min_cop_dist();
        self.update_convex_cop_hull();
        self.update_dist_to_outside_hull();
    }

    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = State { 
            map: graph::Graph::empty(),
            map_shape: GraphShape::Hexagon,
            map_radius: 0,

            extreme_points: [0; 4],

            in_convex_cop_hull: Vec::new(),
            dist_to_outside_hull: Vec::new(),
            min_cop_dist: Vec::new(),

            show_robber_closer: true,
            show_convex_hull: true,
            show_escapeable_nodes: true,
            
            characters: vec![
                Character::new(false, pos2(0.1, 0.1)),
                Character::new(true, pos2(0.9, 1.0)),
                Character::new(true, pos2(1.1, 1.0))],
            tolerance: 0.1,
            queue: VecDeque::new(),
         };
         res.resize(8);
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
        for p in self.extreme_points {
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
                    let mut new = Character::new(true, pos2(1.0, 1.0));
                    new.update(self.tolerance, &self.map, &mut self.queue);
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

            let (to_screen, scale) = {
                let from = Rect{ min: Pos2::ZERO, max: Pos2::new(2.0, 2.0) };

                let rect_len = f32::min(response.rect.height(), response.rect.width());
                let to_middle = (response.rect.width() - rect_len) / 2.0;
                let screen_min = response.rect.min + vec2(to_middle, 0.0);
                let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

                let to_screen = emath::RectTransform::from_to(from, to);
                let scale = rect_len / self.map_radius as f32 * 0.015;
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
                for (&in_hull, &pos) in self.in_convex_cop_hull.iter().zip(self.map.nodes()) {
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
                for ((r_dist, c_dist), &pos) in dist_vs.zip(self.map.nodes()) {
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
                for ((o_dist, c_dist), &pos) in dist_vs.zip(self.map.nodes()) {
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
                    let nearest_node_pos = to_screen.transform_pos(self.map.nodes()[character.nearest_node]);
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
            self.update_convex_cop_hull();
            self.update_min_cop_dist();
            self.update_dist_to_outside_hull();
        });
    }
}
