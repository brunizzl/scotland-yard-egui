
use std::collections::VecDeque;

use egui::{*, epaint::TextShape, text::LayoutJob};

use crate::{ graph::Embedding2D, graph, app::* };


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InSet { No, Perhaps, Yes, NewlyAdded }
impl InSet {
    pub fn yes(self) -> bool { 
        debug_assert!(matches!(self, InSet::No | InSet::Yes));
        self == InSet::Yes 
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RobberInfo { None, EscapableNodes, NearNodes, SmallRobberDist(usize) }

impl RobberInfo {
    fn scale_small_dist_with_radius(dist: usize, radius: usize) -> usize {
        (dist * radius) / 100
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum GraphShape { RegularPolygon(usize), Random, Debug }

#[derive(Clone, Copy, PartialEq)]
pub enum DrawNumbers { None, Indices, RobberAdvantage, MinCopDist }

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Character {
    is_cop: bool, //else is robber
    on_node: bool,
    pub nearest_node: usize,
    pos: Pos2,
    distances: Vec<isize>,
}

impl Character {
    fn new(is_cop: bool, pos: Pos2) -> Self {
        Character { is_cop, on_node: false, nearest_node: 0, pos, distances: Vec::new() }
    }

    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, 
    //  change to that neighbor
    //(converges to globally nearest node only for "convex" graphs, 
    //  e.g. planar graphs, where each inside face is convex and the complement of the outside face is convex)
    fn update(&mut self, tolerance: f32, map: &Embedding2D, queue: &mut VecDeque<usize>) {
        let safe_start = if map.len() > self.nearest_node { self.nearest_node } else { 0 };
        let (nearest_node, nearest_dist_sq) = map.find_nearest_node(self.pos, safe_start);
        self.on_node = nearest_dist_sq <= tolerance * tolerance;

        let need_dist_update = self.distances.len() != map.len() || 
            (self.on_node && nearest_node != self.nearest_node);
        if need_dist_update {
            queue.clear();
            queue.push_back(nearest_node);
            self.distances.clear();
            self.distances.resize(map.len(), isize::MAX);
            self.distances[nearest_node] = 0;
            map.calc_distances_to(queue, &mut self.distances);
        }
        self.nearest_node = nearest_node;
    }
    
    //options to draw cops and robber as emojies: 
    //👮🛂🛃👿🚴🏃
    const COP_EMOJI: &str = "👮";
    const ROBBER_EMOJI: &str = "🏃";

    pub fn emoji(&self) -> &str {
        if self.is_cop { Self::COP_EMOJI } else { Self::ROBBER_EMOJI }
    }
}

pub struct State {
    map: Embedding2D,

    //overall map shape: == 2 is line, == 3 triangle, == 4 square, == 5 pentagon etc.
    map_shape: GraphShape,
    map_radius: usize, //(approx.) shortest path length from origin to rim

    extreme_vertices: [usize; 4], //indices of vertices with extreme coodinates

    //state kept for each node in map
    in_convex_cop_hull: Vec<InSet>, 
    min_cop_dist: Vec<isize>, //elementwise minimum of .distance of active cops in self.characters
    muliple_min_dist_cops: Vec<bool>,
    cop_advantage: Vec<isize>,

    robber_info: RobberInfo,
    vertex_info: DrawNumbers,
    show_convex_hull: bool,
    show_cop_voronoi: bool, //mark positions where (at least) two cops have minimum distance
    debug_info: bool,

    //first is robber, rest are cops
    characters: Vec<Character>,
    tolerance: f32, //how close must a character be to a vertex to count as beeing on thet vertex

    queue: VecDeque<usize>, //kept permanentely to reduce allocations when a character update is computed.

    camera: Camera2D,
}

impl State {

    /// really shitty approximation of convex hull.
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
            GraphShape::Random => graph::random_triangulated(self.map_radius, 8),
            GraphShape::Debug => graph::debugging_graph(),
        };
        for char in &mut self.characters {
            char.update(self.tolerance, &self.map, &mut self.queue);
        }
        self.extreme_vertices = Self::compute_extreme_vertices(self.map.positions());
        self.update_min_cop_dist();
        self.update_convex_cop_hull();
        self.update_cop_advantage();
        self.tolerance = f32::min(0.25, 0.75 / self.map_radius as f32);
    }

    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = State { 
            map: Embedding2D::empty(),
            map_shape: GraphShape::RegularPolygon(6),
            map_radius: 6,

            extreme_vertices: [0; 4],

            in_convex_cop_hull: Vec::new(),
            min_cop_dist: Vec::new(),
            muliple_min_dist_cops: Vec::new(),
            cop_advantage: Vec::new(),

            robber_info: RobberInfo::None,
            vertex_info: DrawNumbers::None,
            show_convex_hull: false,
            show_cop_voronoi: false,
            debug_info: false,
            
            characters: vec![
                Character::new(false, Pos2::ZERO),
                Character::new(true, pos2(0.25, 0.0)),
                //Character::new(true, pos2(-0.25, 0.0))
                ],
            tolerance: 0.25,
            queue: VecDeque::new(),

            camera: Camera2D::new(),
         };
         res.recompute_graph();
         res
    }

    pub fn robber(&self) -> Option<&Character> {
        self.characters.first()
    }

    pub fn active_cops(&self) -> impl Iterator<Item = &Character> {
        let start = usize::min(1, self.characters.len());
        self.characters[start..].iter().filter(|c| c.on_node)
    }

    fn update_min_cop_dist(&mut self) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        let mut multiple = std::mem::take(&mut self.muliple_min_dist_cops);
        min_cop_dist.clear();
        multiple.clear();
        {
            multiple.resize(self.map.len(), false);
            let mut active_cops = self.active_cops();
            if let Some(cop) = active_cops.next() {
                min_cop_dist.clone_from(&cop.distances);
            }
            for cop in active_cops {
                let tripels = itertools::izip!(
                    cop.distances.iter(), min_cop_dist.iter_mut(), multiple.iter_mut());
                for (this, curr_min, shared) in tripels {
                    if this < curr_min {
                        *curr_min = *this;
                        *shared = false;
                    }
                    else if this == curr_min {
                        *shared = true;
                    }
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
        self.muliple_min_dist_cops = multiple;
    }

    fn update_cop_advantage(&mut self) {
        let in_cop_hull = &self.in_convex_cop_hull;
        let mut advantage = std::mem::take(&mut self.cop_advantage);
        advantage.resize(self.map.len(), isize::MAX);        
        let mut queue = std::mem::take(&mut self.queue);
        queue.clear();

        let zipped = itertools::izip!(0.., 
            in_cop_hull.iter(), 
            advantage.iter_mut(), 
            self.min_cop_dist.iter(), 
            self.map.neighbors());
        for (node, &in_hull, adv, &cop_dist, mut neighs) in zipped {
            if !in_hull.yes() && neighs.any(|n| in_cop_hull[n].yes()) {
                queue.push_back(node);
            }
            *adv = if in_hull.yes() { isize::MAX } else { -cop_dist };
        }
        self.map.calc_distances_to(&mut queue, &mut advantage);
        self.cop_advantage = advantage;
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
                    for neigh in self.map.neighbors_of(node) {
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

    pub fn draw_menu(&mut self, ui: &mut Ui) {
        if ui.button("🏠 Position").clicked() {
            self.camera.reset();
        }
        //adjust underlying graph
        ui.collapsing("Form", |ui| {
            let prev_shape = self.map_shape;
            let is_n_gon = matches!(self.map_shape, GraphShape::RegularPolygon(_));
            if ui.add(RadioButton::new(is_n_gon, "ReguläresPolygon")).clicked() {                        
                self.map_shape = GraphShape::RegularPolygon(6);
            }
            if let GraphShape::RegularPolygon(n) = &mut self.map_shape {
                super::add_drag_value(ui, n, "Seiten: ", 3, 10);
            }
            ui.radio_value(&mut self.map_shape, GraphShape::Random, "Zufallsverteilt");
            if self.map_shape == GraphShape::Random {
                if ui.button("neu berechnen").clicked() {
                    self.recompute_graph();
                }
            }
            ui.radio_value(&mut self.map_shape, GraphShape::Debug, "Debugging");
            if prev_shape != self.map_shape {
                self.recompute_graph();
            }
            if add_drag_value(ui, &mut self.map_radius, "Radius: ", 0, 100) {
                self.recompute_graph();
            }
        });
        ui.collapsing("Grün", |ui|{                    
            //settings to draw extra information
            ui.radio_value(&mut self.robber_info, RobberInfo::None, 
                "keine Marker");
            ui.radio_value(&mut self.robber_info, RobberInfo::NearNodes, 
                "markiere für Räuber\nnähere Knoten");
            ui.radio_value(&mut self.robber_info, RobberInfo::EscapableNodes, 
                "markiere Punkte mit\ndirekter Fluchtoption");
            let robber_dist_button = egui::RadioButton::new(
                matches!(self.robber_info, RobberInfo::SmallRobberDist(_)), 
                "markiere Punkte nah\nan Räuber");
            if ui.add(robber_dist_button).clicked() {
                self.robber_info = RobberInfo::SmallRobberDist(50);
            }
            if let RobberInfo::SmallRobberDist(bnd) = &mut self.robber_info {
                add_drag_value(ui, bnd, "% Radius: ", 1, 100);
            }
            if let (RobberInfo::SmallRobberDist(bnd), Some(r)) = (self.robber_info, self.robber()) {
                let r_pos = self.map.positions()[r.nearest_node];
                let mut max_dist = f32::MIN;
                let mut min_dist = f32::MAX;
                let bnd = RobberInfo::scale_small_dist_with_radius(bnd, self.map_radius);
                for (&dist, &pos) in r.distances.iter().zip(self.map.positions()) {
                    if (dist as usize) == bnd {
                        let new_dist = (r_pos - pos).length();
                        max_dist = f32::max(max_dist, new_dist);
                        min_dist = f32::min(min_dist, new_dist);
                    }
                }
                ui.label(format!("min dist:   {}\nmax dist:   {}\nmin / max: {}", 
                    min_dist, max_dist, min_dist / max_dist));
            }
        });
        ui.collapsing("Zahlen (Radius < 20)", |ui|{
            ui.radio_value(&mut self.vertex_info, DrawNumbers::None, 
                "Keine");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::Indices, 
                "Knotenindizes");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::RobberAdvantage, 
                "Räubervorteil");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::MinCopDist, 
                "minimaler Cop Abstand");
        });
        //adjust nr of currently computed figures
        ui.horizontal(|ui| {
            let (minus_emoji, plus_emoji) = match self.characters.len() {
                0 => ("🚫", Character::ROBBER_EMOJI),
                1 => (Character::ROBBER_EMOJI, Character::COP_EMOJI),
                _ => (Character::COP_EMOJI, Character::COP_EMOJI),
            };
            let minus_text = format!("- Figur ({minus_emoji})");
            let plus_text = format!("+ Figur ({plus_emoji})");
            if ui.button(minus_text).clicked() {
                self.characters.pop();
            }
            if ui.button(plus_text).clicked() {
                let is_cop = self.characters.len() > 0;
                let mut new = Character::new(is_cop, Pos2::ZERO);
                new.update(self.tolerance, &self.map, &mut self.queue);
                self.characters.push(new);
            }
        });
        ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige \"Konvexe Hülle\"\n um Cops"));
        ui.add(Checkbox::new(&mut self.show_cop_voronoi, "zeige Punkte mit\nmehreren nächsten Cops"));
        ui.add(Checkbox::new(&mut self.debug_info, "bunte Kanten"));
    }

    fn draw_edges(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {

        let grey_stroke = Stroke::new(scale, GREY);
        let edge_stroke = |v1: usize, v2: usize| if self.debug_info {
            let seed = v1 * v2 + 100;
            let mut gen = crate::rand::LCG::new(seed as u64);
            gen.waste(5);
            let mut rnd = || (64 + (gen.next() % 128)) as u8;
            Stroke::new(scale, Color32::from_rgb(rnd(), rnd(), rnd()))
        }
        else {
            grey_stroke
        };

        self.map.for_each_edge(|v1, p1, v2, p2| { 
            let edge = [
                to_screen.transform_pos(p1), 
                to_screen.transform_pos(p2)];
            let line = Shape::LineSegment { points: edge, stroke: edge_stroke(v1, v2) };
            painter.add(line);
        });
    }

    fn draw_convex_cop_hull(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        for (&in_hull, &pos) in self.in_convex_cop_hull.iter().zip(self.map.positions()) {
            if in_hull.yes()  {
                let draw_pos = to_screen.transform_pos(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 9.0, LIGHT_BLUE);
                painter.add(marker_circle);
            }
        }
    }

    fn draw_green_circles(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {     
        let draw_circle_at = |pos: Pos2|{
            let draw_pos = to_screen.transform_pos(pos);
            let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
            painter.add(marker_circle);
        };
        match (self.robber_info, self.robber()) {
            (RobberInfo::NearNodes, Some(r)) => 
                for (r_dist, c_dist, &pos) in 
                itertools::izip!(r.distances.iter(), self.min_cop_dist.iter(), self.map.positions()) {
                    if r_dist < c_dist {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::EscapableNodes, _) => 
                for (&adv, &pos) in self.cop_advantage.iter().zip(self.map.positions()) {
                    if adv < -1 {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::SmallRobberDist(bnd), Some(r)) => {
                let bnd = RobberInfo::scale_small_dist_with_radius(bnd, self.map_radius);
                for (&dist, &pos) in r.distances.iter().zip(self.map.positions()) {
                    if (dist as usize) <= bnd {
                        draw_circle_at(pos);
                    }
                }
            },
            _ => {},
        }
    }

    fn draw_robber_info(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        if let (RobberInfo::NearNodes, Some(r)) = (self.robber_info, self.robber()) { 
            let dist_vs = r.distances.iter().zip(self.min_cop_dist.iter());
            for ((r_dist, c_dist), &pos) in dist_vs.zip(self.map.positions()) {
                if r_dist < c_dist  {
                    let draw_pos = to_screen.transform_pos(pos);
                    let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
                    painter.add(marker_circle);
                }
            }
        }
        if self.robber_info == RobberInfo::EscapableNodes {
            for (&adv, &pos) in self.cop_advantage.iter().zip(self.map.positions()) {
                if adv < -1  {
                    let draw_pos = to_screen.transform_pos(pos);
                    let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
                    painter.add(marker_circle);
                }
            }
        }
    }

    fn draw_numbers(&self, ui: &mut Ui, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        let font = FontId::proportional(scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode { WHITE } else { BLACK };
        for (i, &pos) in self.map.positions().iter().enumerate() {
            let txt = match self.vertex_info {
                DrawNumbers::Indices => { i.to_string() }
                DrawNumbers::MinCopDist => { self.min_cop_dist[i].to_string() }
                DrawNumbers::None => { panic!() }
                DrawNumbers::RobberAdvantage => { (-1 -self.cop_advantage[i]).to_string() }
            };
            let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * scale);
            layout_job.halign = Align::Center;
            let galley = ui.fonts(|f| f.layout_job(layout_job));
            let screen_pos = to_screen.transform_pos(pos);
            let text = Shape::Text(TextShape::new(screen_pos, galley));
            painter.add(text);
        }
    }

    fn draw_cop_voronoi(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        for (&multiple, &pos) in self.muliple_min_dist_cops.iter().zip(self.map.positions()) {
            if multiple  {
                let draw_pos = to_screen.transform_pos(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 5.0, RED);
                painter.add(marker_circle);
            }
        }
    }

    fn draw_characters(&mut self, ui: &mut Ui, response: &Response, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        for (i, character) in self.characters.iter_mut().enumerate() {                
            let character_size = f32::max(8.0, scale * 8.0);

            let real_screen_pos = to_screen.transform_pos(character.pos);    
            let node_pos = self.map.positions()[character.nearest_node];
            let node_screen_pos = to_screen.transform_pos(node_pos);
            let draw_screen_pos = if character.on_node { node_screen_pos } else { real_screen_pos };

            let rect_len = 3.0 * character_size + self.tolerance;
            let point_rect = Rect::from_center_size(draw_screen_pos, vec2(rect_len, rect_len));
            let character_id = response.id.with(i);
            let point_response = ui.interact(point_rect, character_id, Sense::drag());
            if point_response.dragged_by(PointerButton::Primary) {
                character.pos = to_screen.inverse().transform_pos(real_screen_pos + point_response.drag_delta());
                character.update(self.tolerance, &self.map, &mut self.queue);
            }
            if point_response.drag_released_by(PointerButton::Primary) && character.on_node {
                character.pos = node_pos; //snap actual character postion to node where he was dragged
            }

            let fill_color = if character.is_cop { COP_BLUE } else { ROBBER_RED };
            let character_circle = Shape::circle_filled(draw_screen_pos, character_size, fill_color);
            painter.add(character_circle);
            if character.on_node {
                let stroke_color = if character.is_cop { BLUE_GLOW } else { RED_GLOW };
                let stroke = Stroke::new(scale * 3.0, stroke_color);
                let marker_circle = Shape::circle_stroke(node_screen_pos, character_size, stroke);
                painter.add(marker_circle);
            }
            //draw emoji
            let font = FontId::proportional(character_size * 2.0);
            let emoji_pos = draw_screen_pos - character_size * vec2(0.0, 1.3);
            let emoji_str = character.emoji().to_string();
            let mut layout_job = LayoutJob::simple(emoji_str, font, WHITE, 100.0);
            layout_job.halign = Align::Center;
            let galley = ui.fonts(|f| f.layout_job(layout_job));
            let emoji = Shape::Text(TextShape::new(emoji_pos, galley));
            painter.add(emoji);
        }
    }

    fn maybe_update(&mut self) {
        let require_update = self.show_convex_hull 
            || self.show_cop_voronoi
            || self.robber_info != RobberInfo::None 
            || self.vertex_info != DrawNumbers::None;
        if require_update {
            self.update_min_cop_dist();
            self.update_convex_cop_hull();
            self.update_cop_advantage();
        }
    }

    /// fst maps graph coordinates to screen, snd defines scale to draw edges etc. at
    fn build_to_screen(&self, response: &Response) -> (emath::RectTransform, f32) {
        let camera = self.camera;
        let rect_min = vec2(-1.0, -1.0) / camera.zoom;
        let rect_max = vec2(1.0, 1.0) / camera.zoom;
        let from = Rect::from_min_max(rect_min.to_pos2(), rect_max.to_pos2());

        let rect_len = f32::min(response.rect.height(), response.rect.width());
        let to_middle = (response.rect.width() - rect_len) / 2.0;
        let screen_min = response.rect.min + vec2(to_middle, 0.0) + camera.offset;
        let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

        let to_screen = emath::RectTransform::from_to(from, to);
        let scale = f32::min(rect_len / self.map_radius as f32 * 0.015, 4.0);
        (to_screen, scale * camera.zoom)
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {
        self.maybe_update();

        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover());   
        self.camera.update_cursor_centered(ui, &response); 

        let (to_screen, scale) = self.build_to_screen(&response);

        self.draw_edges(&painter, to_screen, scale);
        if self.show_convex_hull {
            self.draw_convex_cop_hull(&painter, to_screen, scale);
        }
        self.draw_green_circles(&painter, to_screen, scale);
        self.draw_robber_info(&painter, to_screen, scale);

        if self.map_radius < 20 && self.vertex_info != DrawNumbers::None {
            self.draw_numbers(ui, &painter, to_screen, scale);
        }
        if self.show_cop_voronoi {
            self.draw_cop_voronoi(&painter, to_screen, scale);
        }
        self.draw_characters(ui, &response, &painter, to_screen, scale);
    }
}
