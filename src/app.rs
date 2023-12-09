
use std::collections::VecDeque;

use egui::*;

use crate::{graph::{Embedding2D, Embedding3D, InSet, EdgeList}, geo};

mod dim2;
mod dim3;
mod cop_robber_algos;


pub const GREY: Color32 = Color32::from_rgb(130, 130, 150);
pub const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
pub const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
pub const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
pub const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
pub const RED: Color32 = Color32::from_rgb(230, 50, 50);

/// returns if val was changed
fn add_drag_value(ui: &mut Ui, val: &mut usize, name: &str, min: usize, max: usize) -> bool {    
    ui.horizontal(|ui| { 
        let prev = *val;  
        ui.label(name);                     
        if ui.button(" - ").clicked() && prev > min {
            *val -= 1;
        }                     
        ui.add(DragValue::new(val).clamp_range(min..=max));   
        if ui.button(" + ").clicked() && prev < max {
            *val += 1;
        }
        prev != *val
    }).inner
}

pub struct CharData {
    color: Color32,
    glow: Color32,
    emoji: &'static str,
}
//options to draw cops and robber as emojies: 
//👮🛂🛃👿🚴🏃
pub const COP: CharData = CharData {
    color: Color32::from_rgb(10, 50, 170),
    glow: Color32::from_rgb(60, 120, 235),
    emoji: "👮",
};
pub const ROBBER: CharData = CharData {
    color: Color32::from_rgb(170, 40, 40),
    glow: Color32::from_rgb(235, 120, 120),
    emoji: "🏃",
};

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Character {
    data: &'static CharData,
    nearest_node: usize,
    distances: Vec<isize>,
    pos: Pos2,
    on_node: bool,
    dragging: bool, //currently beeing dragged by mouse cursor
}

impl Character {
    fn new(is_cop: bool, pos: Pos2) -> Self {
        let data = if is_cop { &COP } else { &ROBBER };
        //dragging set to true snaps to node next update
        Character { data, nearest_node: 0, distances: Vec::new(), pos, on_node: false, dragging: true }
    }

    fn snap_to_node(&mut self) {
        //if not currently held by mouse, this will generate a falling edge and a falling edge snaps to node
        //in self.drag_and_draw
        self.dragging = true;
    }

    pub fn draw_large_at(&self, draw_pos: Pos2, painter: &Painter, ui: &Ui, scale: f32) {
        //draw circles
        let character_circle = Shape::circle_filled(draw_pos, scale, self.data.color);
        painter.add(character_circle);
        if self.on_node {
            let stroke = Stroke::new(scale * 0.375, self.data.glow);
            let marker_circle = Shape::circle_stroke(draw_pos, scale, stroke);
            painter.add(marker_circle);
        }
        //draw emoji
        let font = FontId::proportional(scale * 2.0);
        let emoji_pos = draw_pos - scale * vec2(0.0, 1.3);
        let emoji_str = self.data.emoji.to_string();
        let mut layout_job = text::LayoutJob::simple(emoji_str, font, WHITE, 100.0);
        layout_job.halign = Align::Center;
        let galley = ui.fonts(|f| f.layout_job(layout_job));
        let emoji = Shape::Text(epaint::TextShape::new(emoji_pos, galley));
        painter.add(emoji);
    }

    pub fn draw_small_at(&self, draw_pos: Pos2, painter: &Painter, scale: f32) {
        let character_size = f32::max(4.0, scale * 4.0);
        let character_circle = Shape::circle_filled(draw_pos, character_size, self.data.glow);
        painter.add(character_circle);
    }

    pub fn drag_and_draw(&mut self, response: &Response, painter: &Painter, ui: &Ui, 
        to_screen: emath::RectTransform, node_pos: Pos2, scale: f32) 
    {       
        let character_size = f32::max(8.0, scale * 8.0);
        let real_screen_pos = to_screen.transform_pos(self.pos);
        let node_screen_pos = to_screen.transform_pos(node_pos);
        let draw_at_node = self.on_node && !self.dragging;
        let draw_screen_pos = if draw_at_node { node_screen_pos } else { real_screen_pos };

        let rect_len = 3.0 * character_size;
        let point_rect = Rect::from_center_size(draw_screen_pos, vec2(rect_len, rect_len));
        let character_id = response.id.with(self as *const Self);
        let point_response = ui.interact(point_rect, character_id, Sense::drag());
        let dragging = point_response.dragged_by(PointerButton::Primary);
        //dragging starts -> update actual position to match drawn position
        if dragging {
            let from_screen = to_screen.inverse();
            let new_screen_pos = draw_screen_pos + point_response.drag_delta();
            self.pos = from_screen.transform_pos(new_screen_pos);
        }
        //test if character was just released. doing this ourselfs allows to simulate release whenever we like
        //(e.g. just set dragging to true and we snap to position)
        if !dragging && self.dragging && self.on_node {
            self.pos = node_pos; //snap actual character postion to node where he was dragged
        }
        self.dragging = dragging;

        self.draw_large_at(draw_screen_pos, painter, ui, character_size);
    }

    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, 
    //  change to that neighbor
    //(converges to globally nearest node only for "convex" graphs, 
    //  e.g. planar graphs, where each inside face is convex and the complement of the outside face is convex)
    fn update_2d(&mut self, tolerance: f32, map: &Embedding2D, queue: &mut VecDeque<usize>) {
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

    //assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    fn update_3d(&mut self, tolerance: f32, map: &Embedding3D, to_2d: &geo::Project3To2, 
        vertex_visible: &[bool], queue: &mut VecDeque<usize>) 
    {
        if self.dragging {
            let safe_start = if map.nr_vertices() > self.nearest_node { self.nearest_node } else { 0 };
            let potential = |v:usize, v_pos| { 
                let dist_2d = (to_2d.project_pos(v_pos) - self.pos).length_sq();
                let backface_penalty = 10.0 * (!vertex_visible[v]) as isize as f32;
                dist_2d + backface_penalty
            };
            let (nearest_node, nearest_dist_sq) = map.find_local_minimum(potential, safe_start);
            self.on_node = nearest_dist_sq <= tolerance * tolerance;
    
            let need_dist_update = self.distances.len() != map.nr_vertices() || 
                (self.on_node && nearest_node != self.nearest_node);
            if need_dist_update {
                queue.clear();
                queue.push_back(nearest_node);
                self.distances.clear();
                self.distances.resize(map.nr_vertices(), isize::MAX);
                self.distances[nearest_node] = 0;
                map.calc_distances_to(queue, &mut self.distances);
            }
            self.nearest_node = nearest_node;
        }
    }
}

fn draw_character_buttons(ui: &mut Ui, characters: &mut Vec<Character>) {
    ui.horizontal(|ui| {
        let (minus_emoji, plus_emoji) = match characters.len() {
            0 => ("🚫", ROBBER.emoji),
            1 => (ROBBER.emoji, COP.emoji),
            _ => (COP.emoji, COP.emoji),
        };
        let minus_text = format!("- Figur ({minus_emoji})");
        let plus_text = format!("+ Figur ({plus_emoji})");
        if ui.button(minus_text).clicked() {
            characters.pop();
        }
        if ui.button(plus_text).clicked() {
            let is_cop = characters.len() > 0;
            characters.push(Character::new(is_cop, Pos2::ZERO));
        }
    });
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RobberInfo { None, EscapableNodes, NearNodes, SmallRobberDist(usize) }

impl RobberInfo {
    fn scale_small_dist_with_radius(dist: usize, radius: usize) -> usize {
        (dist * radius) / 100
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum DrawNumbers { None, Indices, RobberAdvantage, MinCopDist }

pub struct InfoState {
    //state kept for each node in map
    pub in_convex_cop_hull: Vec<InSet>, 
    pub min_cop_dist: Vec<isize>, //elementwise minimum of .distance of active cops in self.characters
    pub muliple_min_dist_cops: Vec<bool>,
    pub cop_advantage: Vec<isize>,

    pub robber_info: RobberInfo,
    pub vertex_info: DrawNumbers,
    pub show_convex_hull: bool,
    pub show_cop_voronoi: bool, //mark positions where (at least) two cops have minimum distance
    pub debug_info: bool,
    
    pub characters: Vec<Character>,

    queue: VecDeque<usize>, //kept permanentely to reduce allocations when a character update is computed.
}

impl InfoState {

    fn new() -> Self {
        Self { 
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
            queue: VecDeque::new(),
        }
    }

    fn robber(&self) -> Option<&Character> {
        self.characters.first()
    }

    fn active_cops(&self) -> impl Iterator<Item = &Character> {
        let start = usize::min(1, self.characters.len());
        self.characters[start..].iter().filter(|&c| c.on_node)
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) {
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
        draw_character_buttons(ui, &mut self.characters);
        ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige \"Konvexe Hülle\"\n um Cops"));
        ui.add(Checkbox::new(&mut self.show_cop_voronoi, "zeige Punkte mit\nmehreren nächsten Cops"));
        ui.add(Checkbox::new(&mut self.debug_info, "bunte Kanten"));
    }

    fn update_min_cop_dist(&mut self, edges: &EdgeList) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        let mut multiple = std::mem::take(&mut self.muliple_min_dist_cops);
        min_cop_dist.clear();
        multiple.clear();
        {
            multiple.resize(edges.nr_vertices(), false);
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

    fn update_cop_advantage(&mut self, edges: &EdgeList) {
        let in_cop_hull = &self.in_convex_cop_hull;
        let mut advantage = std::mem::take(&mut self.cop_advantage);
        advantage.resize(edges.nr_vertices(), isize::MAX);        
        let mut queue = std::mem::take(&mut self.queue);
        queue.clear();

        let zipped = itertools::izip!(0.., 
            in_cop_hull.iter(), 
            advantage.iter_mut(), 
            self.min_cop_dist.iter(), 
            edges.neighbors());
        for (node, &in_hull, adv, &cop_dist, mut neighs) in zipped {
            if !in_hull.yes() && neighs.any(|n| in_cop_hull[n].yes()) {
                queue.push_back(node);
            }
            *adv = if in_hull.yes() { isize::MAX } else { -cop_dist };
        }
        edges.calc_distances_to(&mut queue, &mut advantage);
        self.cop_advantage = advantage;
        self.queue = queue;
    }

    fn update_convex_cop_hull(&mut self, edges: &EdgeList, extreme_vertices: impl Iterator<Item = usize>) {
        self.queue.clear();

        let in_hull = &mut self.in_convex_cop_hull;
        in_hull.clear();
        in_hull.resize(edges.nr_vertices(), InSet::Perhaps);
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
                    for neigh in edges.neighbors_of(node) {
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
                edges.recolor_region((InSet::NewlyAdded, InSet::Yes), in_hull, &mut self.queue);
            }
        }
        //color outside as InSet::No (note: this might miss some edge cases; best to not place cops at rim)
        for p in extreme_vertices {
            if in_hull[p] == InSet::Perhaps {
                in_hull[p] = InSet::No;
                self.queue.push_back(p);
            }
        }
        edges.recolor_region((InSet::Perhaps, InSet::No), in_hull, &mut self.queue);

        //color remaining InSet::Perhaps as InSet::Yes
        for x in in_hull {
            if *x == InSet::Perhaps {
                *x = InSet::Yes;
            }
        }
    }

    fn maybe_update(&mut self, edges: &EdgeList, extreme_vertices: impl Iterator<Item = usize>) {
        let require_update = self.show_convex_hull 
            || self.show_cop_voronoi
            || self.robber_info != RobberInfo::None 
            || self.vertex_info != DrawNumbers::None;
        if require_update {
            self.update_convex_cop_hull(edges, extreme_vertices);
            self.update_min_cop_dist(edges);
            self.update_cop_advantage(edges);
        }
    }
}


#[derive(Clone, Copy)]
struct Camera2D {
    /// == 1.0 -> no change
    /// < 1.0  -> zoomed out
    /// > 1.0  -> zoomed in
    zoom: f32, 
    /// angle in rad 0.0 == no rotation
    rotation: f32,
    /// offset of center independent of zoom
    offset: Vec2,
}

impl Camera2D {
    fn new() -> Self {
        Self { 
            zoom: 1.0, 
            rotation: 0.0,
            offset: Vec2::new(0.0, 0.0), 
        }
    }

    fn update(&mut self, ui: &mut Ui, response: Option<&Response>) {        
        ui.input(|info| {
            if info.pointer.button_down(PointerButton::Secondary) {
                self.offset += info.pointer.delta();
            }
            self.offset += info.scroll_delta;

            let zoom_delta = info.zoom_delta();
            self.zoom *= zoom_delta;
            if zoom_delta != 1.0 {
                if let (Some(ptr_pos), Some(resp)) = (info.pointer.latest_pos(), response) {
                    //keep fixed point of zoom at mouse pointer
                    let draw_mid = resp.rect.center();
                    let mid_to_ptr = ptr_pos - draw_mid;
                    let mut zoom_center = self.offset - mid_to_ptr;
                    zoom_center *= zoom_delta;
                    self.offset = zoom_center + mid_to_ptr;
                }
            }
            if let Some(drag) = info.multi_touch() {
                self.offset += drag.translation_delta;
                self.rotation += drag.rotation_delta;
            }
        });
    }

    /// zoom changes happen with the cursor position as fixed point, thus 
    /// with zooming we also change the offset
    fn update_cursor_centered(&mut self, ui: &mut Ui, response: &Response) {
        self.update(ui, Some(response));
    }

    /// zoom changes don't change the offset at all
    fn update_screen_centered(&mut self, ui: &mut Ui) {
        self.update(ui, None);
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

pub struct State {
    state_2d: dim2::State,
    state_3d: dim3::State,
    show_2d: bool, //else show 3d
}

impl State {
    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self { 
            state_2d: dim2::State::new(_cc), 
            state_3d: dim3::State::new(_cc), 
            show_2d: true,
        }
    }
}

impl eframe::App for State {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {


        SidePanel::left("left_panel").show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.heading("Optionen");
                widgets::global_dark_light_mode_buttons(ui);
                let button_switch_text = if self.show_2d { "Zeige 3D" } else { "Zeige 2D" };
                if ui.button(button_switch_text).clicked() {
                    self.show_2d = !self.show_2d;
                }

                if self.show_2d {
                    self.state_2d.draw_menu(ui);
                }
                else {
                    self.state_3d.draw_menu(ui);
                }
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            if self.show_2d {
                self.state_2d.draw_graph(ui);
            }
            else {
                self.state_3d.draw_graph(ui);
            }
        });
    }
}