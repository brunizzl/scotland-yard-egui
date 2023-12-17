
use std::collections::VecDeque;

use itertools::{izip, Itertools};

use egui::{*, epaint::TextShape, text::LayoutJob, emath::RectTransform};

use crate::{graph::{Embedding2D, Embedding3D, InSet, EdgeList}, geo::{self, Pos3, Vec3, Project3To2, ToScreen}};

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
fn add_drag_value(ui: &mut Ui, val: &mut isize, name: &str, min: isize, max: isize) -> bool {    
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

pub struct CharacterData {
    color: Color32,
    glow: Color32,
    emoji: &'static str,
    job: &'static str,
}
const fn new_cop(emoji: &'static str) -> CharacterData { 
    CharacterData {
        color: Color32::from_rgb(10, 50, 170),
        glow: Color32::from_rgb(60, 120, 235),
        //alternatives: 👮🛂🛃🐈🔫🚔🐂🍩
        emoji,
        job: "Cop",
    } 
}
pub const COPS: [CharacterData; 7] = [
    new_cop("👮"), 
    new_cop("🍩"), 
    new_cop("🐂"), 
    new_cop("🔫"), 
    new_cop("🚔"), 
    new_cop("🛂"), 
    new_cop("🛃")
];
pub const ROBBER: CharacterData = CharacterData {
    color: Color32::from_rgb(170, 40, 40),
    glow: Color32::from_rgb(235, 120, 120),
    //alternatives: 👿🚴🏃🚶🐀🐢🕵💰
    emoji: "🏃",
    job: "Räuber",
};

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Character {
    data: &'static CharacterData,
    nearest_node: usize,
    last_positions: Vec<usize>,
    distances: Vec<isize>,
    pos2: Pos2, //used in both 3d and 3d map (as cursor can't drag in 3d...)
    pos3: Pos3, //only used in 3d map
    on_node: bool,
    dragging: bool, //currently beeing dragged by mouse cursor
}

impl Character {
    fn new(data: &'static CharacterData, pos2: Pos2) -> Self {
        //dragging set to true snaps to node next update
        Character { 
            data, 
            nearest_node: 0, 
            last_positions: Vec::new(), 
            distances: Vec::new(), 
            pos2, 
            pos3: Pos3::ZERO, 
            on_node: false, 
            dragging: true //causes snap to node next update (as dragging will change to false)
        }
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
        to_screen: RectTransform, node_pos: Pos2, scale: f32) -> bool
    {       
        let character_size = f32::max(8.0, scale * 8.0);
        let real_screen_pos = to_screen.transform_pos(self.pos2);
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
            self.pos2 = from_screen.transform_pos(new_screen_pos);
        }

        let mut made_step = false;
        if self.last_positions.is_empty() && self.on_node {
            self.last_positions.push(self.nearest_node);
        }
        //test if character was just released. doing this ourselfs allows to simulate release whenever we like
        //(e.g. just set dragging to true and we snap to position)
        if !dragging && self.dragging && self.on_node {
            self.pos2 = node_pos; //snap actual character postion to node where he was dragged
            if Some(&self.nearest_node) != self.last_positions.last() {
                //position changed and drag released -> new step
                self.last_positions.push(self.nearest_node);
                made_step = true;
            }
        }
        self.dragging = dragging;
        self.draw_large_at(draw_screen_pos, painter, ui, character_size);

        made_step
    }

    fn update_distances(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        queue.push_back(self.nearest_node);
        self.distances.clear();
        self.distances.resize(edges.nr_vertices(), isize::MAX);
        self.distances[self.nearest_node] = 0;
        edges.calc_distances_to(queue, &mut self.distances);
    }

    //go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, 
    //  change to that neighbor
    //(converges to globally nearest node only for "convex" graphs, 
    //  e.g. planar graphs, where each inside face is convex and the complement of the outside face is convex)
    fn update_2d(&mut self, tolerance: f32, map: &Embedding2D, queue: &mut VecDeque<usize>) {
        let safe_start = if map.len() > self.nearest_node { self.nearest_node } else { 0 };
        let (nearest_node, nearest_dist_sq) = map.find_nearest_node(self.pos2, safe_start);
        self.on_node = nearest_dist_sq <= tolerance * tolerance;

        let need_dist_update = self.distances.len() != map.len() || 
            (self.on_node && nearest_node != self.nearest_node);
        self.nearest_node = nearest_node;
        if need_dist_update {
            self.update_distances(map.edges(), queue);
        }
    }

    /// assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    fn update_3d(&mut self, tolerance: f32, map: &Embedding3D, to_2d: &geo::Project3To2, 
        vertex_visible: &[bool], queue: &mut VecDeque<usize>)
    {
        if !self.dragging {
            return;
        }        
        let potential = |v:usize, v_pos| {
            let dist_2d = (to_2d.project_pos(v_pos) - self.pos2).length_sq();
            let backface_penalty = 10.0 * (!vertex_visible[v]) as isize as f32;
            dist_2d + backface_penalty
        };
        //this potential might fail if the map has been swapped -> we assume here the map stayed the same
        assert!(map.nr_vertices() > self.nearest_node);
        let (nearest_node, nearest_dist_sq) = map.find_local_minimum(potential, self.nearest_node);
        self.on_node = nearest_dist_sq <= tolerance * tolerance;

        let need_dist_update = self.distances.len() != map.nr_vertices() || 
            (self.on_node && nearest_node != self.nearest_node);
        self.nearest_node = nearest_node;
        if need_dist_update {
            self.update_distances(map.edges(), queue);
            self.pos3 = map.positions()[self.nearest_node];
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RobberInfo { None, EscapableNodes, NearNodes, SmallRobberDist, CopDist }

impl RobberInfo {
    fn scale_small_dist_with_radius(dist: isize, radius: isize) -> isize {
        (dist * radius) / 100
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum DrawNumbers { None, Indices, RobberAdvantage, MinCopDist }


#[derive(Clone, Copy, PartialEq)]
pub enum RobberStrat { None, EscapeHull }

pub struct InfoState {
    //state kept for each node in map
    pub in_convex_cop_hull: Vec<InSet>, 
    pub min_cop_dist: Vec<isize>, //elementwise minimum of .distance of active cops in self.characters
    pub cop_advantage: Vec<isize>,
    pub visible: Vec<bool>,

    //indices of vertices just inside hull, ordered s.t. two following vertices are neighbors
    pub convex_hull_boundary: Vec<usize>,

    queue: VecDeque<usize>, //kept permanentely to reduce allocations when an update of some info per vertex is computed.
    
    pub characters: Vec<Character>,
    past_moves: Vec<usize>, //present is at end (same below)
    future_moves: Vec<(usize, usize)>, //fst is character index, snd is vertex index

    camera: Camera3D,

    pub robber_info: RobberInfo,
    //both are only used, when the respective RobberInfo is active
    pub marked_cop_dist: isize, //determines cop dist marked in RobberInfo::CopDist
    pub small_robber_dist: isize, //determines max dist marked in RobberInfo::SmallRobberDist

    pub robber_strat: RobberStrat,
    pub vertex_info: DrawNumbers,
    pub show_convex_hull: bool,
    pub show_steps: bool,
    pub debug_info: bool,
}

impl InfoState {

    fn last_moved(&self) -> Option<&Character> {
        self.past_moves.last().map(|&c| &self.characters[c])
    }

    fn next_moved(&self) -> Option<&Character> {
        self.future_moves.last().map(|(c, _)| &self.characters[*c])
    }

    fn reverse_move(&mut self, edges: &EdgeList) {
        if let Some(i) = self.past_moves.pop() {
            let ch = &mut self.characters[i];
            if let Some(v_curr) = ch.last_positions.pop() {
                self.future_moves.push((i, v_curr));
                if let Some(&v_last) = ch.last_positions.last() {
                    ch.nearest_node = v_last;
                    ch.update_distances(edges, &mut self.queue);
                }
            }
        }
    }

    fn redo_move(&mut self, edges: &EdgeList) {
        if let Some((i, v)) = self.future_moves.pop() {
            self.past_moves.push(i);
            let ch = &mut self.characters[i];
            ch.last_positions.push(v);
            ch.nearest_node = v;
            ch.update_distances(edges, &mut self.queue);
        }
    }

    fn new() -> Self {
        Self { 
            in_convex_cop_hull: Vec::new(),
            min_cop_dist: Vec::new(),
            cop_advantage: Vec::new(),
            visible: Vec::new(),

            convex_hull_boundary: Vec::new(),

            queue: VecDeque::new(),
            
            characters: vec![
                Character::new(&ROBBER, Pos2::ZERO),
                Character::new(&COPS[0], pos2(0.25, 0.0)),
                ],
            past_moves: Vec::new(),
            future_moves: Vec::new(),

            camera: Camera3D::new(),

            robber_info: RobberInfo::None,
            small_robber_dist: 10,
            marked_cop_dist: 0,

            robber_strat: RobberStrat::None,
            vertex_info: DrawNumbers::None,
            show_convex_hull: false,
            show_steps: false,
            debug_info: false,
        }
    }

    fn robber(&self) -> Option<&Character> {
        self.characters.first()
    }

    fn active_cops(&self) -> impl Iterator<Item = &Character> {
        let start = usize::min(1, self.characters.len());
        self.characters[start..].iter().filter(|&c| c.on_node)
    }

    pub fn forget_move_history(&mut self) {
        self.past_moves.clear();
        self.future_moves.clear();
        for ch in &mut self.characters {
            ch.last_positions.clear();
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, edges: &EdgeList) {
        if ui.button("🏠 Position").clicked() {
            self.camera.reset();
        }
        ui.collapsing("Knoteninfo", |ui|{                    
            //settings to draw extra information
            ui.radio_value(&mut self.robber_info, RobberInfo::None, 
                "keine Marker");

            ui.radio_value(&mut self.robber_info, RobberInfo::NearNodes, 
                "markiere für Räuber\nnähere Knoten");

            ui.radio_value(&mut self.robber_info, RobberInfo::EscapableNodes, 
                "markiere Punkte mit\ndirekter Fluchtoption");

            ui.radio_value(&mut self.robber_info, RobberInfo::SmallRobberDist, 
                "markiere Punkte nah\nan Räuber");
            if self.robber_info == RobberInfo::SmallRobberDist {
                add_drag_value(ui, &mut self.small_robber_dist, "% Radius: ", 1, 100);
            }

            ui.radio_value(&mut self.robber_info, RobberInfo::CopDist, 
                "markiere Punkte mit\nAbstand zu Cops");
            if self.robber_info == RobberInfo::CopDist {
                add_drag_value(ui, &mut self.marked_cop_dist, "Abstand: ", 0, 1000);
            }

            ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige \"Konvexe Hülle\"\n um Cops"));   
            ui.add(Checkbox::new(&mut self.debug_info, "bunte Kanten"));
        });
        ui.collapsing("Zahlen", |ui|{
            ui.radio_value(&mut self.vertex_info, DrawNumbers::None, 
                "Keine");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::Indices, 
                "Knotenindizes");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::RobberAdvantage, 
                "Räubervorteil");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::MinCopDist, 
                "minimaler Cop Abstand");
        });
        ui.collapsing("Strategie Räuber", |ui|{
            ui.radio_value(&mut self.robber_strat, RobberStrat::None, 
                "Keine");
            ui.radio_value(&mut self.robber_strat, RobberStrat::EscapeHull, 
                "Entkomme Hülle");
        });
        ui.horizontal(|ui| {
            let nr_characters = self.characters.len();
            let (minus_emoji, plus_emoji) = match nr_characters {
                0 => ("🚫", ROBBER.emoji),
                1 => (ROBBER.emoji, COPS[(nr_characters - 1) % 7].emoji),
                _ => (COPS[(nr_characters - 2) % 7].emoji, COPS[(nr_characters - 1) % 7].emoji),
            };
            let minus_text = format!("- Figur ({minus_emoji})");
            let plus_text = format!("+ Figur ({plus_emoji})");
            if ui.button(minus_text).clicked() {
                self.characters.pop();
                self.forget_move_history();
            }
            if ui.button(plus_text).clicked() {
                let data = &COPS[(nr_characters - 1) % 7];
                self.characters.push(Character::new(data, Pos2::ZERO));
            }
        });
        ui.horizontal(|ui| {
            ui.add(Checkbox::new(&mut self.show_steps, "zeige Schritte"));
            if ui.button("Reset").clicked() {
                self.forget_move_history();
            }
        });        
        ui.horizontal(|ui| {
            if ui.button(" ⟲ ").clicked() {
                self.reverse_move(edges);
            }
            if ui.button(" ⟳ ").clicked() {
                self.redo_move(edges);
            }
        });
        if let Some(ch) = self.last_moved() {
            ui.label(format!("letzter Schritt: {}", ch.data.job));
        }
        if let Some(ch) = self.next_moved() {
            ui.label(format!("nächster Schritt: {}", ch.data.job));
        }
    }

    fn update_min_cop_dist(&mut self) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        min_cop_dist.clear();
        {
            let mut active_cops = self.active_cops();
            if let Some(cop) = active_cops.next() {
                min_cop_dist.clone_from(&cop.distances);
            }
            for cop in active_cops {
                for (this, curr_min) in izip!(&cop.distances, &mut min_cop_dist) {
                    if this < curr_min {
                        *curr_min = *this;
                    }
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
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

    fn update_convex_hull_boundary(&mut self, edges: &EdgeList) {
        //idea: walk outside hull and look inside, but actually store the inside nodes
        //this approach will fail, if one cop stands at the border of the map, but this is something
        //2d no happening in the intresting real 3d case anyway.
        let mut boundary = std::mem::take(&mut self.convex_hull_boundary);
        boundary.clear();
        let fst_outside = 'search_start_vertex: loop {
            for cop in self.active_cops() {
                for n in edges.neighbors_of(cop.nearest_node) {
                    if !self.in_convex_cop_hull[n].yes() {
                        boundary.push(cop.nearest_node);
                        break 'search_start_vertex n;
                    }
                }
            }
            return;
        };
        let mut last_outside = usize::MAX;
        let mut curr_outside = fst_outside;
        loop {
            let mut change = false;
            for n in edges.neighbors_of(curr_outside) {
                //find vertices on hull boundary
                if self.in_convex_cop_hull[n].yes() {
                    let len = boundary.len();
                    let range_min = if len < 5 { 0 } else { len - 5 };
                    if !boundary[range_min..].contains(&n) {
                        boundary.push(n);
                    }
                }
                else { //find next vertex just outside hull
                    let still_searching = !change;
                    let is_new = n != last_outside; //can only be trusted if still_searching
                    let borders_hull = edges.neighbors_of(n).any(|nn| self.in_convex_cop_hull[nn].yes());
                    if still_searching && is_new && borders_hull {
                        last_outside = curr_outside;
                        curr_outside = n;
                        self.convex_hull_boundary.push(curr_outside);
                        change = true;
                    }
                }
            }
            if curr_outside == fst_outside || !change {
                break;
            }
        }
        self.convex_hull_boundary = boundary;
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
        self.update_convex_hull_boundary(edges);
    }

    fn maybe_update(&mut self, edges: &EdgeList, extreme_vertices: impl Iterator<Item = usize>) {
        let require_update = self.show_convex_hull 
            || self.robber_strat != RobberStrat::None
            || self.robber_info != RobberInfo::None 
            || self.vertex_info != DrawNumbers::None;
        if require_update {
            self.update_convex_cop_hull(edges, extreme_vertices);
            self.update_min_cop_dist();
            self.update_cop_advantage(edges);
        }
    }

    pub fn draw_convex_cop_hull<T, F>(&self, positions: &[T], painter: &Painter, mut to_screen: F, scale: f32) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        if !self.show_convex_hull {
            return;
        }
        let iter = izip!(&self.in_convex_cop_hull, positions, &self.visible);
        for (&in_hull, &pos, &vis) in iter {
            if vis && in_hull.yes()  {
                let draw_pos = to_screen(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 9.0, LIGHT_BLUE);
                painter.add(marker_circle);
            }
        }
        for &v in &self.convex_hull_boundary {
            if self.visible[v] {
                let draw_pos = to_screen(positions[v]);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 3.0, WHITE);
                painter.add(marker_circle);
            }
        }
    }
    

    fn draw_green_circles<T, F>(&self, positions: &[T], painter: &Painter, mut to_screen: F, scale: f32, radius: isize) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        let mut draw_circle_at = |pos: T|{
            let draw_pos = to_screen(pos);
            let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
            painter.add(marker_circle);
        };
        match (self.robber_info, self.robber()) {
            (RobberInfo::NearNodes, Some(r)) => 
                for (r_dist, c_dist, &pos, &vis) in 
                izip!(&r.distances, &self.min_cop_dist, positions, &self.visible) {
                    if vis && r_dist < c_dist {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::EscapableNodes, _) => 
                for (&adv, &pos, &vis) in izip!(&self.cop_advantage, positions, &self.visible) {
                    if vis && adv < -1 {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::SmallRobberDist, Some(r)) => {
                let bnd = RobberInfo::scale_small_dist_with_radius(self.small_robber_dist, radius);
                for (&dist, &pos, &vis) in izip!(&r.distances, positions, &self.visible) {
                    if vis && dist <= bnd {
                        draw_circle_at(pos);
                    }
                }
            },
            (RobberInfo::CopDist, _) => 
            for (&dist, &pos, &vis) in izip!(&self.min_cop_dist, positions, &self.visible) {
                if vis && dist == self.marked_cop_dist {
                    draw_circle_at(pos);
                }
            }
            _ => {},
        }
    }    

    fn draw_numbers<T, F>(&self, positions: &[T], ui: &Ui, painter: &Painter, mut to_screen: F, scale: f32) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        if self.vertex_info == DrawNumbers::None {
            return;
        }
        let font = FontId::proportional(scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode { WHITE } else { BLACK };
        for (i, &pos, &vis) in izip!(0.., positions, &self.visible) {
            if vis {
                let txt = match self.vertex_info {
                    DrawNumbers::Indices => { i.to_string() }
                    DrawNumbers::MinCopDist => { self.min_cop_dist[i].to_string() }
                    DrawNumbers::None => { panic!() }
                    DrawNumbers::RobberAdvantage => { (-1 -self.cop_advantage[i]).to_string() }
                };
                let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * scale);
                layout_job.halign = Align::Center;
                let galley = ui.fonts(|f| f.layout_job(layout_job));
                let screen_pos = to_screen(pos);
                let text = Shape::Text(TextShape::new(screen_pos, galley));
                painter.add(text);
            }
        }
    }

    fn draw_robber_strat<T, F>(&self, edges: &EdgeList, positions: &[T], painter: &Painter, 
        mut to_screen: F, scale: f32) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        if self.robber_strat == RobberStrat::None {
            return;
        }
        let potential = |v| match self.robber_strat {
            RobberStrat::None => panic!(),
            RobberStrat::EscapeHull => {
                //the robber will always try to increase the robber advantage, except for when that would lead to beeing
                //catched in the next move
                let min_cop_dist = self.min_cop_dist[v];
                if min_cop_dist < 2 {
                    10000.0
                }
                else {
                    //if multiple vertices have the same robber advantage, we use the min_cop_dist as tiebreaker.
                    //(because all vertices v tested are neighbors of the current robber position, their cop advantage
                    //  and min_cop_dist can vary between all of them by at most 2. thus 0.1 is sufficiently small
                    //  as factor to only let the min_cop_dist work as tiebreaker)
                    self.cop_advantage[v] as f32 - 0.1 * min_cop_dist as f32
                }
            }
        };
        if let Some(r) = self.robber() {
            let mut best = Vec::with_capacity(10);   
            best.push(r.nearest_node);
            let mut smallest_pot = potential(r.nearest_node);
            for neigh in edges.neighbors_of(r.nearest_node) {
                let neigh_pot = potential(neigh);
                if neigh_pot < smallest_pot {
                    best.clear();
                    best.push(neigh);
                    smallest_pot = neigh_pot;
                }
                else if neigh_pot == smallest_pot {
                    best.push(neigh);
                }
            }
            for v in best {
                let draw_pos = to_screen(positions[v]);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 4.0, RED);
                painter.add(marker_circle);
            }
        }
    }

    pub fn draw_character_tails<T, F>(&self, positions: &[T], painter: &Painter, mut to_screen: F, scale: f32)
    where T: Copy, F: FnMut(T) -> Pos2 
    {
        if !self.show_steps {
            return;
        }
        for ch in &self.characters {
            let f_len = ch.last_positions.len() as f32;
            for (i, (&v1, &v2)) in ch.last_positions.iter().tuple_windows().enumerate() { 
                if !self.visible[v1] {
                    continue;
                }      
                let draw_pos_1 = to_screen(positions[v1]);
                let size = scale * 2.5 * (i as f32 + 0.8 * f_len) / f_len;
                let marker_circle = Shape::circle_filled(draw_pos_1, size, ch.data.glow);
                painter.add(marker_circle);
                if !self.visible[v2] {
                    continue;
                } 
                let points = [draw_pos_1, to_screen(positions[v2])];
                let stroke = Stroke::new(size * 0.5, ch.data.glow);
                let line = Shape::LineSegment { points, stroke };
                painter.add(line);
            }
        }
    }

    fn process_general_input(&mut self, ui: &mut Ui, edges: &EdgeList) {
        ui.input(|info| {
            if info.modifiers.ctrl && info.key_pressed(Key::Z) {
                self.reverse_move(edges);
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Y) {
                self.redo_move(edges);
            }
        });
    }

    /// zoom changes happen with the cursor position as fixed point, thus 
    /// with zooming we also change the offset
    pub fn process_input_2d(&mut self, ui: &mut Ui, response: &Response, edges: &EdgeList) {
        self.process_general_input(ui, edges);
        self.camera.update_2d(ui, response.rect);
    }

    /// zoom changes don't change the offset at all
    pub fn process_input_3d(&mut self, ui: &mut Ui, response: &Response, edges: &EdgeList) {
        self.process_general_input(ui, edges);
        self.camera.update_3d(ui, response.rect);
    }
}  


const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

struct Camera3D {
    /// == 1.0 -> no change
    /// < 1.0  -> zoomed out
    /// > 1.0  -> zoomed in
    zoom: f32, 
    direction: [Vec3; 3],
    /// offset of center independent of zoom + direction (e.g. in draw plane)
    position: Pos2,

    to_screen: geo::ToScreen,
}

impl Camera3D {
    fn new() -> Self {
        let default_rect = Rect::from_center_size(Pos2::ZERO, Vec2::splat(1.0));
        Self { 
            zoom: 1.0, 
            direction: DEFAULT_AXES,
            position: Pos2::new(0.0, 0.0), 
            to_screen: geo::ToScreen::new(Project3To2::new(&DEFAULT_AXES), RectTransform::identity(default_rect))
        }
    }

    fn update_to_screen(&mut self, screen: Rect) {
        //it is important to use the current zoom to compute the influence of self.position,
        //not the zoom of the last frame. thus we never read the last self.to_streen.
        let scale_rect = {
            let ratio = screen.aspect_ratio();
            //shapes are centered around zero, with extreme vertices having length 1.0
            let mut from_size = Vec2::splat(2.05 / self.zoom);
            if ratio < 1.0 {
                from_size.y /= ratio;    
            }
            else {
                from_size.x *= ratio;   
            };
            let from = Rect::from_center_size(Pos2::ZERO, from_size);
            emath::RectTransform::from_to(from, screen)
        };
        let center = self.position.to_vec2() / scale_rect.scale();
        let from = scale_rect.from().translate(-center);
        let move_rect = emath::RectTransform::from_to(from, screen);
        
        //something something "project" projects to camera coordinates,
        //so we need to invert the axe's rotation or something
        let project = geo::Project3To2::inverse_of(&self.direction);
        self.to_screen = ToScreen::new(project, move_rect);
    }

    fn update_direction(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for axis in &mut self.direction {
            *axis = f(*axis);
        }
    }

    fn rotate_x(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_x(angle))
    }

    fn rotate_y(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_y(angle))
    }

    #[allow(dead_code)]
    fn rotate_z(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_z(angle))
    }

    /// only z-rotation, zoom is centered around mouse pointer
    fn update_2d(&mut self, ui: &mut Ui, screen: Rect) {        
        ui.input(|info| {
            if info.pointer.button_down(PointerButton::Secondary) {
                self.position -= info.pointer.delta();
            }
            self.position -= info.scroll_delta;
            if let Some(drag) = info.multi_touch() {
                self.position -= drag.translation_delta;
                self.rotate_z(drag.rotation_delta);
            }

            let zoom_delta = info.zoom_delta();
            self.zoom *= zoom_delta;
            if zoom_delta != 1.0 {
                if let Some(ptr_pos) = info.pointer.latest_pos() {
                    //keep fixed point of zoom at mouse pointer
                    let mid_to_ptr = ptr_pos - screen.center();
                    let mut zoom_center = self.position.to_vec2() - mid_to_ptr;
                    zoom_center *= zoom_delta;
                    self.position = zoom_center.to_pos2() + mid_to_ptr;
                }
            }
        });
        self.update_to_screen(screen);
    }

    /// no translation, zoom is centered around screen middle    
    fn update_3d(&mut self, ui: &mut Ui, screen: Rect) {        
        ui.input(|info| {
            let mut drag_dist = Vec2::ZERO;
            if info.pointer.button_down(PointerButton::Secondary) {
                drag_dist -= info.pointer.delta();
            }
            drag_dist -= info.scroll_delta;
            if let Some(drag) = info.multi_touch() {
                drag_dist -= drag.translation_delta;
                self.rotate_z(drag.rotation_delta);
            }
            let drag_rot = drag_dist / self.to_screen.move_rect.scale();
            self.rotate_x(drag_rot.y);
            self.rotate_y(drag_rot.x);
            geo::gram_schmidt_3d(&mut self.direction);

            let zoom_delta = info.zoom_delta();
            self.zoom *= zoom_delta;
        });
        self.update_to_screen(screen);
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn to_screen_3d(&self, p: Pos3) -> Pos2 {
        self.to_screen.apply(p)
    }

    pub fn to_screen_2d(&self, p: Pos2) -> Pos2 {
        //TODO: dont ignore rotation (works everywhere, except for character movement)
        self.to_screen.move_rect.transform_pos(p)
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