
use std::collections::VecDeque;

use egui::*;
use itertools::izip;

use crate::{graph::EdgeList, geo::Pos3};

use super::{*, color::*};

#[derive(Clone, Copy)]
pub struct Style {
    pub color: Color32,
    pub glow: Color32,
    pub emoji: &'static str,
    pub job: &'static str,
}

const fn new_cop(emoji: &'static str) -> Style { 
    Style {
        color: Color32::from_rgb(10, 50, 170),
        glow: Color32::from_rgb(60, 120, 235),
        //alternatives: 👮🛂🛃🐈🔫🚔🐂🍩
        emoji,
        job: "Cop",
    } 
}

pub const NR_COP_STYLES: usize = 7;

/// fst is robber, rest are cops
pub const STYLES: [Style; 1 + NR_COP_STYLES] = [
    Style {
        color: Color32::from_rgb(170, 40, 40),
        glow: Color32::from_rgb(235, 120, 120),
        //alternatives: 👿🚴🏃🚶🐀🐢🕵💰
        emoji: "🏃",
        job: "Räuber",
    },
    new_cop("👮"), 
    new_cop("🍩"), 
    new_cop("🚔"), 
    new_cop("🐂"), 
    new_cop("🔫"), 
    new_cop("🛂"), 
    new_cop("🛃"),
];

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Character {
    style_index: usize,

    pub last_positions: Vec<usize>,
    pub distances: Vec<isize>,
    pub nearest_node: usize,

    /// position to where is is currently held while dragging, 
    /// only updated while dragging (coordinates are the middle ones of ToScreen)
    pos2: Pos2, 

    /// position of nearest_node. 
    /// this is only relevant, when a new graph is computed: it places the character at the node on the new graph 
    /// closest to where he was before.
    pub pos3: Pos3, 

    pub on_node: bool,
    dragging: bool, //currently beeing dragged by mouse cursor
    updated: bool, //nearest_node or distances changed this frame
}

impl Character {
    pub fn style(&self) -> &'static Style {
        &STYLES[self.style_index]
    }

    pub fn new(style_index: usize, pos2: Pos2) -> Self {
        //dragging set to true snaps to node next update
        Character { 
            style_index, 

            last_positions: Vec::new(),
            distances: Vec::new(),
            nearest_node: 0, 
            
            pos2, 
            pos3: Pos3::ZERO, 
            on_node: false, 
            dragging: true, //causes snap to node next update (as dragging will change to false)
            updated: true,
        }
    }

    fn draw_large_at(&self, draw_pos: Pos2, painter: &Painter, ui: &Ui, scale: f32) {
        //draw circles
        let character_circle = Shape::circle_filled(draw_pos, scale, self.style().color);
        painter.add(character_circle);
        if self.on_node {
            let stroke = Stroke::new(scale * 0.375, self.style().glow);
            let marker_circle = Shape::circle_stroke(draw_pos, scale, stroke);
            painter.add(marker_circle);
        }
        //draw emoji
        let font = FontId::proportional(scale * 2.0);
        let emoji_pos = draw_pos - scale * vec2(0.0, 1.35);
        let emoji_str = self.style().emoji.to_string();
        let mut layout_job = text::LayoutJob::simple(emoji_str, font, WHITE, 100.0);
        layout_job.halign = Align::Center;
        let galley = ui.fonts(|f| f.layout_job(layout_job));
        //let emoji = Shape::Text(epaint::TextShape::new(emoji_pos, galley));
        let emoji = Shape::Text(epaint::TextShape::new(emoji_pos, galley, WHITE));
        painter.add(emoji);
    }

    fn draw_small_at_node(&self, con: &DrawContext<'_>) {
        let character_size = f32::max(4.0, con.scale * 4.0);
        let draw_pos = con.vertex_draw_pos(self.nearest_node);
        let character_circle = Shape::circle_filled(draw_pos, character_size, self.style().glow);
        con.painter.add(character_circle);
    }

    /// returns true iff character was just released and has changed its node
    fn drag_and_draw(&mut self, ui: &Ui, con: &DrawContext<'_>) -> bool {       
        let to_plane = con.cam().to_screen().to_plane;
        let move_rect = con.cam().to_screen().move_rect;

        let node_pos = to_plane.project_pos(con.positions[self.nearest_node]);
        let draw_at_node = self.on_node && !self.dragging;
        let draw_screen_pos = if draw_at_node { 
            move_rect.transform_pos(node_pos)
        } else { 
            move_rect.transform_pos(self.pos2) 
        };

        let character_size = f32::max(6.0, con.scale * 8.0);
        let rect_len = 3.0 * character_size;
        let point_rect = Rect::from_center_size(draw_screen_pos, vec2(rect_len, rect_len));
        let character_id = con.response.id.with(self as *const Self);
        let point_response = ui.interact(point_rect, character_id, Sense::drag());
        let dragging = point_response.dragged_by(PointerButton::Primary);
        //dragging starts -> update actual position to match drawn position
        if dragging {
            let new_screen_pos = draw_screen_pos + point_response.drag_delta();
            self.pos2 = move_rect.inverse().transform_pos(new_screen_pos);
        }

        if self.last_positions.is_empty() && self.on_node {
            self.last_positions.push(self.nearest_node);
        }
        //test if character was just released. doing this ourselfs allows to simulate release whenever we like
        //(e.g. just set dragging to true and we snap to position)
        let mut just_released_on_new_node = false;
        if !dragging && self.dragging && self.on_node {
            self.pos2 = node_pos; //snap actual character postion to node where he was dragged
            if Some(&self.nearest_node) != self.last_positions.last() {
                //position changed and drag released -> new step
                self.last_positions.push(self.nearest_node);
                self.pos3 = con.positions[self.nearest_node];
                just_released_on_new_node = true;
            }
        }
        self.dragging = dragging;
        self.draw_large_at(draw_screen_pos, &con.painter, ui, character_size);

        just_released_on_new_node  
    }

    /// assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    fn update(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>)
    {
        if !self.dragging {
            return;
        }   
        let to_plane = con.cam().to_screen().to_plane;
        let mut best_dist_sq = f32::MAX;
        let mut best_vertex = usize::MAX;
        for (v, &vis, &pos) in izip!(0.., con.visible, con.positions) {
            if vis {
                let new_dist_sq = (to_plane.project_pos(pos) - self.pos2).length_sq();
                if new_dist_sq < best_dist_sq {
                    best_dist_sq = new_dist_sq;
                    best_vertex = v;
                }
            } 
        }
        self.on_node = best_dist_sq <= con.tolerance * con.tolerance;

        let change = self.on_node && best_vertex != self.nearest_node;
        self.nearest_node = best_vertex;
        if change || self.distances.len() != con.positions.len() {
            self.update_distances(con.edges, queue);
        }
    }

    pub fn update_distances(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        queue.push_back(self.nearest_node);
        self.distances.clear();
        self.distances.resize(edges.nr_vertices(), isize::MAX);
        self.distances[self.nearest_node] = 0;
        edges.calc_distances_to(queue, &mut self.distances);

        self.updated = true;
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct CharacterState {    
    characters: Vec<Character>,
    past_moves: Vec<usize>, //present is at end (same below)
    future_moves: Vec<(usize, usize)>, //fst is character index, snd is vertex index

    show_steps: bool,
}

impl CharacterState {
    pub fn show_steps(&self) -> bool {
        self.show_steps
    }

    pub fn new() -> Self {
        Self {            
            characters: vec![
                Character::new(0, Pos2::ZERO),
                Character::new(1, pos2(0.25, 0.0)),
                ],
            past_moves: Vec::new(),
            future_moves: Vec::new(),

            show_steps: false,
        }
    }

    pub fn last_moved(&self) -> Option<&Character> {
        self.past_moves.last().map(|&c| &self.characters[c])
    }

    pub fn next_moved(&self) -> Option<&Character> {
        self.future_moves.last().map(|(c, _)| &self.characters[*c])
    }

    pub fn reverse_move(&mut self, edges: &EdgeList, positions: &[Pos3], queue: &mut VecDeque<usize>) {
        if let Some(i) = self.past_moves.pop() {
            let ch = &mut self.characters[i];
            if let Some(v_curr) = ch.last_positions.pop() {
                self.future_moves.push((i, v_curr));
                if let Some(&v_last) = ch.last_positions.last() {
                    ch.nearest_node = v_last;
                    ch.update_distances(edges, queue);
                    ch.pos3 = positions[ch.nearest_node];            
                }
            }
        }
    }

    pub fn redo_move(&mut self, edges: &EdgeList, positions: &[Pos3], queue: &mut VecDeque<usize>) {
        if let Some((i, v)) = self.future_moves.pop() {
            self.past_moves.push(i);
            let ch = &mut self.characters[i];
            ch.last_positions.push(v);
            ch.nearest_node = v;
            ch.update_distances(edges, queue);
            ch.pos3 = positions[ch.nearest_node];     
        }
    }

    pub fn all(&self) -> &[Character] {
        &self.characters
    } 

    pub fn all_mut(&mut self) -> &mut [Character] {
        &mut self.characters
    } 

    pub fn robber(&self) -> Option<&Character> {
        self.characters.first()
    }

    pub fn cops(&self) -> &[Character] {
        let start = usize::min(1, self.characters.len());
        &self.characters[start..]
    }

    pub fn active_cops(&self) -> impl Iterator<Item = &Character> {
        self.cops().iter().filter(|&c| c.on_node)
    }

    pub fn robber_updated(&self) -> bool {
        self.robber().map_or(false, |r| r.updated)
    }

    pub fn active_cop_updated(&self) -> bool {
        self.active_cops().any(|c| c.updated)
    }

    pub fn frame_is_finished(&mut self) {
        for c in &mut self.characters {
            c.updated = false;
        }
    }

    pub fn forget_move_history(&mut self) {
        self.past_moves.clear();
        self.future_moves.clear();
        for ch in &mut self.characters {
            ch.last_positions.clear();
        }
    }

    /// returns a menu change (e.g. a new character was added, one removed...)
    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map, queue: &mut VecDeque<usize>) -> bool {
        let mut change = false;
        ui.horizontal(|ui| {
            let nr_characters = self.characters.len();
            let minus_emoji = self.characters.last().map_or("🚫", |c| c.style().emoji);
            let next_index = if nr_characters == 0 { 
                0 //indexes to ROBBER
            } else {
                let nr_cops = nr_characters - 1;
                let next_cop = nr_cops % NR_COP_STYLES;
                next_cop + 1 //+ 1 as robber is at beginning
            };
            let minus_text = format!("- Figur ({})", minus_emoji);
            let plus_text = format!("+ Figur ({})", STYLES[next_index].emoji);
            if ui.button(minus_text).clicked() {
                self.characters.pop();
                self.forget_move_history();
                change = true;
            }
            if ui.button(plus_text).clicked() {
                let mut new_ch = Character::new(next_index, Pos2::ZERO);
                let find_screen_facing = |v: usize| -map.positions()[v].to_vec3().normalized().dot(map.camera().screen_normal());
                let (v, _) = map.edges().find_local_minimum(find_screen_facing, 0);
                new_ch.nearest_node = v;
                self.characters.push(new_ch);
                change = true;
            }
        });
        ui.horizontal(|ui| {
            ui.add(Checkbox::new(&mut self.show_steps, "zeige Schritte"));
            if ui.button("Reset").clicked() {
                self.forget_move_history();
                change = true;
            }
        });        
        ui.horizontal(|ui| {
            if ui.button(" ⟲ ").on_hover_text("strg + z").clicked() {
                self.reverse_move(map.edges(), map.positions(), queue);
                change = true;
            }
            if ui.button(" ⟳ ").on_hover_text("strg + y").clicked() {
                self.redo_move(map.edges(), map.positions(), queue);
                change = true;
            }
        });
        if let Some(ch) = self.last_moved() {
            ui.label(format!("letzter Schritt: {} ({})", ch.style().job, ch.style().emoji));
        }
        if let Some(ch) = self.next_moved() {
            ui.label(format!("nächster Schritt: {} ({})", ch.style().job, ch.style().emoji));
        }
        change
    }

    pub fn update(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>) {
        for ch in &mut self.characters {
            ch.update(con, queue);
        }
    }

    pub fn draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        for (i, ch) in self.characters.iter_mut().enumerate() {
            if ch.on_node && con.visible[ch.nearest_node] || !ch.on_node {
                let finished_move = ch.drag_and_draw(ui, con);                
                if finished_move {
                    self.past_moves.push(i);
                    self.future_moves.clear();
                }
            }
            else {
                ch.draw_small_at_node(con);
            }
        }
    }
}
