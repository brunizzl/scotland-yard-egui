
use std::collections::VecDeque;

use egui::*;

use crate::{graph::EdgeList, geo::Pos3};

use super::*;

pub struct MarkerData {
    pub color: Color32,
    pub glow: Color32,
    pub emoji: &'static str,
    pub job: &'static str,
}

const fn new_cop(emoji: &'static str) -> MarkerData { 
    MarkerData {
        color: Color32::from_rgb(10, 50, 170),
        glow: Color32::from_rgb(60, 120, 235),
        //alternatives: 👮🛂🛃🐈🔫🚔🐂🍩
        emoji,
        job: "Cop",
    } 
}
pub const COPS: [MarkerData; 7] = [
    new_cop("👮"), 
    new_cop("🍩"), 
    new_cop("🐂"), 
    new_cop("🔫"), 
    new_cop("🚔"), 
    new_cop("🛂"), 
    new_cop("🛃")
];

pub const ROBBER: MarkerData = MarkerData {
    color: Color32::from_rgb(170, 40, 40),
    glow: Color32::from_rgb(235, 120, 120),
    //alternatives: 👿🚴🏃🚶🐀🐢🕵💰
    emoji: "🏃",
    job: "Räuber",
};

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Character {
    pub last_positions: Vec<usize>,
    pub distances: Vec<isize>,

    pub data: &'static MarkerData,
    pub nearest_node: usize,
    pub pos2: Pos2, //used in both 3d and 3d map (as cursor can't drag in 3d...)
    pub pos3: Pos3, //only used in 3d map
    pub on_node: bool,
    pub dragging: bool, //currently beeing dragged by mouse cursor
}

impl Character {
    pub fn new(data: &'static MarkerData, pos2: Pos2) -> Self {
        //dragging set to true snaps to node next update
        Character { 
            last_positions: Vec::new(),
            distances: Vec::new(),

            data, 
            nearest_node: 0, 
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

    pub fn draw_small(&self, con: &DrawContext<'_>) {
        let character_size = f32::max(4.0, con.scale * 4.0);
        let draw_pos = con.vertex_draw_pos(self.nearest_node);
        let character_circle = Shape::circle_filled(draw_pos, character_size, self.data.glow);
        con.painter.add(character_circle);
    }

    pub fn drag_and_draw(&mut self, ui: &Ui, con: &DrawContext<'_>) -> bool
    {       
        let to_plane = con.cam.to_screen().to_plane;
        let move_rect = con.cam.to_screen().move_rect;

        let node_pos = to_plane.project_pos(con.positions[self.nearest_node]);
        let draw_at_node = self.on_node && !self.dragging;
        let draw_screen_pos = if draw_at_node { 
            move_rect.transform_pos(node_pos)
        } else { 
            move_rect.transform_pos(self.pos2) 
        };

        let character_size = f32::max(8.0, con.scale * 8.0);
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
        self.draw_large_at(draw_screen_pos, &con.painter, ui, character_size);

        made_step
    }

    /// assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    pub fn update(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>)
    {
        if !self.dragging {
            return;
        }   
        let to_plane = con.cam.to_screen().to_plane;
        let potential = |v:usize| {
            let dist_2d = (to_plane.project_pos(con.positions[v]) - self.pos2).length_sq();
            let backface_penalty = 10.0 * (!con.visible[v]) as isize as f32;
            dist_2d + backface_penalty
        };
        //this potential might fail if the map has been swapped -> we assume here the map stayed the same
        assert!(con.positions.len() > self.nearest_node);
        let (nearest_node, nearest_dist_sq) = con.edges.find_local_minimum(potential, self.nearest_node);
        self.on_node = nearest_dist_sq <= con.tolerance * con.tolerance;

        let change = self.on_node && nearest_node != self.nearest_node;
        self.nearest_node = nearest_node;
        if change || self.distances.len() != con.positions.len() {
            self.update_distances(con.edges, queue);
            self.pos3 = con.positions[self.nearest_node];
        }
    }

    pub fn update_distances(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        queue.push_back(self.nearest_node);
        self.distances.clear();
        self.distances.resize(edges.nr_vertices(), isize::MAX);
        self.distances[self.nearest_node] = 0;
        edges.calc_distances_to(queue, &mut self.distances);
    }
}

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
                Character::new(&ROBBER, Pos2::ZERO),
                Character::new(&COPS[0], pos2(0.25, 0.0)),
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

    pub fn reverse_move(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        if let Some(i) = self.past_moves.pop() {
            let ch = &mut self.characters[i];
            if let Some(v_curr) = ch.last_positions.pop() {
                self.future_moves.push((i, v_curr));
                if let Some(&v_last) = ch.last_positions.last() {
                    ch.nearest_node = v_last;
                    ch.update_distances(edges, queue);
                }
            }
        }
    }

    pub fn redo_move(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        if let Some((i, v)) = self.future_moves.pop() {
            self.past_moves.push(i);
            let ch = &mut self.characters[i];
            ch.last_positions.push(v);
            ch.nearest_node = v;
            ch.update_distances(edges, queue);
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

    pub fn forget_move_history(&mut self) {
        self.past_moves.clear();
        self.future_moves.clear();
        for ch in &mut self.characters {
            ch.last_positions.clear();
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        ui.horizontal(|ui| {
            let nr_characters = self.characters.len();
            let minus_emoji = self.characters.last().map_or("🚫", |c| c.data.emoji);
            let next_data = if nr_characters == 0 { 
                &ROBBER 
            } else {
                let nr_cops = nr_characters - 1;
                let next_cop = nr_cops % COPS.len();
                &COPS[next_cop]
            };
            let minus_text = format!("- Figur ({})", minus_emoji);
            let plus_text = format!("+ Figur ({})", next_data.emoji);
            if ui.button(minus_text).clicked() {
                self.characters.pop();
                self.forget_move_history();
            }
            if ui.button(plus_text).clicked() {
                self.characters.push(Character::new(next_data, Pos2::ZERO));
            }
        });
        ui.horizontal(|ui| {
            ui.add(Checkbox::new(&mut self.show_steps, "zeige Schritte"));
            if ui.button("Reset").clicked() {
                self.forget_move_history();
            }
        });        
        ui.horizontal(|ui| {
            if ui.button(" ⟲ ").on_hover_text("strg + z").clicked() {
                self.reverse_move(edges, queue);
            }
            if ui.button(" ⟳ ").on_hover_text("strg + y").clicked() {
                self.redo_move(edges, queue);
            }
        });
        if let Some(ch) = self.last_moved() {
            ui.label(format!("letzter Schritt: {} ({})", ch.data.job, ch.data.emoji));
        }
        if let Some(ch) = self.next_moved() {
            ui.label(format!("nächster Schritt: {} ({})", ch.data.job, ch.data.emoji));
        }
    }

    pub fn update_and_draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>, queue: &mut VecDeque<usize>) {
        for (i, ch) in self.characters.iter_mut().enumerate() {
            ch.update(con, queue);
            if ch.on_node && con.visible[ch.nearest_node] || !ch.on_node {
                let moved = ch.drag_and_draw(ui, con);
                if moved {
                    self.past_moves.push(i);
                    self.future_moves.clear();
                }
            }
            else {
                ch.draw_small(con);
            }
        }
    }
}
