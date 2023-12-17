
use std::collections::VecDeque;

use egui::{*, emath::RectTransform};

use crate::{graph::{Embedding2D, Embedding3D, EdgeList}, geo::{self, Pos3}};

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

pub const MARKER: MarkerData = MarkerData {
    color: Color32::from_rgb(180, 180, 10),
    glow: Color32::from_rgb(240, 240, 50),
    emoji: "",
    job: "Marker",
};

//denotes eighter a cop or the robber as node on screen
//Pos2 is in graph coordinates, not in screen coordinates
pub struct Marker {
    pub data: &'static MarkerData,
    pub nearest_node: usize,
    pub pos2: Pos2, //used in both 3d and 3d map (as cursor can't drag in 3d...)
    pub pos3: Pos3, //only used in 3d map
    pub on_node: bool,
    pub dragging: bool, //currently beeing dragged by mouse cursor
}

impl Marker {
    pub fn new(data: &'static MarkerData, pos2: Pos2) -> Self {
        //dragging set to true snaps to node next update
        Marker { 
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

    pub fn draw_small_at(&self, draw_pos: Pos2, painter: &Painter, scale: f32) {
        let character_size = f32::max(4.0, scale * 4.0);
        let character_circle = Shape::circle_filled(draw_pos, character_size, self.data.glow);
        painter.add(character_circle);
    }

    pub fn drag_and_draw(&mut self, response: &Response, painter: &Painter, ui: &Ui, 
        to_screen: RectTransform, node_pos: Pos2, scale: f32, step_history: Option<&mut Vec<usize>>) -> bool
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
        if let Some(hist) = step_history {
            if hist.is_empty() && self.on_node {
                hist.push(self.nearest_node);
            }
            //test if character was just released. doing this ourselfs allows to simulate release whenever we like
            //(e.g. just set dragging to true and we snap to position)
            if !dragging && self.dragging && self.on_node {
                self.pos2 = node_pos; //snap actual character postion to node where he was dragged
                if Some(&self.nearest_node) != hist.last() {
                    //position changed and drag released -> new step
                    hist.push(self.nearest_node);
                    made_step = true;
                }
            }
        }
        self.dragging = dragging;
        self.draw_large_at(draw_screen_pos, painter, ui, character_size);

        made_step
    }

    /// go through all neighbors of current nearest node, if any neigbor is closer than current nearest node, 
    ///   change to that neighbor
    /// (converges to globally nearest node only for "convex" graphs, 
    ///   e.g. planar graphs, where each inside face is convex and the complement of the outside face is convex)
    /// returns if self snapped to a new node
    pub fn update_2d(&mut self, tolerance: f32, map: &Embedding2D) -> bool {
        let safe_start = if map.len() > self.nearest_node { self.nearest_node } else { 0 };
        let (nearest_node, nearest_dist_sq) = map.find_nearest_node(self.pos2, safe_start);
        self.on_node = nearest_dist_sq <= tolerance * tolerance;

        let change = self.on_node && nearest_node != self.nearest_node;
        self.nearest_node = nearest_node;
        change
    }

    /// assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    pub fn update_3d(&mut self, tolerance: f32, map: &Embedding3D, to_2d: &geo::Project3To2, 
        vertex_visible: &[bool]) -> bool
    {
        if !self.dragging {
            return false;
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

        let change = self.on_node && nearest_node != self.nearest_node;
        self.nearest_node = nearest_node;
        change
    }
}

pub struct Character {
    pub marker: Marker,
    pub last_positions: Vec<usize>,
    pub distances: Vec<isize>,
}

impl Character {
    pub fn new(data: &'static MarkerData, pos2: Pos2) -> Self {
        Self { 
            marker: Marker::new(data, pos2), 
            last_positions: Vec::new(), 
            distances: Vec::new() 
        }
    }    

    pub fn drag_and_draw(&mut self, response: &Response, painter: &Painter, ui: &Ui, 
        to_screen: RectTransform, node_pos: Pos2, scale: f32) -> bool
    {       
        self.marker.drag_and_draw(response, painter, ui, to_screen, node_pos, scale, 
            Some(&mut self.last_positions))
    }

    pub fn update_distances(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        queue.push_back(self.marker.nearest_node);
        self.distances.clear();
        self.distances.resize(edges.nr_vertices(), isize::MAX);
        self.distances[self.marker.nearest_node] = 0;
        edges.calc_distances_to(queue, &mut self.distances);
    }
    
    pub fn update_2d(&mut self, tolerance: f32, map: &Embedding2D, queue: &mut VecDeque<usize>) {
        let change = self.marker.update_2d(tolerance, map);
        if change || self.distances.len() != map.len() {
            self.update_distances(map.edges(), queue);
        }
    }

    /// assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    pub fn update_3d(&mut self, tolerance: f32, map: &Embedding3D, to_2d: &geo::Project3To2, 
        vertex_visible: &[bool], queue: &mut VecDeque<usize>)
    {

        let change = self.marker.update_3d(tolerance, map, to_2d, vertex_visible);
        if change || self.distances.len() != map.nr_vertices() {
            self.update_distances(map.edges(), queue);
            self.marker.pos3 = map.positions()[self.marker.nearest_node];
        }
    }
}
