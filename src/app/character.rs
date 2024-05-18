use std::collections::{HashMap, VecDeque};

use egui::*;
use itertools::{izip, Itertools};

use crate::{
    geo::{Pos3, Vec3},
    graph::{EdgeList, Embedding3D},
};

use super::{color::*, *};

#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
pub enum Id {
    Cop(usize), //which emoji is chosen
    Robber,
}

impl Id {
    pub fn str(&self) -> &'static str {
        match self {
            Id::Cop(_) => "Cop",
            Id::Robber => "Räuber",
        }
    }

    pub fn color(&self) -> Color32 {
        match self {
            Id::Cop(_) => Color32::from_rgb(10, 50, 170),
            Id::Robber => Color32::from_rgb(170, 40, 40),
        }
    }

    pub fn glow(&self) -> Color32 {
        match self {
            Id::Cop(_) => Color32::from_rgb(60, 120, 235),
            Id::Robber => Color32::from_rgb(235, 120, 120),
        }
    }

    //alternatives for cops: 👮🛂🛃🐈🔫🚔🐂🍩
    const COP_EMOJIES: &'static [&'static str] = &[
        "👮", "🍩", "🚔", "🐂", "🔫", "🛂", "🛃", "🚓", "🚁", "💂", "🏇",
    ];

    pub fn emoji(self) -> &'static str {
        match self {
            Id::Cop(i) => Self::COP_EMOJIES[i],
            //alternatives for robber: 👿🚴🏃🚶🐀🐢🕵💰
            Id::Robber => "🏃",
        }
    }

    pub fn same_job(self, other: Self) -> bool {
        matches!(self, Self::Robber) == matches!(other, Self::Robber)
    }
}

pub fn emojis_as_latex_commands() -> HashMap<&'static str, &'static str> {
    HashMap::from([
        ("🏃", "\\emoji{runner}"),
        ("👮", "\\emoji{police-officer}"),
        ("🍩", "\\emoji{doughnut}"),
        ("🚔", "\\emoji{oncoming-police-car}"),
        ("🐂", "\\emoji{ox}"),
        ("🔫", "\\emoji{water-pistol}"),
        ("🛂", "\\emoji{passport-control}"),
        ("🛃", "\\emoji{customs}"),
        ("🚓", "\\emoji{police-car}"),
        ("🚁", "\\emoji{helicopter}"),
        ("💂", "\\emoji{guard}"),
        ("🏇", "\\emoji{horse-racing}"),
    ])
}

#[derive(serde::Deserialize, serde::Serialize)]
enum Pos {
    /// [`Pos3`] is position of vertex where character was last dropped of.
    /// this is only relevant, when a new graph is computed: it places the character at the node on the new graph
    /// closest to where he was before.
    OnVertex(Pos3),
    /// position to where is is currently held while dragging,
    /// only active while dragging (coordinates are in the intermediate system of ToScreen)
    OnScreen(Pos2),
}

//denotes eighter a cop or the robber as node on screen
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Character {
    id: Id,
    pos: Pos,

    /// past values of [`Self::nearest_node`] where Character was not only dragged over
    past_vertices: Vec<usize>,

    /// distance of every vertex in graph to [`Self::nearest_node`]
    #[serde(skip)]
    distances: Vec<isize>,
    nearest_vertex: usize,

    on_node: bool, //currently close to node, can also be true while dragging.
    enabled: bool, //manual choice to not consider this instance in computations
    updated: bool, //nearest_node or distances changed this frame
}

impl Character {
    pub fn id(&self) -> Id {
        self.id
    }

    /// index of vertex where character is placed (if character is placed somewhere)
    pub fn vertex(&self) -> usize {
        self.nearest_vertex
    }

    /// number of steps to reach each vertex
    pub fn dists(&self) -> &[isize] {
        &self.distances
    }

    /// vertices where character used to stand on from oldest to newest
    pub fn past_vertices(&self) -> &[usize] {
        &self.past_vertices
    }

    pub fn is_active(&self) -> bool {
        self.enabled && self.on_node
    }

    pub fn new(id: Id, pos2: Pos2) -> Self {
        //dragging set to true snaps to node next update
        Character {
            id,
            pos: Pos::OnScreen(pos2),
            past_vertices: Vec::new(),
            distances: Vec::new(),
            nearest_vertex: 0,
            on_node: true,
            enabled: true,
            updated: true,
        }
    }

    fn draw_large_at(&self, draw_pos: Pos2, painter: &Painter, ui: &Ui, scale: f32) {
        //draw circles
        let character_circle = Shape::circle_filled(draw_pos, scale, self.id.color());
        painter.add(character_circle);
        if self.on_node {
            let stroke = Stroke::new(scale * 0.375, self.id.glow());
            let marker_circle = Shape::circle_stroke(draw_pos, scale, stroke);
            painter.add(marker_circle);
        }
        //draw emoji
        let font = FontId::proportional(scale * 2.0);
        let emoji_pos = draw_pos - scale * vec2(0.0, 1.35);
        let emoji_str = self.id.emoji().to_string();
        let mut layout_job = text::LayoutJob::simple(emoji_str, font, WHITE, 100.0);
        layout_job.halign = Align::Center;
        let galley = ui.fonts(|f| f.layout_job(layout_job));
        let emoji = Shape::Text(epaint::TextShape::new(emoji_pos, galley, WHITE));
        painter.add(emoji);
    }

    fn draw_small_at_node(&self, con: &DrawContext<'_>) {
        let character_size = f32::max(4.0, con.scale * 4.0);
        let draw_pos = con.vertex_draw_pos(self.nearest_vertex);
        let character_circle = Shape::circle_filled(draw_pos, character_size, self.id.glow());
        con.painter.add(character_circle);
    }

    /// returns true iff character was just released and has changed its node
    fn drag_and_draw(
        &mut self,
        ui: &Ui,
        con: &DrawContext<'_>,
        nr_others_at_same_pos: usize,
    ) -> bool {
        let to_plane = con.cam().to_screen().to_plane;
        let move_rect = con.cam().to_screen().move_rect;
        let character_size = f32::max(6.0, con.scale * 15.0);

        let node_pos = to_plane.project_pos(con.positions[self.nearest_vertex]);
        let draw_screen_pos = match self.pos {
            Pos::OnScreen(p2) => move_rect.transform_pos(p2),
            Pos::OnVertex(_) => {
                move_rect.transform_pos(node_pos)
                    + (nr_others_at_same_pos as f32) * vec2(0.0, character_size * 0.75)
            },
        };

        let rect_len = if nr_others_at_same_pos > 0 { 1.0 } else { 3.0 } * character_size;
        let point_rect = Rect::from_center_size(draw_screen_pos, vec2(rect_len, rect_len));
        let character_id = con.response.id.with(self as *const Self);
        let point_response = ui.interact(point_rect, character_id, Sense::drag());
        let now_dragging = point_response.dragged_by(PointerButton::Primary);
        let was_dragging = matches!(self.pos, Pos::OnScreen(_));
        //dragging starts -> update actual position to match drawn position
        if now_dragging {
            let new_screen_pos = draw_screen_pos + point_response.drag_delta();
            let new_pos = move_rect.inverse().transform_pos(new_screen_pos);
            self.pos = Pos::OnScreen(new_pos);
        }

        if self.past_vertices.is_empty() && self.on_node {
            self.past_vertices.push(self.nearest_vertex);
        }
        //test if character was just released. doing this ourselfs allows to simulate release whenever we like
        //(e.g. just set dragging to true and we snap to position)
        let mut just_released_on_new_node = false;
        if !now_dragging && was_dragging && self.on_node {
            self.pos = Pos::OnVertex(con.positions[self.nearest_vertex]);
            if Some(&self.nearest_vertex) != self.past_vertices.last() {
                //position changed and drag released -> new step
                self.past_vertices.push(self.nearest_vertex);
                just_released_on_new_node = true;
            }
        }
        self.draw_large_at(draw_screen_pos, &con.painter, ui, character_size);

        just_released_on_new_node
    }

    pub fn adjust_to_new_map(&mut self, map: &Embedding3D, queue: &mut VecDeque<usize>) {
        use crate::graph::planar3d::Z_OFFSET_2D;
        let dir_3d = match self.pos {
            Pos::OnScreen(p2) => Vec3::new(p2.x, p2.y, Z_OFFSET_2D),
            Pos::OnVertex(p3) => p3.to_vec3(),
        };
        let potential = |_, v_pos: Pos3| -dir_3d.dot(v_pos.to_vec3().normalized());
        let (best_new_vertex, _) = map.find_local_minimum(potential, 0);
        self.nearest_vertex = best_new_vertex;
        self.update_distances(map.edges(), queue);
    }

    /// assumes current nearest node to be "good", e.g. not on side of surface facing away from camera
    fn update(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>) {
        let Pos::OnScreen(pos2) = self.pos else {
            return;
        };
        let to_plane = con.cam().to_screen().to_plane;
        let mut best_dist_sq = f32::MAX;
        let mut best_vertex = usize::MAX;
        for (v, &vis, &pos) in izip!(0.., con.visible, con.positions) {
            if vis {
                let new_dist_sq = (to_plane.project_pos(pos) - pos2).length_sq();
                if new_dist_sq < best_dist_sq {
                    best_dist_sq = new_dist_sq;
                    best_vertex = v;
                }
            }
        }
        self.on_node = best_dist_sq <= con.tolerance * con.tolerance;

        let change = best_vertex != self.nearest_vertex;
        self.nearest_vertex = best_vertex;
        if change || self.distances.len() != con.positions.len() {
            self.update_distances(con.edges, queue);
        }
    }

    pub fn update_distances(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        queue.push_back(self.nearest_vertex);
        self.distances.clear();
        self.distances.resize(edges.nr_vertices(), isize::MAX);
        self.distances[self.nearest_vertex] = 0;
        edges.calc_distances_to(queue, &mut self.distances);

        self.updated = true;
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct CharacterState {
    characters: Vec<Character>,
    past_moves: Vec<usize>,            //present is at end (same below)
    future_moves: Vec<(usize, usize)>, //fst is character index, snd is vertex index

    show_past_steps: bool,
    show_allowed_next_steps: bool,
}

impl CharacterState {
    pub fn show_past_steps(&self) -> bool {
        self.show_past_steps
    }

    pub fn show_allowed_next_steps(&self) -> bool {
        self.show_allowed_next_steps
    }

    pub fn new() -> Self {
        Self {
            characters: vec![
                Character::new(Id::Robber, Pos2::ZERO),
                Character::new(Id::Cop(0), pos2(0.25, 0.0)),
            ],
            past_moves: Vec::new(),
            future_moves: Vec::new(),

            show_past_steps: false,
            show_allowed_next_steps: false,
        }
    }

    pub fn last_moved(&self) -> Option<&Character> {
        self.past_moves.last().map(|&c| &self.characters[c])
    }

    pub fn next_moved(&self) -> Option<&Character> {
        self.future_moves.last().map(|(c, _)| &self.characters[*c])
    }

    pub fn reverse_move(
        &mut self,
        edges: &EdgeList,
        positions: &[Pos3],
        queue: &mut VecDeque<usize>,
    ) {
        if let Some(i) = self.past_moves.pop() {
            let ch = &mut self.characters[i];
            if let Some(v_curr) = ch.past_vertices.pop() {
                self.future_moves.push((i, v_curr));
                if let Some(&v_last) = ch.past_vertices.last() {
                    ch.nearest_vertex = v_last;
                    ch.update_distances(edges, queue);
                    ch.pos = Pos::OnVertex(positions[ch.nearest_vertex]);
                }
            }
        }
    }

    pub fn redo_move(&mut self, edges: &EdgeList, positions: &[Pos3], queue: &mut VecDeque<usize>) {
        if let Some((i, v)) = self.future_moves.pop() {
            self.past_moves.push(i);
            let ch = &mut self.characters[i];
            ch.past_vertices.push(v);
            ch.nearest_vertex = v;
            ch.update_distances(edges, queue);
            ch.pos = Pos::OnVertex(positions[ch.nearest_vertex]);
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
        self.cops().iter().filter(|c| c.is_active())
    }

    pub fn active_cop_vertices(&self) -> smallvec::SmallVec<[usize; 8]> {
        let mut res = smallvec::SmallVec::new();
        res.extend(self.active_cops().map(|c| c.nearest_vertex));
        res
    }

    pub fn robber_updated(&self) -> bool {
        self.robber().map_or(false, |r| r.updated)
    }

    pub fn cop_updated(&self) -> bool {
        self.cops().iter().any(|c| c.updated)
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
            ch.past_vertices.clear();
        }
    }

    /// returns a menu change (e.g. a new character was added, one removed...)
    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map, queue: &mut VecDeque<usize>) -> bool {
        let mut change = false;
        ui.collapsing("Figuren", |ui| {
            ui.horizontal(|ui| {
                let minus_emoji = self.characters.last().map_or("🚫", |c| c.id.emoji());
                let next_id = 'choose_next: {
                    if self.characters.is_empty() {
                        break 'choose_next Id::Robber;
                    }
                    let mut cops_used = [0usize; Id::COP_EMOJIES.len()];
                    for c in &self.characters {
                        if let Id::Cop(i) = c.id {
                            cops_used[i] += 1;
                        }
                    }
                    Id::Cop(cops_used.iter().position_min().unwrap())
                };
                let minus_text = format!("- Figur ({})", minus_emoji);
                let plus_text = format!("+ Figur ({})", next_id.emoji());
                if ui.button(minus_text).clicked() {
                    self.characters.pop();
                    self.forget_move_history();
                    change = true;
                }
                if ui.button(plus_text).clicked() {
                    let pos = map.camera().in_front_of_cam();
                    let mut new_ch = Character::new(next_id, pos);
                    let find_screen_facing = |v: usize| {
                        -map.positions()[v].to_vec3().normalized().dot(map.camera().screen_normal())
                    };
                    let (v, _) = map.edges().find_local_minimum(find_screen_facing, 0);
                    new_ch.nearest_vertex = v;
                    self.characters.push(new_ch);
                    change = true;
                }
            });
            ui.collapsing("Aktive Cops", |ui| {
                if self.characters.len() > 1 {
                    let mut delete = None;
                    for (i, cop) in izip!(1.., &mut self.characters[1..]) {
                        ui.horizontal(|ui| {
                            ui.label(cop.id.emoji());
                            let was_enabled = cop.enabled;
                            ui.checkbox(&mut cop.enabled, "")
                                .on_hover_text("Berücksichte Cop bei Berechnungen");
                            change |= was_enabled != cop.enabled;
                            if ui.button("Löschen").clicked() {
                                delete = Some(i);
                            }
                        });
                    }
                    if let Some(i) = delete {
                        self.characters.remove(i);
                        self.forget_move_history();
                        change = true;
                    }
                }
            });
            ui.checkbox(&mut self.show_allowed_next_steps, "zeige Zugoptionen");
            ui.horizontal(|ui| {
                ui.add(Checkbox::new(&mut self.show_past_steps, "zeige Züge"));
                if ui.button("Reset Züge").clicked() {
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

            let print_style = |ch: Option<&Character>| {
                ch.map_or_else(
                    || " 🚫   ".to_string(),
                    |c| format!("{} ({})", c.id.str(), c.id.emoji()),
                )
            };

            ui.label(format!("letzter Zug: {}", print_style(self.last_moved())));
            ui.label(format!("nächster Zug: {}", print_style(self.next_moved())));
        });

        change
    }

    pub fn update(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>) {
        for ch in &mut self.characters {
            ch.update(con, queue);
        }
    }

    pub fn draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        let mut positions = smallvec::SmallVec::<[usize; 20]>::new();
        positions.extend(self.characters.iter().map(|c| c.nearest_vertex));

        for (i, ch) in self.characters.iter_mut().enumerate() {
            let nr_others_at_same_pos =
                positions[..i].iter().filter(|&&p| p == ch.nearest_vertex).count();
            //always allow to drag, except character sits on backside of graph
            if !ch.on_node || con.visible[ch.nearest_vertex] {
                let finished_move = ch.drag_and_draw(ui, con, nr_others_at_same_pos);
                if finished_move {
                    self.past_moves.push(i);
                    self.future_moves.clear();
                }
            } else {
                ch.draw_small_at_node(con);
            }
        }
    }
}
