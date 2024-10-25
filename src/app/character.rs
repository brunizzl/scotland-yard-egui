use std::{
    cmp::Ordering,
    collections::{HashMap, VecDeque},
};

use itertools::{izip, Itertools};

use egui::{epaint::PathStroke, pos2, vec2, Color32, Painter, Pos2, Rect, Sense, Stroke, Ui};

use crate::{
    app::bruteforce_state::GameType,
    geo::{Pos3, Vec3},
    graph::{
        bruteforce::{RawCops, MAX_COPS},
        EdgeList, Embedding3D,
    },
};

use super::{color::*, map, DrawContext};

#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
pub enum Id {
    Cop(usize), //which emoji is chosen
    Robber,
}

impl Id {
    pub fn job_str(&self) -> &'static str {
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

    //alternatives for cops: 👮🛂🛃🐈🔫🚔🐂🍩 "👁", "🚨", "🚴", "🚤",
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

    pub fn is_robber(self) -> bool {
        matches!(self, Self::Robber)
    }

    pub fn same_job(self, other: Self) -> bool {
        self.is_robber() == other.is_robber()
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

#[derive(Clone, Copy, serde::Deserialize, serde::Serialize)]
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

    /// past values of [`Self::nearest_vertex`] where Character was not only dragged over.
    /// if character rests on a vertex, this vertex is the last element.
    past_vertices: Vec<usize>,

    /// distance of every vertex in graph to [`Self::nearest_vertex`]
    #[serde(skip)]
    distances: Vec<isize>,
    nearest_vertex: usize,

    /// manual choice to not consider this instance in computations
    enabled: bool,
    /// currently close to node, can also be true while dragging.
    on_node: bool,
    /// [`Self::nearest_vertex`] or [`Self::distances`] changed this frame
    updated: bool,
}

impl Character {
    pub fn id(&self) -> Id {
        self.id
    }

    /// index of vertex where character is placed (if character is placed somewhere)
    pub fn vertex(&self) -> usize {
        self.nearest_vertex
    }

    /// number of steps to reach each vertex.
    /// only allowed to be accessed if character is on a vertex
    pub fn dists(&self) -> &[isize] {
        debug_assert!(self.on_node);
        &self.distances
    }

    /// vertices where character used to stand on from oldest to newest
    pub fn past_vertices(&self) -> &[usize] {
        &self.past_vertices
    }

    pub fn is_active(&self) -> bool {
        self.enabled && self.on_node
    }

    fn new(id: Id, pos2: Pos2) -> Self {
        //dragging set to true snaps to node next update
        Character {
            id,
            pos: Pos::OnScreen(pos2),
            past_vertices: Vec::new(),
            distances: Vec::new(),
            nearest_vertex: 0,
            on_node: false,
            enabled: true,
            updated: true,
        }
    }

    fn draw_large_at(&self, draw_pos: Pos2, painter: &Painter, character_size: f32) {
        //draw circles
        let character_circle =
            egui::Shape::circle_filled(draw_pos, character_size, self.id.color());
        painter.add(character_circle);
        if self.on_node {
            let stroke = Stroke::new(character_size * 0.375, self.id.glow());
            let marker_circle = egui::Shape::circle_stroke(draw_pos, character_size, stroke);
            painter.add(marker_circle);
        }
        //draw emoji
        let font = egui::FontId::proportional(character_size * 2.0);
        let emoji_pos = draw_pos - character_size * vec2(0.0, 1.35);
        let emoji_str = self.id.emoji().to_string();
        let mut layout_job = egui::text::LayoutJob::simple(emoji_str, font, WHITE, 100.0);
        layout_job.halign = egui::Align::Center;
        let galley = painter.ctx().fonts(|f| f.layout_job(layout_job));
        let emoji = egui::Shape::Text(egui::epaint::TextShape::new(emoji_pos, galley, WHITE));
        painter.add(emoji);
    }

    fn draw_small_at_node(&self, con: &DrawContext<'_>) {
        let character_size = f32::max(4.0, con.scale * 4.0);
        let draw_pos = con.vertex_draw_pos(self.nearest_vertex);
        let character_circle = egui::Shape::circle_filled(draw_pos, character_size, self.id.glow());
        con.painter.add(character_circle);
    }

    pub fn draw_size(con: &DrawContext<'_>) -> f32 {
        f32::max(6.0, con.scale * 15.0)
    }

    /// returns true iff character was just released and has changed its node
    fn drag_and_draw(
        &mut self,
        ui: &Ui,
        con: &DrawContext<'_>,
        nr_others_at_same_pos: usize,
        drag_enabled: bool,
    ) -> bool {
        let move_rect = con.cam().to_screen().move_rect;
        let character_size = Self::draw_size(con);

        let draw_screen_pos = match self.pos {
            Pos::OnScreen(p2) => move_rect.transform_pos(p2),
            Pos::OnVertex(_) => {
                let vertex_pos = con.cam().to_screen().apply(con.positions[self.nearest_vertex]);
                // don't draw multiple characters on the exact same position
                let offset = vec2(0.0, nr_others_at_same_pos as f32 * character_size * 0.75);
                vertex_pos + offset
            },
        };

        self.draw_large_at(draw_screen_pos, &con.painter, character_size);
        if !drag_enabled {
            return false;
        }

        let point_response = {
            let len = if nr_others_at_same_pos > 0 { 1.0 } else { 3.0 } * character_size;
            let full_rect = Rect::from_center_size(draw_screen_pos, vec2(len, len));
            // make sure that the character interaction area does not overlap the side menu
            let point_rect = con.screen().intersect(full_rect);

            let character_id = con.response.id.with(self as *const Self);
            ui.interact(point_rect, character_id, Sense::drag())
        };

        // change a cop's apperance on right click.
        // this has no actual meaning, so no need to tell anyone something changed.
        if let Id::Cop(i) = &mut self.id {
            point_response.context_menu(|ui| {
                ui.set_max_width(0.0);
                for (emoji_i, &emoji) in izip!(0.., Id::COP_EMOJIES) {
                    ui.radio_value(i, emoji_i, emoji);
                }
            });
        }

        let now_dragging = point_response.dragged_by(egui::PointerButton::Primary);
        let was_dragging = matches!(self.pos, Pos::OnScreen(_));
        //dragging starts -> update actual position to match drawn position
        if now_dragging {
            let new_screen_pos = draw_screen_pos + point_response.drag_delta();
            let new_pos = move_rect.inverse().transform_pos(new_screen_pos);
            self.pos = Pos::OnScreen(new_pos);
        }

        // gurantee that release on a new node guarantees _at least two_
        // vertices in self.past_vertices, such that a line depicting this movement can succesfully be drawn.
        if self.past_vertices.is_empty() && self.on_node {
            self.past_vertices.push(self.nearest_vertex);
        }

        //test if character was just released. doing this ourselfs allows to simulate release whenever we like
        //(e.g. just set dragging to true and we snap to position)
        if !now_dragging && was_dragging && self.on_node {
            self.pos = Pos::OnVertex(con.positions[self.nearest_vertex]);
            if Some(&self.nearest_vertex) != self.past_vertices.last() {
                //position changed and drag released -> new step
                self.past_vertices.push(self.nearest_vertex);
                return true;
            }
        }
        false
    }

    pub fn adjust_to_new_map(&mut self, map: &Embedding3D, queue: &mut VecDeque<usize>) {
        use crate::graph::planar3d::Z_OFFSET_2D;
        let dir_3d = match self.pos {
            Pos::OnScreen(p2) => Vec3::new(p2.x, p2.y, Z_OFFSET_2D),
            Pos::OnVertex(p3) => p3.to_vec3(),
        };
        let potential = |_, pos: Pos3| -dir_3d.dot(pos.to_vec3().normalized());
        let (best_new_vertex, _) = map.find_global_minimum(potential);
        self.nearest_vertex = best_new_vertex;
        self.update_distances(map.edges(), queue);
    }

    /// this function is expensive to call,
    /// if the character is beeing dragged or if the character is placed outside the map.
    /// the former one is ok, only up to a single character can be dragged at the same time,
    /// the latter one is bad and needs fixing (TODO!)
    fn update_nearest_vertex(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>) {
        let Pos::OnScreen(pos2) = self.pos else {
            return;
        };
        let to_plane = con.cam().to_screen().to_plane;
        let mut best_dist_sq = f32::MAX;
        let mut best_vertex = 0; //start at valid position in case no vertices are visible
        for (v, &vis, &pos) in izip!(0.., con.visible, con.positions) {
            if vis {
                let new_dist_sq = (to_plane.project_pos(pos) - pos2).length_sq();
                if new_dist_sq < best_dist_sq {
                    best_dist_sq = new_dist_sq;
                    best_vertex = v;
                }
            }
        }
        let now_on_node = best_dist_sq <= con.tolerance * con.tolerance;

        let change = best_vertex != self.nearest_vertex || now_on_node != self.on_node;
        self.nearest_vertex = best_vertex;
        self.on_node = now_on_node;
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

    fn clone_without_distances(&self) -> Self {
        Self {
            past_vertices: self.past_vertices.clone(),
            distances: Vec::new(),
            updated: true,
            ..*self
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct State {
    characters: Vec<Character>,

    /// oldest to newest. for an element fst is character index, snd is vertex index
    past_moves: Vec<(usize, usize)>,
    /// newest to oldest. for an element fst is character index, snd is vertex index
    future_moves: Vec<(usize, usize)>,

    pub show_past_steps: bool,
    pub show_allowed_next_steps: bool,
}

impl State {
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
        self.past_moves.last().map(|&(c, _)| &self.characters[c])
    }

    pub fn snd_last_moved(&self) -> Option<&Character> {
        let nr_past = self.past_moves.len();
        (nr_past >= 2).then(|| {
            let (c, _) = self.past_moves[nr_past - 2];
            &self.characters[c]
        })
    }

    pub fn next_moved(&self) -> Option<&Character> {
        self.future_moves.last().map(|&(c, _)| &self.characters[c])
    }

    /// moves are both `self.past_moves` and `self.future_moves`.
    /// result is rounded up to return something greater than zero if moves exist
    fn five_percent_of_moves(&self) -> usize {
        (self.past_moves.len() + self.future_moves.len() + 19) / 20
    }

    /// undoes move but will not update character distances.
    /// returns index of changed character.
    fn undo_move_without_update(&mut self, positions: &[Pos3]) -> Option<usize> {
        let (i, v) = self.past_moves.pop()?;
        self.future_moves.push((i, v));
        let ch = &mut self.characters[i];
        ch.on_node = true;
        ch.past_vertices.pop();
        if let Some(&v_last) = ch.past_vertices.last() {
            ch.nearest_vertex = v_last;
            ch.pos = Pos::OnVertex(positions[ch.nearest_vertex]);
        }
        Some(i)
    }

    pub fn undo_move(&mut self, edges: &EdgeList, positions: &[Pos3], queue: &mut VecDeque<usize>) {
        if let Some(i) = self.undo_move_without_update(positions) {
            let ch = &mut self.characters[i];
            ch.update_distances(edges, queue);
        }
    }

    fn undo_multiple_moves(
        &mut self,
        edges: &EdgeList,
        positions: &[Pos3],
        queue: &mut VecDeque<usize>,
        number: usize,
    ) {
        let mut need_update = vec![false; self.characters.len()];
        for i in std::iter::from_fn(|| self.undo_move_without_update(positions)).take(number) {
            need_update[i] = true;
        }
        for (&up, ch) in izip!(&need_update, &mut self.characters) {
            if up {
                ch.update_distances(edges, queue);
            }
        }
    }

    /// redoes move but will not update character distances.
    /// returns index of changed character.
    fn redo_move_without_update(&mut self, positions: &[Pos3]) -> Option<usize> {
        let (i, v) = self.future_moves.pop()?;
        self.past_moves.push((i, v));
        let ch = &mut self.characters[i];
        ch.past_vertices.push(v);
        ch.nearest_vertex = v;
        ch.pos = Pos::OnVertex(positions[ch.nearest_vertex]);
        ch.on_node = true;
        Some(i)
    }

    pub fn redo_move(&mut self, edges: &EdgeList, positions: &[Pos3], queue: &mut VecDeque<usize>) {
        if let Some(i) = self.redo_move_without_update(positions) {
            let ch = &mut self.characters[i];
            ch.update_distances(edges, queue);
        }
    }

    fn redo_multiple_moves(
        &mut self,
        edges: &EdgeList,
        positions: &[Pos3],
        queue: &mut VecDeque<usize>,
        number: usize,
    ) {
        let mut need_update = vec![false; self.characters.len()];
        for i in std::iter::from_fn(|| self.redo_move_without_update(positions)).take(number) {
            need_update[i] = true;
        }
        for (&up, ch) in izip!(&need_update, &mut self.characters) {
            if up {
                ch.update_distances(edges, queue);
            }
        }
    }

    /// we try to find a pattern in the last made moves and try to continue this pattern
    pub fn continue_move_pattern(
        &mut self,
        edges: &EdgeList,
        positions: &[Pos3],
        queue: &mut VecDeque<usize>,
    ) {
        // find longest period with at least one repetition
        let (character_index, last_destination) = 'find_period_start: {
            let hist = &self.past_moves[..];
            let mut period_len = usize::min(hist.len() / 2, 3);
            while period_len > 0 {
                let newest_hist = &hist[(hist.len() - 2 * period_len)..];
                let fst_period = &newest_hist[..period_len];
                let last_period = &newest_hist[period_len..];
                let mut same_character_pattern = true;
                for step in 0..period_len {
                    // only which character moved is compared, because
                    // this is vastly easier than in what direction the last moves where
                    // (we have the character indices stored directly, move directions not.)
                    if fst_period[step].0 != last_period[step].0 {
                        same_character_pattern = false;
                        break;
                    }
                }
                if same_character_pattern {
                    break 'find_period_start fst_period[0];
                }
                period_len -= 1;
            }
            // if all else fails but two moves where made, take the character of the second last move
            if let &[.., pair, _] = hist {
                break 'find_period_start pair;
            }
            // if only a single move was made, repeat that.
            if let &[.., pair] = hist {
                break 'find_period_start pair;
            }
            // no move was made -> nothing to repeat -> do nothing
            return;
        };

        let ch = &mut self.characters[character_index];
        let last_origin = 'find_last_origin: {
            let hist = ch.past_vertices();
            for (i, &v) in izip!(0..(hist.len()), hist).rev() {
                if v == last_destination && i > 0 {
                    break 'find_last_origin hist[i - 1];
                }
            }
            return;
        };
        let &[.., prev_v, curr_v] = ch.past_vertices() else {
            return;
        };

        let next_v = {
            // the next vertex should have the fewest possible commn neighbors with the last vertex
            // as proxy to increase the distance to the last vertex as fast as possible.
            // in case this graph-structure-only condition yields multiple candidates, we then
            // take the vertex which is closest to a step extrapolated from the last step.
            let prev_neighs = smallvec::SmallVec::<[_; 8]>::from_iter(edges.neighbors_of(prev_v));
            let mut fewest_common = usize::MAX;
            let mut vertex_with_least_common = usize::MAX;

            let last_step_dir = (positions[last_destination] - positions[last_origin]).normalized();
            let mut largest_dir_similarity = -1e10;
            let mut vertex_with_biggest_dot_prod = usize::MAX;
            for n in edges.neighbors_of(curr_v) {
                let nr_common = edges.neighbors_of(n).filter(|nn| prev_neighs.contains(nn)).count()
                    + 100 * prev_neighs.contains(&n) as usize;

                let dir_similarity =
                    (positions[n] - positions[curr_v]).normalized().dot(last_step_dir);
                debug_assert!((-1.1..=1.1).contains(&dir_similarity));

                if nr_common < fewest_common
                    || nr_common <= fewest_common && largest_dir_similarity < dir_similarity
                {
                    fewest_common = nr_common;
                    vertex_with_least_common = n;
                }

                if largest_dir_similarity < dir_similarity
                    || (largest_dir_similarity - dir_similarity).abs() < 0.05
                        && nr_common < fewest_common
                {
                    largest_dir_similarity = dir_similarity;
                    vertex_with_biggest_dot_prod = n;
                }
            }
            // we prefer the vertex in most similar direction, if the direction is similar enough.
            if largest_dir_similarity >= 0.5 {
                vertex_with_biggest_dot_prod
            } else if vertex_with_least_common != usize::MAX {
                vertex_with_least_common
            } else {
                return;
            }
        };

        ch.past_vertices.push(next_v);
        ch.nearest_vertex = next_v;
        ch.update_distances(edges, queue);
        ch.pos = Pos::OnVertex(positions[ch.nearest_vertex]);
        ch.on_node = true;

        self.past_moves.push((character_index, next_v));
        self.future_moves.clear();
    }

    pub fn forget_move_history(&mut self) {
        self.past_moves.clear();
        self.future_moves.clear();
        for ch in &mut self.characters {
            ch.past_vertices.clear();
        }
    }

    fn forget(&mut self, character_i: usize) {
        let retain_c = |(c, _): &mut (usize, _)| match (*c).cmp(&character_i) {
            Ordering::Less => true,
            Ordering::Equal => false,
            Ordering::Greater => {
                *c -= 1;
                true
            },
        };
        self.past_moves.retain_mut(retain_c);
        self.future_moves.retain_mut(retain_c);
    }

    pub fn all(&self) -> &[Character] {
        &self.characters
    }

    pub fn all_mut(&mut self) -> &mut [Character] {
        &mut self.characters
    }

    pub fn active_robber(&self) -> Option<&Character> {
        self.characters.first().filter(|r| r.is_active())
    }

    pub fn cops(&self) -> &[Character] {
        let start = usize::min(1, self.characters.len());
        &self.characters[start..]
    }

    pub fn active_cops(&self) -> impl Iterator<Item = &Character> {
        self.cops().iter().filter(|c| c.is_active())
    }

    pub fn active_cop_vertices(&self) -> smallvec::SmallVec<[usize; 8]> {
        smallvec::SmallVec::from_iter(self.active_cops().map(|c| c.nearest_vertex))
    }

    pub fn robber_updated(&self) -> bool {
        self.active_robber().map_or(false, |r| r.updated)
    }

    pub fn cop_updated(&self) -> bool {
        self.cops().iter().any(|c| c.updated)
    }

    pub fn start_new_frame(&mut self, con: &DrawContext<'_>, queue: &mut VecDeque<usize>) {
        for c in &mut self.characters {
            c.updated = false;
            c.update_nearest_vertex(con, queue);
        }
    }

    fn next_id(&self) -> Id {
        if self.characters.is_empty() {
            return Id::Robber;
        }
        let mut cops_used = [0usize; Id::COP_EMOJIES.len()];
        for c in &self.characters {
            if let Id::Cop(i) = c.id {
                cops_used[i] += 1;
            }
        }
        Id::Cop(cops_used.iter().position_min().unwrap())
    }

    pub fn create_character_at(&mut self, screen_pos: Pos2, map: &map::Map) {
        let pos = map.camera().screen_to_intermediary(screen_pos);
        self.characters.push(Character::new(self.next_id(), pos));
    }

    /// if the robber is at the given position, he will remain, but forget his move history.
    pub fn remove_cop_at_vertex(&mut self, v: usize) -> bool {
        if let Some(i) = self.characters.iter().position(|c| c.nearest_vertex == v) {
            if i == 0 {
                debug_assert!(self.characters[0].id.is_robber());
                self.characters[0].past_vertices.clear();
                self.past_moves.retain(|m| m.0 != 0);
                self.future_moves.retain(|m| m.0 != 0);
            } else {
                self.characters.remove(i);
                self.forget(i);
            }
            return true;
        }
        false
    }

    /// returns a menu change (e.g. a new character was added, one removed...)
    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map, queue: &mut VecDeque<usize>) -> bool {
        let mut change = false;
        ui.collapsing("Figuren / Züge", |ui| {
            ui.horizontal(|ui| {
                let minus_emoji = self.characters.last().map_or("🚫", |c| c.id.emoji());
                let minus_text = format!("- Figur ({})", minus_emoji);
                let plus_text = format!("+ Figur ({})", self.next_id().emoji());
                if ui.button(minus_text).on_hover_text("F2").clicked() {
                    self.characters.pop();
                    self.forget(self.characters.len());
                    change = true;
                }
                if ui.button(plus_text).on_hover_text("F2").clicked() {
                    let pos = map.camera().in_front_of_cam();
                    self.create_character_at(pos, map);
                    change = true;
                }
            });
            let nr_cops = self.cops().len();
            let draw_cops = |ui: &mut Ui| {
                if self.characters.len() > 1 {
                    let mut delete = None;
                    for (i, cop) in izip!(1.., &mut self.characters[1..]) {
                        ui.horizontal(|ui| {
                            ui.label(cop.id.emoji());
                            let was_enabled = cop.enabled;
                            ui.checkbox(&mut cop.enabled, "")
                                .on_hover_text("berücksichte Cop bei Berechnungen");
                            change |= was_enabled != cop.enabled;
                            if ui.button(" 🗑 ").on_hover_text("löschen").clicked() {
                                delete = Some(i);
                            }
                        });
                    }
                    if let Some(i) = delete {
                        self.characters.remove(i);
                        self.forget(i);
                        change = true;
                    }
                }
            };
            ui.add_space(5.0);
            ui.menu_button(format!("alle Cops ({})", nr_cops), |ui| {
                egui::ScrollArea::vertical().show(ui, draw_cops);
            });
            ui.add_space(8.0);
            ui.checkbox(&mut self.show_allowed_next_steps, "zeige Zugoptionen")
                .on_hover_text("F3");
            ui.checkbox(&mut self.show_past_steps, "zeige Züge").on_hover_text("F4");
            ui.horizontal(|ui| {
                //⏭⏮⏩⏪
                if ui.button(" ⏮ ").on_hover_text("zurück zum Anfang der Zeit").clicked() {
                    self.undo_multiple_moves(map.edges(), map.positions(), queue, usize::MAX);
                    change = true;
                }
                if ui.button(" ⏪ ").on_hover_text("5% der Zeit zurückspulen").clicked() {
                    let nr = self.five_percent_of_moves();
                    self.undo_multiple_moves(map.edges(), map.positions(), queue, nr);
                    change = true;
                }
                if ui
                    .button(" ⟲ ")
                    .on_hover_text("einen Zug in Vergangenheit (strg + z)")
                    .clicked()
                {
                    self.undo_move(map.edges(), map.positions(), queue);
                    change = true;
                }
                if ui.button(" ⟳ ").on_hover_text("einen Zug in Zukunft (strg + y)").clicked() {
                    self.redo_move(map.edges(), map.positions(), queue);
                    change = true;
                }
                if ui.button(" ⏩ ").on_hover_text("5% der Zeit vorspulen").clicked() {
                    let nr = self.five_percent_of_moves();
                    self.redo_multiple_moves(map.edges(), map.positions(), queue, nr);
                    change = true;
                }
                if ui.button(" ⏭ ").on_hover_text("vorspulen ans Ende der Zeit").clicked() {
                    self.redo_multiple_moves(map.edges(), map.positions(), queue, usize::MAX);
                    change = true;
                }
            });

            ui.add_space(8.0);
            let mut print_move = |move_name: &str, ch: Option<&Character>| {
                let ch_name = ch.map_or_else(
                    || " 🚫   ".to_string(),
                    |c| format!("{} ({})", c.id.job_str(), c.id.emoji()),
                );
                ui.label(format!("{move_name}: {ch_name}"));
            };
            print_move("vorletzter Zug", self.snd_last_moved());
            print_move("letzter Zug", self.last_moved());
            print_move("nächster Zug", self.next_moved());

            ui.add_space(8.0);
            if ui.button(" 🗑 ").on_hover_text("Spiel vergessen").clicked() {
                self.forget_move_history();
                change = true;
            }
        });

        change
    }

    pub fn draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>, drag_enabled: bool) {
        type PosVec = smallvec::SmallVec<[usize; 20]>;
        let positions = PosVec::from_iter(self.characters.iter().map(|c| c.nearest_vertex));

        for (i, ch) in self.characters.iter_mut().enumerate() {
            let nr_others_at_same_pos =
                positions[..i].iter().filter(|&&p| p == ch.nearest_vertex).count();
            //always allow to drag, except character sits on backside of graph
            if !ch.on_node || con.visible[ch.nearest_vertex] {
                let finished_move = ch.drag_and_draw(ui, con, nr_others_at_same_pos, drag_enabled);
                if finished_move {
                    self.past_moves.push((i, ch.nearest_vertex));
                    self.future_moves.clear();
                }
            } else {
                ch.draw_small_at_node(con);
            }
        }
    }

    pub fn draw_tails(&self, con: &DrawContext<'_>) {
        if !self.show_past_steps {
            return;
        }
        for ch in self.all() {
            let glow = ch.id().glow();
            let trans = glow.gamma_multiply(0.3);

            let mut size = con.scale * 5.0;
            for (&v1, &v2) in ch.past_vertices().iter().rev().tuple_windows() {
                if !con.visible[v1] || !con.visible[v2] {
                    continue;
                }

                let points = [con.vertex_draw_pos(v1), con.vertex_draw_pos(v2)];
                let stroke = {
                    let space_dist = (con.positions[v2] - con.positions[v1]).length();
                    let max = con.map.data().max_shown_edge_length();
                    let color = if space_dist < max { glow } else { trans };
                    PathStroke::new(size * 0.75, color)
                };
                let line = egui::Shape::LineSegment { points, stroke };
                con.painter.add(line);

                if size > con.scale * 3.0 {
                    size *= 0.95;
                }
            }
        }
    }

    pub fn draw_allowed_next_steps(&self, con: &DrawContext<'_>) {
        if !self.show_allowed_next_steps {
            return;
        }
        let Some(last_moved_character) = self.last_moved() else {
            return;
        };
        for ch in self.all() {
            let name = ch.id();
            if !ch.is_active() || name.same_job(last_moved_character.id()) {
                continue;
            }
            let Some(&v) = ch.past_vertices().last() else {
                continue;
            };
            for n in con.edges.neighbors_of(v) {
                if con.visible[n] {
                    let draw_pos = con.vertex_draw_pos(n);
                    let stroke = Stroke::new(con.scale * 2.0, name.glow());
                    let marker_circle =
                        egui::Shape::circle_stroke(draw_pos, con.scale * 6.5, stroke);
                    con.painter.add(marker_circle);
                }
            }
        }
    }

    pub fn clone_without_distances(&self) -> Self {
        Self {
            characters: self
                .characters
                .iter()
                .map(Character::clone_without_distances)
                .collect_vec(),
            past_moves: self.past_moves.clone(),
            future_moves: self.future_moves.clone(),

            ..*self
        }
    }

    /// this function is only really of use, when the [`GameType`] returned as `.1`
    /// is computed as bruteforce thingy.
    /// It is thus ok to return [`RawCops`], because bruteforce also needs to respect `MAX_COPS`.
    pub fn police_state(&self, con: &DrawContext<'_>) -> (RawCops, GameType) {
        let active_cops = self.active_cop_vertices();
        let raw_nr_cops = usize::min(active_cops.len(), MAX_COPS);
        let raw_cops = RawCops::new(&active_cops[..raw_nr_cops]);
        let game_type = GameType {
            nr_cops: active_cops.len(),
            resolution: con.map.resolution(),
            shape: con.map.shape(),
        };
        (raw_cops, game_type)
    }
}
