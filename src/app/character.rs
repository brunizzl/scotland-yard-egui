use std::{
    cmp::Ordering,
    collections::{HashMap, VecDeque},
};

use web_time::{Duration, Instant};

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

use super::{add_drag_value, map, style::Style, DrawContext};

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
    /// only active while dragging or while sitting outside of graph
    /// (coordinates are in the intermediate system of ToScreen).
    OnScreen(Pos2),
}

#[derive(serde::Deserialize, serde::Serialize, Clone, Copy, PartialEq)]
pub struct CharactersStyle(pub Style<3>);

impl CharactersStyle {
    pub const DEFAULT_COLORS: [Color32; 3] = [
        Color32::from_rgb(10, 50, 170),
        Color32::WHITE,
        Color32::from_rgb(170, 40, 40),
    ];

    pub const DEFAULT: Self = Self(Style {
        colors: CharactersStyle::DEFAULT_COLORS,
        size: 1.0,
    });

    pub fn symbol_color(&self) -> Color32 {
        self.0.colors[1]
    }

    pub fn base_color(&self, id: Id) -> Color32 {
        match id {
            Id::Cop(_) => self.0.colors[0],
            Id::Robber => self.0.colors[2],
        }
    }

    pub fn glow_color(&self, id: Id) -> Color32 {
        let base = self.base_color(id);
        // continuous function, choosen to produce the same results as
        // the hard coded values before, given the inputs are the default colors.
        let brighter = |channel: u8| -> u8 {
            match channel {
                0..=40 => 2 * channel + 40,
                41..=55 => 120,
                56..=175 => channel + 65,
                176..=255 => 240,
            }
        };
        let [r, g, b, _] = base.to_array();
        Color32::from_rgb(brighter(r), brighter(g), brighter(b))
    }

    pub fn size(&self) -> f32 {
        self.0.size
    }
}

impl Default for CharactersStyle {
    fn default() -> Self {
        Self::DEFAULT
    }
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
}

pub enum Change {
    Released,
    NewPos,
    ToggleActive,
    Delete,
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
        }
    }

    fn draw_large_at(
        &self,
        draw_pos: Pos2,
        painter: &Painter,
        character_size: f32,
        style: &CharactersStyle,
    ) {
        //draw circles
        let character_circle =
            egui::Shape::circle_filled(draw_pos, character_size, style.base_color(self.id));
        painter.add(character_circle);
        if self.on_node {
            let stroke = Stroke::new(character_size * 0.375, style.glow_color(self.id));
            let marker_circle = egui::Shape::circle_stroke(draw_pos, character_size, stroke);
            painter.add(marker_circle);
        }
        //draw emoji
        let font = egui::FontId::proportional(character_size * 2.0);
        let emoji_pos = draw_pos - character_size * vec2(0.0, 1.35);
        let emoji_str = self.id.emoji().to_string();
        let mut layout_job =
            egui::text::LayoutJob::simple(emoji_str, font, style.symbol_color(), 100.0);
        layout_job.halign = egui::Align::Center;
        let galley = painter.ctx().fonts(|f| f.layout_job(layout_job));
        let emoji = egui::Shape::Text(egui::epaint::TextShape::new(
            emoji_pos,
            galley,
            style.symbol_color(),
        ));
        painter.add(emoji);
    }

    fn draw_small_at_node(&self, con: &DrawContext<'_>, style: &CharactersStyle) {
        let character_size = f32::max(4.0, con.scale * 4.0 * style.size());
        let draw_pos = con.vertex_draw_pos(self.nearest_vertex);
        let character_circle =
            egui::Shape::circle_filled(draw_pos, character_size, style.glow_color(self.id));
        con.painter.add(character_circle);
    }

    fn drag_and_draw(
        &mut self,
        ui: &Ui,
        con: &DrawContext<'_>,
        nr_others_at_same_pos: usize,
        drag_enabled: bool,
        style: &CharactersStyle,
    ) -> Option<Change> {
        let move_rect = con.cam().to_screen().move_rect;
        let character_size = f32::max(6.0, con.scale * 15.0) * style.size();

        let draw_screen_pos = match self.pos {
            Pos::OnScreen(p2) => move_rect.transform_pos(p2),
            Pos::OnVertex(_) => {
                let vertex_pos = con.cam().to_screen().apply(con.positions[self.nearest_vertex]);
                // don't draw multiple characters on the exact same position
                let offset = vec2(0.0, nr_others_at_same_pos as f32 * character_size * 0.75);
                vertex_pos + offset
            },
        };

        self.draw_large_at(draw_screen_pos, &con.painter, character_size, style);
        if !drag_enabled {
            return None;
        }

        let was_dragging = matches!(self.pos, Pos::OnScreen(_));
        let now_dragging = 'handle_interaction: {
            let resp = {
                let len = if nr_others_at_same_pos > 0 { 1.0 } else { 3.0 } * character_size;
                let full_rect = Rect::from_center_size(draw_screen_pos, vec2(len, len));
                // only create interaction rectangle if character is actually visible.
                if !con.screen().intersects(full_rect) {
                    break 'handle_interaction false;
                }
                // make sure that the character interaction area does not overlap the side menu
                let point_rect = con.screen().intersect(full_rect);

                let character_id = con.response.id.with(self as *const Self);
                ui.interact(point_rect, character_id, Sense::drag())
            };

            // change a cop's apperance or status in right click menu.
            if let Id::Cop(i) = &mut self.id {
                let mut change = None;
                resp.context_menu(|ui| {
                    if ui
                        .checkbox(&mut self.enabled, "Aktiv")
                        .on_hover_text("berücksichte Cop bei Berechnungen")
                        .clicked()
                    {
                        change = Some(Change::ToggleActive);
                    };
                    if ui.button("🗑 löschen").clicked() {
                        change = Some(Change::Delete);
                    }
                    ui.set_max_width(0.0);
                    ui.separator();
                    for (emoji_i, &emoji) in izip!(0.., Id::COP_EMOJIES) {
                        ui.radio_value(i, emoji_i, emoji);
                    }
                });
                if change.is_some() {
                    return change;
                }
            }
            let now_dragging = resp.dragged_by(egui::PointerButton::Primary);
            if now_dragging {
                // update actual position to match drawn position
                let new_screen_pos = draw_screen_pos + resp.drag_delta();
                let new_pos = move_rect.inverse().transform_pos(new_screen_pos);
                self.pos = Pos::OnScreen(new_pos);
            }
            now_dragging
        };

        // gurantee that release on a new node guarantees _at least two_
        // vertices in self.past_vertices, such that a line depicting this movement can succesfully be drawn.
        if self.past_vertices.is_empty() && self.on_node {
            self.past_vertices.push(self.nearest_vertex);
        }

        if !now_dragging && was_dragging && self.on_node {
            self.pos = Pos::OnVertex(con.positions[self.nearest_vertex]);
            if Some(&self.nearest_vertex) != self.past_vertices.last() {
                //position changed and drag released -> new step
                self.past_vertices.push(self.nearest_vertex);
                return Some(Change::Released);
            }
        }
        if self.update_nearest_vertex(con) {
            return Some(Change::NewPos);
        }

        None
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

    fn update_nearest_vertex(&mut self, con: &DrawContext<'_>) -> bool {
        let Pos::OnScreen(pos2) = self.pos else {
            return false;
        };
        let to_plane = con.cam().to_screen().to_plane;
        let potential = |v: usize| -> f32 {
            let visible_weight = if con.visible[v] { 0.0 } else { 1000.0 };
            let dist_weight = (to_plane.project_pos(con.positions[v]) - pos2).length_sq();
            visible_weight + dist_weight
        };
        {
            // try on the cheap if we can improve the current position.
            // if not, we return immediately without running the expensive part below
            let (local_best, dist_sq) =
                con.edges.find_local_minimum(potential, self.nearest_vertex);
            let now_on_node = dist_sq <= con.tolerance * con.tolerance;
            if local_best == self.nearest_vertex && now_on_node == self.on_node {
                return false;
            }
        }
        let (best_vertex, dist_sq) = con.edges.find_global_minimum(potential);
        let now_on_node = dist_sq <= con.tolerance * con.tolerance;

        let change = best_vertex != self.nearest_vertex || now_on_node != self.on_node;
        self.nearest_vertex = best_vertex;
        self.on_node = now_on_node;
        change
    }

    pub fn update_distances(&mut self, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        queue.clear();
        queue.push_back(self.nearest_vertex);
        self.distances.clear();
        self.distances.resize(edges.nr_vertices(), isize::MAX);
        self.distances[self.nearest_vertex] = 0;
        edges.calc_distances_to(queue, &mut self.distances);
    }

    fn clone_without_distances(&self) -> Self {
        Self {
            past_vertices: self.past_vertices.clone(),
            distances: Vec::new(),
            ..*self
        }
    }

    fn set_vertex_no_dist_update(&mut self, next_v: usize, positions: &[Pos3]) {
        if self.past_vertices.is_empty() {
            self.past_vertices.push(self.vertex());
        }
        self.past_vertices.push(next_v);
        self.nearest_vertex = next_v;
        self.pos = Pos::OnVertex(positions[next_v]);
        self.on_node = true;
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

    /// if active, fst is random generator, snd is time of last step.
    /// these are grouped like this, to make random steps reproducable:
    /// every time the "random steps" option is activated, the Lcg starts with the same state.
    #[serde(skip)]
    random_steps: Option<(crate::rand::Lcg, Instant)>,
    #[serde(skip)]
    nr_random_steps_at_once: usize,
    #[serde(skip)]
    random_update_period: Duration,

    #[serde(skip)]
    pub robber_changed: bool,
    #[serde(skip)]
    pub cop_changed: bool,
}

impl State {
    fn update_character(&mut self, i: usize, edges: &EdgeList, queue: &mut VecDeque<usize>) {
        let ch = &mut self.characters[i];
        ch.update_distances(edges, queue);
        if ch.id().is_robber() {
            self.robber_changed = true;
        } else {
            self.cop_changed = true;
        }
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

            random_steps: None,
            nr_random_steps_at_once: 1,
            random_update_period: Duration::ZERO,

            robber_changed: false,
            cop_changed: false,
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
            self.update_character(i, edges, queue);
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
        for (i, need_up) in izip!(0.., need_update) {
            if need_up {
                self.update_character(i, edges, queue);
            }
        }
    }

    /// redoes move but will not update character distances.
    /// returns index of changed character.
    fn redo_move_without_update(&mut self, positions: &[Pos3]) -> Option<usize> {
        let (i, v) = self.future_moves.pop()?;
        self.past_moves.push((i, v));
        let ch = &mut self.characters[i];
        ch.set_vertex_no_dist_update(v, positions);
        Some(i)
    }

    pub fn redo_move(&mut self, edges: &EdgeList, positions: &[Pos3], queue: &mut VecDeque<usize>) {
        if let Some(i) = self.redo_move_without_update(positions) {
            self.update_character(i, edges, queue);
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
        for (i, need_up) in izip!(0.., need_update) {
            if need_up {
                self.update_character(i, edges, queue);
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
        let (character_index, last_destination) = 'find_period_start: {
            let hist = &self.past_moves[..];
            let mut period_len = usize::min(hist.len() / 3, 8) + 1;
            // find longest period with at least two repetitions
            while period_len > 1 {
                period_len -= 1;

                let newest_hist = &hist[(hist.len() - 3 * period_len)..];
                let fst_period = &newest_hist[..period_len];
                let mid_period = &newest_hist[period_len..(2 * period_len)];
                let last_period = &newest_hist[(2 * period_len)..];
                // the pattern should be more interesting than just the same guy over and over.
                if fst_period.iter().map(|x| x.0).all_equal() {
                    continue;
                }
                let mut same_character_pattern = true;
                for i in 0..period_len {
                    // only which character moved is compared, because
                    // this is vastly easier than in what direction the last moves where
                    // (we have the character indices stored directly, move directions not.)
                    if fst_period[i].0 != last_period[i].0 || fst_period[i].0 != mid_period[i].0 {
                        same_character_pattern = false;
                        break;
                    }
                }
                if same_character_pattern {
                    // this is hit or miss.
                    // to actually choose correctly between mid_period[0] and fst_period[0],
                    // we would need to also find a pattern in the step directions.
                    break 'find_period_start mid_period[0];
                }
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

        ch.set_vertex_no_dist_update(next_v, positions);
        self.update_character(character_index, edges, queue);

        self.past_moves.push((character_index, next_v));
        self.future_moves.clear();
    }

    pub fn make_random_next_step(
        &mut self,
        edges: &EdgeList,
        positions: &[Pos3],
        queue: &mut VecDeque<usize>,
    ) -> bool {
        let mut last_id = self.last_moved().map_or(Id::Robber, |c| c.id());

        let Some((ref mut gen, ref mut step_time)) = self.random_steps else {
            return false;
        };
        let now = Instant::now();
        if (now - *step_time) >= self.random_update_period.mul_f64(0.8) {
            *step_time = now;
        } else {
            return false;
        }

        let character_options =
            self.characters.iter().positions(Character::is_active).collect_vec();
        if character_options.is_empty() {
            return false;
        }
        let mut need_update = vec![false; self.characters.len()];

        let mut change = false;
        let mut with_right_job = Vec::new();
        for _ in 0..self.nr_random_steps_at_once {
            with_right_job.clear();
            with_right_job.extend(
                character_options
                    .iter()
                    .copied()
                    .filter(|&i| !self.characters[i].id().same_job(last_id)),
            );
            if with_right_job.is_empty() {
                last_id = match last_id {
                    Id::Cop(_) => Id::Robber,
                    Id::Robber => Id::Cop(0),
                };
                continue;
            }

            let ch_index = {
                let choice = gen.next() as usize % with_right_job.len();
                with_right_job[choice]
            };
            let ch = &mut self.characters[ch_index];
            last_id = ch.id();

            // choose a step to take
            let last_v = if ch.past_vertices.len() >= 2 {
                debug_assert_eq!(ch.past_vertices.last(), Some(&ch.vertex()));
                ch.past_vertices[ch.past_vertices.len() - 2]
            } else {
                usize::MAX
            };
            let step_options =
                edges.neighbors_of(ch.vertex()).filter(|&n| n != last_v).collect_vec();
            if step_options.is_empty() {
                continue;
            }
            let next_v_index = gen.next() as usize % step_options.len();
            let next_v = step_options[next_v_index];

            // update state
            ch.set_vertex_no_dist_update(next_v, positions);
            need_update[ch_index] = true;
            self.past_moves.push((ch_index, next_v));
            self.future_moves.clear();

            change = true;
        }

        for (i, need_up) in izip!(0.., need_update) {
            if need_up {
                self.update_character(i, edges, queue);
            }
        }

        if self.past_moves.len() >= 20_000 {
            self.forget_move_history();
        }

        change
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

    pub fn new_character_at(&mut self, screen_pos: Pos2, map: &map::Map) {
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
    pub fn draw_menu(
        &mut self,
        ui: &mut Ui,
        style: &mut CharactersStyle,
        map: &map::Map,
        queue: &mut VecDeque<usize>,
    ) -> bool {
        debug_assert_eq!(
            self.past_moves.len(),
            self.characters
                .iter()
                .map(|ch| ch.past_vertices().len().saturating_sub(1))
                .sum()
        );

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
                    self.new_character_at(pos, map);
                    change = true;
                }
                style.0.draw_options(ui, &CharactersStyle::DEFAULT_COLORS);
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
            ui.label(format!(
                "Rundenindex: {}",
                self.active_robber()
                    .map_or(0, |r| r.past_vertices().len().saturating_sub(1))
            ));

            ui.add_space(8.0);
            if ui.button(" 🗑 ").on_hover_text("Spiel vergessen").clicked() {
                self.forget_move_history();
                change = true;
            }

            ui.add_space(8.0);
            let mut make_random_steps = self.random_steps.is_some();
            ui.add(egui::Checkbox::new(
                &mut make_random_steps,
                "zufällige Schritte",
            ));
            {
                let nr = &mut self.nr_random_steps_at_once;
                add_drag_value(ui, nr, "Züge pro Update", 1..=100, 1);

                let mut period = self.random_update_period.as_secs_f32();
                const STEP: f32 = std::f32::consts::SQRT_2;
                add_drag_value(ui, &mut period, "Updaterate [s]", 0.125..=16.0, STEP);
                self.random_update_period = Duration::from_secs_f32(period);
            }
            if make_random_steps != self.random_steps.is_some() {
                self.random_steps =
                    make_random_steps.then(|| (crate::rand::Lcg::new(0), Instant::now()));
            }
            if make_random_steps {
                change |= self.make_random_next_step(map.edges(), map.positions(), queue);
                ui.ctx().request_repaint_after(self.random_update_period);
            }
        });

        change
    }

    /// return true if something changed
    pub fn update_and_draw(
        &mut self,
        ui: &mut Ui,
        style: &CharactersStyle,
        con: &DrawContext<'_>,
        drag_enabled: bool,
        queue: &mut VecDeque<usize>,
    ) {
        type PosVec = smallvec::SmallVec<[usize; 20]>;
        let positions = PosVec::from_iter(self.characters.iter().map(|c| c.nearest_vertex));
        let mut delete = None;

        for i in 0..self.characters.len() {
            let ch = &mut self.characters[i];
            let nr_others_at_same_pos =
                positions[..i].iter().filter(|&&p| p == ch.nearest_vertex).count();

            //always allow to drag, except character sits on backside of graph
            if ch.on_node && !con.visible[ch.nearest_vertex] {
                ch.draw_small_at_node(con, style);
                continue;
            }

            let res = ch.drag_and_draw(ui, con, nr_others_at_same_pos, drag_enabled, style);
            let Some(change) = res else {
                continue;
            };
            match change {
                Change::ToggleActive => {},
                Change::Delete => {
                    delete = Some(i);
                },
                Change::NewPos => {},
                Change::Released => {
                    self.past_moves.push((i, ch.nearest_vertex));
                    self.future_moves.clear();
                },
            }
            self.update_character(i, con.edges, queue);
        }

        if let Some(i) = delete {
            self.forget(i);
            self.characters.remove(i);
        }
    }

    pub fn draw_tails(&self, style: &CharactersStyle, con: &DrawContext<'_>) {
        if !self.show_past_steps {
            return;
        }
        for ch in self.all() {
            let glow = style.glow_color(ch.id());
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

    pub fn draw_allowed_next_steps(&self, style: &CharactersStyle, con: &DrawContext<'_>) {
        if !self.show_allowed_next_steps {
            return;
        }
        let Some(last_moved_character) = self.last_moved() else {
            return;
        };
        let radius = style.size() * con.scale * 6.5;
        for ch in self.all() {
            let name = ch.id();
            if !ch.is_active() || name.same_job(last_moved_character.id()) {
                continue;
            }
            let Some(&v) = ch.past_vertices().last() else {
                continue;
            };
            let stroke = Stroke::new(con.scale * 2.0, style.glow_color(ch.id()));
            for n in con.edges.neighbors_of(v) {
                if con.visible[n] {
                    let draw_pos = con.vertex_draw_pos(n);
                    let marker_circle = egui::Shape::circle_stroke(draw_pos, radius, stroke);
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
