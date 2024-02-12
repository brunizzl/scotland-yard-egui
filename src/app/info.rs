use std::collections::VecDeque;

use itertools::{izip, Itertools};

use egui::*;

use crate::app::character::CharacterState;
use crate::graph::{bruteforce as bf, *};

use self::bruteforce_state::{BruteforceComputationState, GameType};

use super::{color, *};

#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Deserialize, serde::Serialize)]
pub enum VertexColorInfo {
    None,
    Escape1,
    BruteForceRes,
    Escape2,
    NearNodes,
    RobberDist,
    CopDist,
    VertexEquivalenceClasses,
    RobberVertexClass, //equivalence class of the robbers vertices
    CopsRotatedToEquivalence,
    Debugging,
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum VertexNumberInfo {
    None,
    Indices,
    RobberAdvantage,
    EscapeableNodes,
    MinCopDist,
    RobberDist,
    VertexEquivalenceClass,
    Debugging,
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
struct Options {
    manual_marker_colors: [Color32; 8],
    active_manual_marker: usize, //expected in 0..8
    shown_manual_markers: u8,    //bitmask

    automatic_marker_color: Color32,
    vertex_color_info: VertexColorInfo,

    //both are only used, when the respective RobberInfo is active
    marked_cop_dist: isize, //determines cop dist marked in RobberInfo::CopDist
    marked_robber_dist: isize, //determines max dist marked in RobberInfo::SmallRobberDist

    vertex_number_info: VertexNumberInfo,
    show_convex_hull: bool,
    show_hull_boundary: bool,
    draw_vertices: bool,
}

const DEFAULT_OPTIONS: Options = Options {
    manual_marker_colors: color::HAND_PICKED_MARKER_COLORS,
    active_manual_marker: 0,
    shown_manual_markers: u8::MAX,

    automatic_marker_color: color::GREEN,
    vertex_color_info: VertexColorInfo::None,
    marked_cop_dist: 10,
    marked_robber_dist: 10,
    vertex_number_info: VertexNumberInfo::None,
    show_convex_hull: false,
    show_hull_boundary: false,
    draw_vertices: false,
};

impl Options {
    pub fn draw_menu(&mut self, ui: &mut Ui) -> bool {
        let mut menu_change = false;
        ui.collapsing("Knoteninfo", |ui|{
            //obv. wether to draw vertices or not has no influence over any actual information -> no need to update menu change
            ui.add(Checkbox::new(&mut self.draw_vertices, "zeige Knoten"));
            ui.add_space(5.0);

            menu_change |=
                ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige Konvexe Hülle um Cops"))
                .on_hover_text("Berechnung des Randes kann aus Effizienzgründen für sehr kleine / \
                sehr dünne Hüllen fehlerhaft sein. \n\
                Wenn die Cops im 3D Fall den gesamten Graphen durch die Hülle abdecken, wird trotzdem ein Rand gezeigt, da\
                der Punkt am weitesten entfernt von jedem Cop vom Algorithmus hier immer als außerhalb der Hülle angenommen wird.")
                .changed();
            menu_change |= ui.add(Checkbox::new(&mut self.show_hull_boundary, "zeige Grenze"))
                .changed();


            ui.add_space(5.0);
            ui.label("Marker:");
            ui.horizontal(|ui| {
                ui.label("Farbe: ");
                ui.color_edit_button_srgba(&mut self.automatic_marker_color);
                if ui.button("Reset").clicked() {
                    self.automatic_marker_color = DEFAULT_OPTIONS.automatic_marker_color;
                }
            });
            let old = self.vertex_color_info;
            //settings to draw extra information
            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::None,
                "Keine");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::NearNodes,
                "für Räuber nähere Knoten")
                .on_hover_text("alle Knoten näher am Räuber als am nächsten Cop");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::Escape1,
                "Punkte mit direkter Fluchtoption 1")
                .on_hover_text("alle Punkte in der Konvexen Hülle, \
                die näher an einem Punkt ausserhalb der Hülle sind, als der nächste Cop an diesem Punkt ist");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::Escape2,
                "Punkte mit direkter Fluchtoption 2")
                .on_hover_text("Jedes Paar von benauchbarten Cops am Hüllenrand kontrolliert einen Randbereich. \
                Will der Räuber durch diesen Bereich fliehen, dürfen die Cops in der Zeit, \
                die der Räuber zum Rand braucht, diesen nicht auf Länge 0 kürzen können. \n\
                Markiert werden alle Punkte, die schneller an jedem Punkt des Randabschnittes sind, \
                als die Cops diesen Abschnitt dicht machen können.");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::BruteForceRes, "Bruteforce Ergebnis")
                .on_hover_text("Wenn Bruteforce Berechnung ergeben hat, \
                dass der aktuelle Graph vom Räuber gewonnen wird und aktuell so viele Cops \
                aktiv sind wie bei der Bruteforce Rechnung, werden mit dieser Option alle Knoten angezeigt, \
                die dem Räuber für die gegebenen Coppositionen einen Sieg ermöglichen.");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::CopDist,
                "Punkte mit Abstand zu Cops")
                .on_hover_text("Abstand einstellbar bei ausgewählter Option");
            if self.vertex_color_info == VertexColorInfo::CopDist {
                add_drag_value(ui, &mut self.marked_cop_dist, "Abstand: ", 0, 1000);
            }

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::RobberDist,
                "Räuberabstand")
                .on_hover_text("Alle Punkte die eingestellten Abstand zu Räuber haben");
            if self.vertex_color_info == VertexColorInfo::RobberDist {
                add_drag_value(ui, &mut self.marked_robber_dist, "Abstand: ", 0, 1000);
            }

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::VertexEquivalenceClasses,
                "Symmetrieäquivalenzklassen")
                .on_hover_text("Für symmetrische Graphen werden Knoten, die mit einer symmetrierespektierenden \
                Rotation + Spiegelung auf einander abgebildet werden, in die selbe Klasse gesteckt. \
                Das macht Bruteforce etwas weniger speicherintensiv.");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::RobberVertexClass, "Äquivalenzklasse Räuberknoten");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::CopsRotatedToEquivalence,
                "Rotierte Coppositionen")
                .on_hover_text("Coppositionen rotiert auf repräsentative Knoten selber Äquivalenzklasse");

            ui.radio_value(&mut self.vertex_color_info, VertexColorInfo::Debugging,
                "Debugging")
                .on_hover_text("Was auch immer gerade während des letzten mal kompilierens interessant war");

            menu_change |= old != self.vertex_color_info;
        });
        ui.collapsing("Manuelle Marker", |ui| {
            ui.label("[Info]").on_hover_text(
                "Manuelle Marker können an dem der Mausposition nächsten Knoten \
                mit Taste [m] hinzugefügt und [n] entfernt werden.\n\
                Es werden automatish alle manuellen Marker entfernt, wenn der Graph geändert wird.",
            );

            for (i, color) in izip!(0.., &mut self.manual_marker_colors) {
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.active_manual_marker, i, "");
                    ui.color_edit_button_srgba(color);
                    if ui.button("Reset").clicked() {
                        *color = color::HAND_PICKED_MARKER_COLORS[i];
                    }

                    let bit_i = 1u8 << i;
                    let curr_shown = self.shown_manual_markers & bit_i != 0;
                    let mut show = curr_shown;
                    ui.add(Checkbox::new(&mut show, ""));
                    if show {
                        self.shown_manual_markers |= bit_i;
                    } else if curr_shown {
                        self.shown_manual_markers -= bit_i;
                    }
                });
            }
        });
        ui.collapsing("Zahlen", |ui|{
            let old = self.vertex_number_info;
            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::None,
                "Keine");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::Indices,
                "Knotenindizes")
                .on_hover_text("nur relevant für Debugging");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::RobberAdvantage,
                "Marker Fluchtoption 1")
                .on_hover_text("Helfer zur Berechnung von Fluchtoption 1");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::EscapeableNodes,
                "Marker Fluchtoption 2")
                .on_hover_text("jedes benachbarte Cop-Paar auf dem Hüllenrand hat einen Namen in { 0 .. 9, A .. }. \
                Der Marker listet alle Paare auf, zwischen denen der Räuber durchschlüpfen kann.");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::MinCopDist,
                "minimaler Cop Abstand")
                .on_hover_text("punktweises Minimum aus den Abständen aller Cops");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::RobberDist,
                "Räuberabstand")
                .on_hover_text("Abstand von Räuberposition zu jedem Knoten");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::VertexEquivalenceClass,
                "Symmetrieäquivalenzklasse")
                .on_hover_text("Für symmetrische Graphen werden Knoten, die mit einer symmetrierespektierenden \
                Rotation + Spiegelung auf einander abgebildet werden, in die selbe Klasse gesteckt. \
                Das macht Bruteforce etwas weniger speicherintensiv.");

            ui.radio_value(&mut self.vertex_number_info, VertexNumberInfo::Debugging,
                "Debugging")
                .on_hover_text("Überraschungsinfo, die zum letzten Kompilierzeitpunkt \
                gerade spannend zum debuggen war");

            menu_change |= old != self.vertex_number_info;
        });
        menu_change
    }
}

pub struct Info {
    //state kept for each node in map
    cop_hull_data: ConvexHullData,
    escapable: EscapeableNodes,
    /// elementwise minimum of `.distance` of active cops in `self.characters`
    min_cop_dist: Vec<isize>,
    /// outside hull interieur == -min_cop_dist, then expanded into interieur via [`EdgeList::calc_distances_to`]
    /// the robber can escape lazy cops on a continuous hull, if cop_advantage on robber's position is <= -2
    /// as a cop's position has value 0 and a cops neighbors value >= -1.
    cop_advantage: Vec<isize>,
    /// each bit represents one marker color -> there are 8 distinct manual markers
    pub marked_manually: Vec<u8>,

    /// this is only used as intermediary variable during computations.
    /// to not reallocate between frames/ algorithms, the storage is kept and passed to where needed.
    pub queue: VecDeque<usize>,

    pub characters: CharacterState,

    options: Options,
    menu_change: bool,
    take_screenshot: bool,

    worker: BruteforceComputationState,
}

mod storage_keys {
    pub const OPTIONS: &str = "app::info::options";
    pub const CHARACTERS: &str = "app::info::characters";
    pub const MARKED_MANUALLY: &str = "app::info::manually_marked";
}

impl Info {
    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, OPTIONS, &self.options);
        eframe::set_value(storage, CHARACTERS, &self.characters);

        if self.marked_manually.iter().any(|&x| x != 0) {
            eframe::set_value(storage, MARKED_MANUALLY, &self.marked_manually);
        } else {
            let empty = Vec::<u8>::new();
            eframe::set_value(storage, MARKED_MANUALLY, &empty);
        }
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        use storage_keys::*;
        let characters = load_or(cc.storage, CHARACTERS, CharacterState::new);
        let marked_manually = load_or(cc.storage, MARKED_MANUALLY, Vec::new);
        let options = load_or(cc.storage, OPTIONS, || DEFAULT_OPTIONS);

        Self {
            cop_hull_data: ConvexHullData::new(),
            escapable: EscapeableNodes::new(),
            min_cop_dist: Vec::new(),
            cop_advantage: Vec::new(),
            marked_manually,

            queue: VecDeque::new(),

            characters,
            options,
            menu_change: false,
            take_screenshot: false,

            worker: BruteforceComputationState::new(),
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map) {
        self.menu_change = false;
        self.menu_change |= self.options.draw_menu(ui);

        //everything going on here happens on a nother thread -> no need to recompute our data
        //-> no need to log wether something changed
        let nr_cops = self.characters.active_cops().count();
        self.worker.draw_menu(nr_cops, ui, map);

        self.menu_change |= self.characters.draw_menu(ui, map, &mut self.queue);
    }

    fn update_min_cop_dist(&mut self, edges: &EdgeList) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        min_cop_dist.clear();
        let mut active_cops = self.characters.active_cops();
        if let Some(cop) = active_cops.next() {
            min_cop_dist.clone_from(&cop.distances);
        } else {
            min_cop_dist.resize(edges.nr_vertices(), isize::MAX);
        }
        for cop in active_cops {
            for (this, curr_min) in izip!(&cop.distances, &mut min_cop_dist) {
                if this < curr_min {
                    *curr_min = *this;
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
    }

    fn update_cop_advantage(&mut self, edges: &EdgeList) {
        let cop_hull = &self.cop_hull_data.hull();
        let mut advantage = std::mem::take(&mut self.cop_advantage);
        advantage.resize(edges.nr_vertices(), isize::MAX);
        let mut queue = std::mem::take(&mut self.queue);
        queue.clear();

        let zipped = itertools::izip!(
            0..,
            cop_hull.iter(),
            advantage.iter_mut(),
            self.min_cop_dist.iter(),
        );
        for (node, &hull, adv, &cop_dist) in zipped {
            if hull.on_boundary() {
                queue.push_back(node);
            }
            *adv = if hull.in_interieur() {
                isize::MAX
            } else {
                -cop_dist
            };
        }
        edges.calc_distances_to(&mut queue, &mut advantage);
        self.cop_advantage = advantage;
        self.queue = queue;
    }

    fn update_convex_cop_hull(&mut self, con: &DrawContext<'_>) {
        let mut temp = [usize::MAX];
        let vertices_outside_hull = if !con.extreme_vertices.is_empty() {
            con.extreme_vertices
        } else {
            debug_assert_eq!(con.positions.len(), self.min_cop_dist.len());
            let (furthest_vertex, _) =
                self.min_cop_dist.iter().enumerate().fold((0, 0), |best, (v, &dist)| {
                    if dist > best.1 {
                        (v, dist)
                    } else {
                        best
                    }
                });
            temp[0] = furthest_vertex;
            &temp
        };

        self.cop_hull_data.update(
            self.characters.cops(),
            con.edges,
            &self.min_cop_dist,
            &mut self.queue,
            vertices_outside_hull,
        );
    }

    fn update_escapable(&mut self, con: &DrawContext<'_>) {
        self.escapable.update(
            self.characters.cops(),
            &self.cop_hull_data,
            con.edges,
            &mut self.queue,
        )
    }

    /// recomputes everything
    fn definitely_update(&mut self, con: &DrawContext<'_>) {
        self.characters.update(con, &mut self.queue);
        self.update_min_cop_dist(con.edges);
        self.update_convex_cop_hull(con);
        self.update_escapable(con);
        self.update_cop_advantage(con.edges);
    }

    /// recomputes only things currently shown or required by things currently shown
    /// and only if something relevant (e.g. a cop's position) changed
    fn maybe_update(&mut self, con: &DrawContext<'_>) {
        self.characters.update(con, &mut self.queue);
        let robber_moved = self.characters.robber_updated();
        let cop_moved = self.characters.active_cop_updated();

        let update_cop_advantage = self.options.vertex_color_info == VertexColorInfo::Escape1
            || self.options.vertex_color_info == VertexColorInfo::Debugging
            || self.options.vertex_number_info == VertexNumberInfo::RobberAdvantage
            || self.options.vertex_number_info == VertexNumberInfo::Debugging;

        let update_escapable = self.options.vertex_color_info == VertexColorInfo::Escape2
            || self.options.vertex_color_info == VertexColorInfo::Debugging
            || self.options.vertex_number_info == VertexNumberInfo::EscapeableNodes
            || self.options.vertex_number_info == VertexNumberInfo::Debugging;

        let update_hull = update_cop_advantage
            || update_escapable
            || self.options.show_convex_hull
            || self.options.show_hull_boundary;

        let update_min_cop_dist = update_hull
            || self.options.vertex_color_info == VertexColorInfo::CopDist
            || self.options.vertex_color_info == VertexColorInfo::NearNodes
            || self.options.vertex_number_info == VertexNumberInfo::MinCopDist;

        let nr_vertices = con.edges.nr_vertices();
        if (cop_moved || self.min_cop_dist.len() != nr_vertices) && update_min_cop_dist {
            self.update_min_cop_dist(con.edges);
        }
        if (cop_moved || self.cop_hull_data.hull().len() != nr_vertices) && update_hull {
            self.update_convex_cop_hull(con);
        }
        if (cop_moved || self.escapable.escapable().len() != nr_vertices) && update_escapable {
            self.update_escapable(con);
        }
        if (cop_moved || robber_moved || self.cop_advantage.len() != nr_vertices)
            && update_cop_advantage
        {
            self.update_cop_advantage(con.edges);
        }
    }

    fn change_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>, set: bool) {
        let bit = 1u8 << self.options.active_manual_marker;
        if !con.positions.is_empty() {
            let (vertex, _) = con.find_closest_vertex(screen_pos);
            if set {
                self.marked_manually[vertex] |= bit;
            } else if self.marked_manually[vertex] & bit != 0 {
                self.marked_manually[vertex] -= bit;
            }
        }
    }

    fn add_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, true)
    }

    fn remove_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, false)
    }

    fn screenshot_as_tikz(&self, con: &DrawContext<'_>) {
        if !self.take_screenshot || !NATIVE {
            return;
        }
        use chrono::{Datelike, Local, Timelike};
        let now = Local::now();
        let date = now.date_naive();
        let date_str = format!("{}-{}-{}", date.year(), date.month(), date.day());
        let time = now.time();
        let time_str = format!("{}-{}-{}", time.hour(), time.minute(), time.second());
        let file_name =
            std::path::PathBuf::from(format!("screenshots/{}--{}.txt", date_str, time_str));
        super::tikz::draw_to_file(
            file_name,
            &con.painter,
            *con.screen(),
            character::emojis_as_latex_commands(),
        );
    }

    pub fn process_general_input(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        ui.input(|info| {
            self.take_screenshot = info.key_pressed(Key::F2);

            if info.modifiers.ctrl && info.key_pressed(Key::Z) {
                self.characters.reverse_move(con.edges, con.positions, &mut self.queue);
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Y) {
                self.characters.redo_move(con.edges, con.positions, &mut self.queue);
            }
            if info.key_down(Key::M) {
                if let Some(pointer_pos) = info.pointer.latest_pos() {
                    self.add_marker_at(pointer_pos, con);
                }
            }
            if info.key_down(Key::N) {
                if let Some(pointer_pos) = info.pointer.latest_pos() {
                    self.remove_marker_at(pointer_pos, con);
                }
            }
        });
    }

    fn draw_convex_cop_hull(&self, con: &DrawContext<'_>) {
        if self.options.show_convex_hull {
            for (&in_hull, &pos, &vis) in
                izip!(self.cop_hull_data.hull(), con.positions, con.visible)
            {
                if vis && in_hull.in_set() {
                    let draw_pos = con.cam().transform(pos);
                    let marker_circle =
                        Shape::circle_filled(draw_pos, con.scale * 8.0, color::LIGHT_BLUE);
                    con.painter.add(marker_circle);
                }
            }
        }
        if self.options.show_hull_boundary {
            for &v in self.cop_hull_data.boundary() {
                if con.visible[v] {
                    let draw_pos = con.vertex_draw_pos(v);
                    let marker_circle =
                        Shape::circle_filled(draw_pos, con.scale * 2.0, color::WHITE);
                    con.painter.add(marker_circle);
                }
            }
        }
    }

    fn draw_vertices(&self, con: &DrawContext<'_>) {
        if !self.options.draw_vertices {
            return;
        }
        for (&vis, &pos) in izip!(con.visible, con.positions) {
            if vis {
                let draw_pos = con.cam().transform(pos);
                let marker_circle = Shape::circle_filled(draw_pos, con.scale * 4.0, Color32::GRAY);
                con.painter.add(marker_circle);
            }
        }
    }

    fn draw_green_circles(&self, con: &DrawContext<'_>) {
        let draw_circle_at = |pos, color| {
            let draw_pos = con.cam().transform(pos);
            let marker_circle = Shape::circle_filled(draw_pos, con.scale * 6.0, color);
            con.painter.add(marker_circle);
        };
        match self.options.vertex_color_info {
            VertexColorInfo::NearNodes => {
                if let Some(r) = self.characters.robber() {
                    for (r_dist, c_dist, &pos, &vis) in
                        izip!(&r.distances, &self.min_cop_dist, con.positions, con.visible)
                    {
                        if vis && r_dist < c_dist {
                            draw_circle_at(pos, self.options.automatic_marker_color);
                        }
                    }
                }
            },
            VertexColorInfo::Escape1 => {
                for (&adv, &pos, &vis, &hull) in izip!(
                    &self.cop_advantage,
                    con.positions,
                    con.visible,
                    self.cop_hull_data.hull()
                ) {
                    if vis && hull.in_set() && adv < -1 {
                        draw_circle_at(pos, self.options.automatic_marker_color);
                    }
                }
            },
            VertexColorInfo::RobberDist => {
                if let Some(r) = self.characters.robber() {
                    let bnd = self.options.marked_robber_dist;
                    for (&dist, &pos, &vis) in izip!(&r.distances, con.positions, con.visible) {
                        if vis && dist == bnd {
                            draw_circle_at(pos, self.options.automatic_marker_color);
                        }
                    }
                }
            },
            VertexColorInfo::CopDist => {
                for (&dist, &pos, &vis) in izip!(&self.min_cop_dist, con.positions, con.visible) {
                    if vis && dist == self.options.marked_cop_dist {
                        draw_circle_at(pos, self.options.automatic_marker_color);
                    }
                }
            },
            VertexColorInfo::Escape2 => {
                for (&esc, &pos, &vis) in
                    izip!(self.escapable.escapable(), con.positions, con.visible)
                {
                    if vis && esc != 0 {
                        draw_circle_at(
                            pos,
                            color::u32_marker_color(esc, &color::MARKER_COLORS_F32),
                        );
                    }
                }
            },
            VertexColorInfo::BruteForceRes => {
                let game_type = GameType {
                    nr_cops: self.characters.active_cops().count(),
                    resolution: con.map.resolution(),
                    shape: con.map.shape(),
                };
                if let Some(bf::Outcome::RobberWins(data)) = &self.worker.result_for(&game_type) {
                    let mut active_cops =
                        self.characters.active_cops().map(|c| c.nearest_node).collect_vec();

                    let sym_group = data.symmetry.to_dyn();
                    let (_, cop_positions) = data.pack(active_cops.iter().copied());
                    let safe_vertices = data.safe.robber_safe_when(cop_positions);
                    let transform = sym_group.dyn_to_representative(&mut active_cops)[0];
                    for (v, safe) in izip!(0.., safe_vertices) {
                        let v_rot = transform.dyn_apply_backward(v);
                        if safe && con.visible[v_rot] {
                            let rot_pos = con.positions[v_rot];
                            draw_circle_at(rot_pos, self.options.automatic_marker_color);
                        }
                    }
                }
            },
            VertexColorInfo::VertexEquivalenceClasses => {
                if let SymGroup::Explicit(equiv) = con.sym_group() {
                    for (&class, &pos, &vis) in izip!(equiv.classes(), con.positions, con.visible) {
                        if vis {
                            let colors = &super::color::MARKER_COLORS_U8;
                            draw_circle_at(pos, colors[class as usize % colors.len()]);
                        }
                    }
                }
            },
            VertexColorInfo::RobberVertexClass => {
                if let Some(r) = self.characters.robber() {
                    let sym_group = con.sym_group().to_dyn();
                    let v0 = r.nearest_node;
                    let mut f = |transform: &dyn DynAutomorphism| {
                        let sym_v = transform.dyn_apply_forward(v0);
                        if con.visible[sym_v] {
                            let sym_pos = con.positions[sym_v];
                            draw_circle_at(sym_pos, self.options.automatic_marker_color);
                        }
                    };
                    sym_group.for_each_transform(&mut f as &mut dyn FnMut(&dyn DynAutomorphism));
                }
            },
            VertexColorInfo::CopsRotatedToEquivalence => {
                let sym_group = con.sym_group().to_dyn();
                let mut active_cops =
                    self.characters.active_cops().map(|c| c.nearest_node).collect_vec();
                sym_group.dyn_to_representative(&mut active_cops);
                for v in active_cops {
                    let pos = con.positions[v];
                    if con.visible[v] {
                        draw_circle_at(pos, self.options.automatic_marker_color);
                    } else {
                        //only draw half size
                        let draw_pos = con.cam().transform(pos);
                        let marker_circle = Shape::circle_filled(
                            draw_pos,
                            con.scale * 3.0,
                            self.options.automatic_marker_color,
                        );
                        con.painter.add(marker_circle);
                    }
                }
            },
            VertexColorInfo::Debugging => {
                for &v in self.escapable.inner_connecting_line() {
                    if con.visible[v] {
                        let pos = con.positions[v];
                        draw_circle_at(pos, self.options.automatic_marker_color);
                    }
                }
            },
            VertexColorInfo::None => {},
        }
    }

    fn draw_numbers(&self, ui: &Ui, con: &DrawContext<'_>) {
        if self.options.vertex_number_info == VertexNumberInfo::None {
            return;
        }
        let true_bits = |x: u32| -> String {
            const NAMES: [char; 32] = [
                '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
            ];
            izip!(NAMES, 0..)
                .filter_map(|(name, i)| ((1u32 << i) & x != 0).then_some(name))
                .collect()
        };
        let font = FontId::proportional(con.scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode {
            color::WHITE
        } else {
            color::BLACK
        };
        for (v, &pos, &vis) in izip!(0.., con.positions, con.visible) {
            if vis {
                let txt = match self.options.vertex_number_info {
                    VertexNumberInfo::Indices => v.to_string(),
                    VertexNumberInfo::MinCopDist => self.min_cop_dist[v].to_string(),
                    VertexNumberInfo::None => panic!(),
                    VertexNumberInfo::RobberAdvantage => self.cop_advantage[v].to_string(),
                    VertexNumberInfo::EscapeableNodes => true_bits(self.escapable.escapable()[v]),
                    VertexNumberInfo::VertexEquivalenceClass => {
                        if let SymGroup::Explicit(e) = con.sym_group() {
                            e.classes()[v].to_string()
                        } else {
                            String::new()
                        }
                    },
                    VertexNumberInfo::RobberDist => {
                        if let Some(r) = self.characters.robber() {
                            r.distances[v].to_string()
                        } else {
                            String::new()
                        }
                    },
                    //VertexInfoNumber::Debugging => self.escapable.owners()[i].to_string(),
                    //VertexInfoNumber::Debugging => {
                    //    let d = self.escapable.boundary_segment_dist()[v];
                    //    if d == isize::MAX { String::new() } else { d.to_string() }
                    //}
                    VertexNumberInfo::Debugging => {
                        if let SymGroup::Explicit(equiv) = con.sym_group() {
                            equiv.vertex_representatives()[v].to_string()
                        } else {
                            String::new()
                        }
                    },
                };
                let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * con.scale);
                layout_job.halign = Align::Center;
                let galley = ui.fonts(|f| f.layout_job(layout_job));
                let screen_pos = con.cam().transform(pos);
                let text = Shape::Text(TextShape::new(screen_pos, galley, color));
                con.painter.add(text);
            }
        }
    }

    fn draw_character_tails(&self, con: &DrawContext<'_>) {
        if !self.characters.show_steps() {
            return;
        }
        for ch in self.characters.all() {
            let f_len = ch.last_positions.len() as f32;
            let draw_size = |i| con.scale * 2.5 * (i + 0.8 * f_len) / f_len;
            for (i, (&v1, &v2)) in ch.last_positions.iter().tuple_windows().enumerate() {
                if !con.visible[v1] {
                    continue;
                }
                let draw_pos_1 = con.vertex_draw_pos(v1);
                let size = draw_size(i as f32);
                let marker_circle = Shape::circle_filled(draw_pos_1, size, ch.style().glow);
                con.painter.add(marker_circle);
                if !con.visible[v2] {
                    continue;
                }
                let points = [draw_pos_1, con.vertex_draw_pos(v2)];
                let stroke = Stroke::new(size * 0.75, ch.style().glow);
                let line = Shape::LineSegment { points, stroke };
                con.painter.add(line);
            }
            if let Some(&v) = ch.last_positions.last() {
                if con.visible[v] {
                    let draw_pos = con.vertex_draw_pos(v);
                    let size = draw_size(f_len);
                    let marker_circle = Shape::circle_filled(draw_pos, size, ch.style().glow);
                    con.painter.add(marker_circle);
                }
            }
        }
    }

    fn draw_manual_markers(&self, con: &DrawContext<'_>) {
        let mut f32_colors = [color::F32Color::default(); 8];
        let mask = self.options.shown_manual_markers;
        color::zip_to_f32(
            f32_colors.iter_mut(),
            self.options.manual_marker_colors.iter(),
        );
        for (&vis, &marked, &pos) in izip!(con.visible, &self.marked_manually, con.positions) {
            let masked = marked & mask;
            if vis && masked != 0 {
                let draw_pos = con.cam().transform(pos);
                let color = color::u8_marker_color(masked, &f32_colors);
                let marker_circle = Shape::circle_filled(draw_pos, con.scale * 4.5, color);
                con.painter.add(marker_circle);
            }
        }
    }

    pub fn update_and_draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        self.process_general_input(ui, con);
        if self.menu_change {
            self.definitely_update(con);
        } else {
            self.maybe_update(con);
        }
        self.draw_vertices(con);
        self.draw_convex_cop_hull(con);
        self.draw_green_circles(con);
        self.draw_manual_markers(con);
        self.draw_character_tails(con);
        self.draw_numbers(ui, con);
        self.characters.draw(ui, con);
        self.characters.frame_is_finished();
        self.screenshot_as_tikz(con);
    }
}
