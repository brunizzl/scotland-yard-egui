use std::collections::VecDeque;

use itertools::{izip, Itertools};

use strum::IntoEnumIterator;

use egui::{Checkbox, Color32, ComboBox, Label, Pos2, Ui, Vec2};

use crate::geo::Pos3;
use crate::graph::{
    self, bruteforce as bf, Automorphism, EdgeList, Embedding3D, SymGroup, SymmetryGroup,
};

use super::{
    add_disabled_drag_value, add_drag_value,
    bruteforce_state::BruteforceComputationState,
    character::{self, Character, CharactersStyle},
    color, load_or, map,
    style::{self, Style},
    DrawContext, NATIVE,
};

#[derive(
    Clone, Copy, PartialEq, Eq, Debug, serde::Deserialize, serde::Serialize, strum_macros::EnumIter,
)]
pub enum VertexColorInfo {
    None,
    NearNodes,
    SafeOutside,
    SafeBoundary,
    Escape1,
    Escape2,
    Escape2Grid,
    EscapeConeGrid,
    Escape3Grid,
    Escape23Grid,
    BruteForceRes,
    MinCopDist,
    MaxCopDist,
    AnyCopDist,
    RobberDist,
    RobberCone,
    VertexEquivalenceClasses,
    RobberVertexClass, //equivalence class of the robbers vertices
    CopsRotatedToEquivalence,
    CopsVoronoi,
    CopStratPlaneDanger, //makes only sense in infinite plane
    Debugging,
    SpecificVertex,
}

impl VertexColorInfo {
    const fn description(self) -> &'static str {
        use VertexColorInfo::*;
        match self {
            None => "Es werden keine automatisch berechneten Punkte angezeigt",
            NearNodes => "alle Knoten näher am Räuber als am nächsten Cop",
            SafeOutside => "Alle Knoten nicht in konvexer Hülle und mit Mindestabstand >= 2 zu nächstem Cop",
            SafeBoundary => "Alle Knoten auf Rand von konvexer Hülle mit Mindestabstand >= 2 zu nächstem Cop",
            Escape1 => "alle Punkte in der Konvexen Hülle, \
            die näher an einem Punkt ausserhalb der Hülle sind, als der nächste Cop an diesem Punkt ist",
            Escape2 => "Jedes Paar von benauchbarten Cops am Hüllenrand kontrolliert einen Randbereich. \
            Will der Räuber durch diesen Bereich fliehen, dürfen die Cops in der Zeit, \
            die der Räuber zum Rand braucht, diesen nicht auf Länge 0 kürzen können. \n\
            Markiert werden alle Punkte, die schneller an jedem Punkt des Randabschnittes sind, \
            als die Cops diesen Abschnitt dicht machen können.",
            Escape2Grid => "Variante von Escape2 die nur auf Gittern (Zerschnitten + Tori) funktioniert",
            EscapeConeGrid => "Knoten, an denen ein Kegel aus mindestens zwei Richtungen hervorgeht, 
            so dass der Kegel keinen Cop oder Nachbarn eines Cops enthält. (Funktioniert nur auf Gittern)",
            Escape3Grid => "Knoten von denen aus ein \"Fluchtoption 2\" Knoten erreicht werden kann, weil \
            sich mehrere von denen überlappen (funzt noch bei weitem nicht immer. \
            Diese Schätzung kann sowohl zu konservativ, als auch zu generös sein.)",
            Escape23Grid => "Vereinigung aus Fluchtoptionen 2 und 3",
            BruteForceRes => "Wenn Bruteforce Berechnung ergeben hat, \
            dass der aktuelle Graph vom Räuber gewonnen wird und aktuell so viele Cops \
            aktiv sind wie bei der Bruteforce Rechnung, werden mit dieser Option alle Knoten angezeigt, \
            die dem Räuber für die gegebenen Coppositionen einen Sieg ermöglichen.",
            MinCopDist | MaxCopDist | AnyCopDist => "Punktweise, Abstand einstellbar bei ausgewählter Option",
            RobberDist => "Alle Punkte die eingestellten Abstand zu Räuber haben",
            RobberCone => "Kegel beginnend an Räuber in eingestellten Richtungen",
            VertexEquivalenceClasses => "Für symmetrische Graphen werden Knoten, die mit einer symmetrierespektierenden \
            Rotation + Spiegelung auf einander abgebildet werden, in die selbe Klasse gesteckt. \
            Das macht Bruteforce etwas weniger speicherintensiv.",
            RobberVertexClass => "Alle Knoten die in selber Symmetrieäquivalenzklasse sitzen \
            wie die aktuelle Räuberposition",
            CopsRotatedToEquivalence => "Coppositionen rotiert auf repräsentative Knoten selber Äquivalenzklasse",
            CopsVoronoi => "Punkte mit mehreren nähesten Cops. Anzahl Cops von opak zu transparent:\n\
            - drei Cops exakt\n\
            - drei Cops ca. (zwei exakt und einer +1 oder einer exakt und zwei +1)\n\
            - zwei Cops exakt, kein dritter +1\n\
            - zwei Cops ca. (einer exakt und einer +1)",
            CopStratPlaneDanger => "Knoten die Räuber nicht betreten sollte, wenn Cops bestimmte \
            Strategie auf unendlicher Ebene fahren.",
            Debugging => "Was auch immer gerade während des letzten mal kompilierens interessant war",
            SpecificVertex => "Knoten mit bestimmten Index, nur relevant für Debugging",
        }
    }
    const fn name_str(self) -> &'static str {
        use VertexColorInfo::*;
        match self {
            None => "Keine",
            NearNodes => "für Räuber nähere Knoten",
            SafeOutside => "Sicherer Außenbereich",
            SafeBoundary => "Sicherere Grenze",
            Escape1 => "Flucht 1",
            Escape2 => "Flucht 2 (Komponenten)",
            Escape2Grid => "Flucht 2 (Gitter)",
            EscapeConeGrid => "Flucht Kegel (Gitter)",
            Escape3Grid => "Flucht 3 (Gitter)",
            Escape23Grid => "Flucht 2 + 3 (Gitter)",
            BruteForceRes => "Bruteforce Räuberstrategie",
            MinCopDist => "minimaler Cop Abstand",
            MaxCopDist => "maximaler Cop Abstand",
            AnyCopDist => "jeder Cop Abstand",
            RobberDist => "Räuberabstand",
            RobberCone => "Kegel von Räuber (Gitter)",
            VertexEquivalenceClasses => "Symmetrieäquivalenzklassen",
            RobberVertexClass => "Äquivalenzklasse Räuberknoten",
            CopsRotatedToEquivalence => "Rotierte Coppositionen",
            CopsVoronoi => "Cops Voronoi",
            CopStratPlaneDanger => "Cops Ebene Strat Gefahr",
            Debugging => "Debugging",
            SpecificVertex => "einzelner Knoten",
        }
    }
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize, strum_macros::EnumIter)]
pub enum VertexSymbolInfo {
    None,
    Indices,
    RobberAdvantage,
    Escape2,
    Escape2Grid,
    EscapeConeGrid,
    Escape3Grid,
    Escape23Grid,
    MinCopDist,
    MaxCopDist,
    RobberDist,
    VertexEquivalenceClass,
    BruteforceCopMoves,
    Debugging,
}

impl VertexSymbolInfo {
    const fn description(self) -> &'static str {
        use VertexSymbolInfo::*;
        match self {
            None => "Es werden keine Zahlen angezeigt",
            Indices => "nur relevant für Debugging",
            RobberAdvantage => "Helfer zur Berechnung von Fluchtoption 1",
            Escape2 => "jedes benachbarte Cop-Paar auf dem Hüllenrand hat einen Namen in { 0 .. 9, A .. }. \
            Der Marker listet alle Paare auf, zwischen denen der Räuber durchschlüpfen kann.",
            Escape2Grid => "alle Richtungen, in die der Räuber fliehen kann.",
            EscapeConeGrid => "jede Richtung, die in einem Fluchtkegel enthalten ist als Pfeil.",
            Escape23Grid => "Kombination aus Fluchtoption 2 + 3 auf Gitter.",
            Escape3Grid => "eine dieser Richtungen führt garantiert wieder auf einen Dilemmaknoten",
            MinCopDist => "punktweises Minimum aus den Abständen aller Cops",
            MaxCopDist => "punktweises Maximum aus den Abständen aller Cops",
            VertexEquivalenceClass => "Für symmetrische Graphen werden Knoten, die mit einer symmetrierespektierenden \
            Rotation + Spiegelung auf einander abgebildet werden, in die selbe Klasse gesteckt. \
            Das macht Bruteforce etwas weniger speicherintensiv.",
            RobberDist => "Abstand von Räuberposition zu jedem Knoten",
            BruteforceCopMoves => "Wenn Cops optimal ziehen, lebt Räuber noch maximal so viele Züge",
            Debugging => "Überraschungsinfo, die zum letzten Kompilierzeitpunkt \
            gerade spannend zum debuggen war",
        }
    }

    const fn name_str(self) -> &'static str {
        use VertexSymbolInfo::*;
        match self {
            None => "Keine",
            Indices => "Knotenindizes",
            RobberAdvantage => "Marker Flucht 1",
            Escape2 => "Marker Flucht 2",
            Escape2Grid => "Pfeile Flucht 2 (Gitter)",
            EscapeConeGrid => "Pfeile Flucht Kegel (Gitter)",
            Escape3Grid => "Pfeile Flucht 3 (Gitter)",
            Escape23Grid => "Pfeile Flucht 2 & 3 (Gitter)",
            MinCopDist => "minimaler Cop Abstand",
            MaxCopDist => "maximaler Cop Abstand",
            VertexEquivalenceClass => "Symmetrieäquivalenzklasse",
            RobberDist => "Räuberabstand",
            BruteforceCopMoves => "Bruteforce Cop Züge",
            Debugging => "Debugging",
        }
    }
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
struct Options {
    vertex_style: Style<1>,
    manual_marker_sizes: [f32; 8],
    manual_marker_colors: [Color32; 8],
    automatic_marker_style: Style<1>,
    hull_style: Style<1>,
    hull_boundary_style: Style<1>,
    number_style: Style<0>,
    character_style: CharactersStyle,

    active_manual_marker: usize, //expected in 0..8
    shown_manual_markers: u8,    //bitmask

    /// currently selected one at index 0
    last_selected_vertex_color_infos: [VertexColorInfo; 7],
    /// currently selected one at index 0
    last_selected_vertex_number_infos: [VertexSymbolInfo; 7],

    //these are only used, when the respective VertexColorInfo(s) is/are active
    marked_cop_dist: isize, //determines cop dist marked in VertexColorInfo::{Max/Min/Any}CopDist
    marked_robber_dist: isize, //determines max dist marked in VertexColorInfo::RobberDist
    #[serde(skip)]
    specific_shown_vertex: usize,
    shown_escape_directions: u8,

    show_convex_hull: bool,
    show_hull_boundary: bool,
    draw_vertices: bool,
    show_cop_strat: bool,
    show_manual_marker_window: bool,
    combine_manual_marker_colors: bool,
}

const DEFAULT_OPTIONS: Options = Options {
    vertex_style: Style::new(&[Color32::GRAY]),
    manual_marker_sizes: [1.0; 8],
    manual_marker_colors: color::HAND_PICKED_MARKER_COLORS,
    automatic_marker_style: Style::new(&[color::GREEN]),
    hull_style: Style::new(&[color::LIGHT_BLUE]),
    hull_boundary_style: Style::new(&[color::WHITE]),
    number_style: Style::new(&[]),
    character_style: CharactersStyle::DEFAULT,

    active_manual_marker: 0,
    shown_manual_markers: u8::MAX,

    marked_cop_dist: 10,
    marked_robber_dist: 10,
    specific_shown_vertex: 0,
    shown_escape_directions: 63,

    last_selected_vertex_color_infos: [VertexColorInfo::None; 7],
    last_selected_vertex_number_infos: [VertexSymbolInfo::None; 7],

    show_convex_hull: false,
    show_hull_boundary: false,
    draw_vertices: false,
    show_cop_strat: false,
    show_manual_marker_window: false,
    combine_manual_marker_colors: true,
};

impl Default for Options {
    fn default() -> Self {
        DEFAULT_OPTIONS
    }
}

impl Options {
    pub fn vertex_color_info(&self) -> VertexColorInfo {
        self.last_selected_vertex_color_infos[0]
    }

    pub fn vertex_number_info(&self) -> VertexSymbolInfo {
        self.last_selected_vertex_number_infos[0]
    }

    fn draw_drag_value(&mut self, ui: &mut Ui, draw_color_val: bool) {
        #[derive(PartialEq, Eq, PartialOrd, Ord)]
        enum ShownValue {
            None,
            MarkedCopDist,
            MarkedRobberDist,
            VertexIndex,
            DirectionBits,
        }
        let val = if draw_color_val {
            match self.vertex_color_info() {
                VertexColorInfo::MinCopDist
                | VertexColorInfo::MaxCopDist
                | VertexColorInfo::AnyCopDist => ShownValue::MarkedCopDist,
                VertexColorInfo::RobberDist => ShownValue::MarkedRobberDist,
                VertexColorInfo::SpecificVertex => ShownValue::VertexIndex,
                VertexColorInfo::Escape2Grid
                | VertexColorInfo::EscapeConeGrid
                | VertexColorInfo::RobberCone => ShownValue::DirectionBits,
                _ => ShownValue::None,
            }
        } else {
            match self.vertex_number_info() {
                VertexSymbolInfo::Escape2Grid
                | VertexSymbolInfo::Escape3Grid
                | VertexSymbolInfo::EscapeConeGrid
                | VertexSymbolInfo::Escape23Grid => ShownValue::DirectionBits,
                _ => ShownValue::None,
            }
        };
        match val {
            ShownValue::MarkedCopDist => {
                add_drag_value(ui, &mut self.marked_cop_dist, "Abstand", (0, 1000), 1);
            },
            ShownValue::MarkedRobberDist => {
                add_drag_value(ui, &mut self.marked_robber_dist, "Abstand", (0, 1000), 1);
            },
            ShownValue::VertexIndex => {
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.specific_shown_vertex));
                    ui.label("Index");
                });
            },
            ShownValue::DirectionBits => {
                let mut shown = crate::graph::grid::Dirs(self.shown_escape_directions);
                ui.horizontal(|ui| {
                    if ui.button(" - ").clicked() {
                        shown = shown.rotate_left_hex();
                    }
                    ui.add(egui::DragValue::new(&mut shown.0).range(0..=63).binary(6, true));
                    if ui.button(" + ").clicked() {
                        shown = shown.rotate_right_hex();
                    }
                    ui.label("Richtungen").on_hover_text(
                        "Richtungen werden als Bitset gespeichert. \
                        Bits für Richtungen e₃ und -e₃ werden auf Vierecksgitter ignoriert.",
                    );
                });
                self.shown_escape_directions = shown.0;
            },
            ShownValue::None => {
                add_disabled_drag_value(ui);
            },
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) -> bool {
        let mut menu_change = false;
        ui.collapsing("Knoteninfo", |ui| {
            //obv. whether to draw vertices or not has no influence over any actual information -> no need to update menu change
            ui.horizontal(|ui| {
                ui.add(Checkbox::new(&mut self.draw_vertices, "Knoten"));
                self.vertex_style.draw_options(ui, &[Color32::GRAY]);
            });
            ui.add_space(8.0);


            ui.horizontal(|ui| {
                menu_change |= ui
                    .add(Checkbox::new(&mut self.show_convex_hull, "konvexe Hülle um Cops"))
                    .on_hover_text(
                        "Wenn die Cops im 3D/Torus- Fall den gesamten Graphen durch die Hülle abdecken, \
                    wird trotzdem ein Rand gezeigt, da der Punkt am weitesten entfernt von jedem Cop vom Algorithmus \
                    hier immer als außerhalb der Hülle angenommen wird.",
                    )
                    .changed();
                self.hull_style.draw_options(ui, &[color::LIGHT_BLUE]);
            });

            ui.horizontal(|ui| {
                menu_change |= ui
                .add(Checkbox::new(
                    &mut self.show_hull_boundary,
                    "Grenze konvexer Hülle",
                ))
                .on_hover_text(
                    "Punkte in Hülle, die mindestens einen Nachbarn ausserhalb Hülle haben. \
                Weil diese Punkte nicht in Isolation interessant sind, sondern als Kreis um die Hülle, \
                wird ein Punkt nur als Randpunkt makiert, wenn er erfolgreich in den Kreis um die Hülle \
                aufgenommen werden konnte.",
                )
                .changed();
                self.hull_boundary_style.draw_options(ui, &[color::WHITE]);
            });

            ui.add_space(8.0);
            ui.add(Checkbox::new(&mut self.show_cop_strat, "Polizeistrategie"))
                .on_hover_text(
                    "Wenn für aktuelle Anzahl Cops & für aktuellen Graphen Bruteforce \
                Polizeistrategie berechnet wurde und aktueller Spielstate von Cops \
                gewonnen wird, werden alle idealen Züge angezeigt."
                );


            ui.add_space(8.0);
            ui.horizontal(|ui| {
                ui.label("Marker:");
                self.automatic_marker_style.draw_options(ui, &[color::GREEN]);
            });
            ComboBox::from_id_source(&self.last_selected_vertex_color_infos as *const _)
                .selected_text(self.vertex_color_info().name_str())
                .show_ui(ui, |ui| {
                    super::style::close_options_menu();

                    let mut curr = self.vertex_color_info();
                    for val in VertexColorInfo::iter() {
                        ui.radio_value(&mut curr, val, val.name_str())
                            .on_hover_text(val.description());
                    }
                    if curr != self.vertex_color_info() {
                        let infos = &mut self.last_selected_vertex_color_infos;
                        let new_pos = infos.iter().position(|&i| i == curr).unwrap_or(infos.len() - 1);
                        infos[..=new_pos].rotate_right(1);
                        debug_assert!(!infos.contains(&curr) || infos[0] == curr);
                        infos[0] = curr;

                        menu_change = true;
                    }
                }).response.on_hover_text("rotiere durch letzte mit [Q] + [1]/[2]/[3]/[4]");
            self.draw_drag_value(ui, true);


            ui.add_space(8.0);
            ui.horizontal(|ui| {
                ui.label("Symbole:");
                self.number_style.draw_options(ui, &[]);
            });
            ComboBox::from_id_source(&self.last_selected_vertex_number_infos as *const _)
                .selected_text(self.vertex_number_info().name_str())
                .show_ui(ui, |ui| {
                    super::style::close_options_menu();

                    let mut curr = self.vertex_number_info();
                    for val in VertexSymbolInfo::iter() {
                        ui.radio_value(&mut curr, val, val.name_str())
                            .on_hover_text(val.description());
                    }
                    if curr != self.vertex_number_info() {
                        let infos = &mut self.last_selected_vertex_number_infos;
                        let new_pos = infos.iter().position(|&i| i == curr).unwrap_or(infos.len() - 1);
                        infos[..=new_pos].rotate_right(1);
                        debug_assert!(!infos.contains(&curr) || infos[0] == curr);
                        infos[0] = curr;

                        menu_change = true;
                    }
                }).response.on_hover_text("rotiere durch letzte mit [W] + [1]/[2]/[3]/[4]");
            self.draw_drag_value(ui, false);
        });

        menu_change
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum MouseTool {
    Drag,
    Draw,
    Erase,
    Paintbucket,
}

impl MouseTool {
    fn symbol(self) -> &'static str {
        match self {
            MouseTool::Drag => " ♟ ",
            MouseTool::Draw => " ✏ ",
            MouseTool::Erase => " 📗 ",
            MouseTool::Paintbucket => " 💦 ",
        }
    }

    fn what(self) -> &'static str {
        match self {
            MouseTool::Drag => "bewege Figuren ([E] + [1])",
            MouseTool::Draw => "zeichne ([E] + [2])",
            MouseTool::Erase => "radiere ([E] + [3])",
            MouseTool::Paintbucket => {
                "Farbeimer ([E] + [4])\n\
                Verschiedene Farben interagieren nicht miteinander.\n\
                Klicken auf eine bereits markierte Region löscht diese."
            },
        }
    }

    pub const ALL: [Self; 4] = [Self::Drag, Self::Draw, Self::Erase, Self::Paintbucket];

    /// input is old self, result new one. updating stored information must be done elsewhere
    pub fn draw_mouse_tool_controls(self, ui: &mut Ui) -> Self {
        let mut new = self;
        ui.horizontal(|ui| {
            ui.label("Werkzeug: ").on_hover_text(
                "Manuelle Marker der aktiven Farbe können an dem der Mausposition nächsten Knoten \
                mit Klicken bei Auswahl des [✏]-Werkzeuges hinzugefügt und bei Auswahl von [📗] entfernt werden. \
                Alternativ: Setzten mit [m] und Entfernen mit [n]. \
                Es werden automatish alle manuellen Marker entfernt, wenn der Graph geändert wird."
            );
            for t in MouseTool::ALL {
                let button = egui::Button::new(t.symbol()).selected(self == t);
                if ui.add(button).on_hover_text(t.what()).clicked() {
                    new = t;
                }
            }
        });
        new
    }
}

pub struct Info {
    tool: MouseTool,

    ///remembers all vertices currently colored by `VertexColorInfo` of `self.options`
    currently_marked: Vec<bool>,

    //state kept for each node in map
    cop_hull_data: graph::CopsHull,
    escapable: graph::EscapableNodes,
    escapable_grid: graph::EscapableDirections,
    dilemma: graph::DilemmaNodes,
    plane_cop_strat: graph::PlaneCopStat,

    /// elementwise minimum of `.distance` of active cops in `self.characters`
    min_cop_dist: Vec<isize>,
    /// elementwise maximum of `.distance` of active cops in `self.characters`
    max_cop_dist: Vec<isize>,
    /// outside hull interieur == -min_cop_dist, then expanded into interieur via [`EdgeList::calc_distances_to`]
    /// the robber can escape lazy cops on a continuous hull, if cop_advantage on robber's position is <= -2
    /// as a cop's position has value 0 and a cops neighbors value >= -1.
    /// as it turns out, values <= -2 are found at exactly those vertices,
    /// which are contained in "winning stage II vertices" defined in the thesis.
    cop_advantage: Vec<isize>,
    /// each bit represents one marker color -> there are 8 distinct manual markers
    pub marked_manually: Vec<u8>,

    /// this is only used as intermediary variable during computations.
    /// to not reallocate between frames/ algorithms, the storage is kept and passed to where needed.
    pub queue: VecDeque<usize>,

    pub characters: character::State,

    options: Options,
    frame_number: usize,
    last_change_frame: usize,

    worker: BruteforceComputationState,

    take_screenshot: bool,
    screenshot_name: String,
}

mod storage_keys {
    pub const OPTIONS: &str = "app::info::options";
    pub const CHARACTERS: &str = "app::info::characters";
    pub const MARKED_MANUALLY: &str = "app::info::manually_marked";
}

impl Default for Info {
    fn default() -> Self {
        Self {
            tool: MouseTool::Drag,
            currently_marked: Vec::new(),

            cop_hull_data: graph::CopsHull::new(),
            escapable: graph::EscapableNodes::new(),
            escapable_grid: graph::EscapableDirections::new(),
            dilemma: graph::DilemmaNodes::new(),
            plane_cop_strat: graph::PlaneCopStat::new(),
            min_cop_dist: Vec::new(),
            max_cop_dist: Vec::new(),
            cop_advantage: Vec::new(),
            marked_manually: Vec::new(),

            queue: VecDeque::new(),

            characters: character::State::new(),
            options: DEFAULT_OPTIONS,
            last_change_frame: 0,
            frame_number: 0,

            worker: BruteforceComputationState::new(),

            take_screenshot: false,
            screenshot_name: String::new(),
        }
    }
}

impl Info {
    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, OPTIONS, &self.options);
        eframe::set_value(storage, CHARACTERS, &self.characters);
        let rle_manually = crate::rle::encode(&self.marked_manually);
        eframe::set_value(storage, MARKED_MANUALLY, &rle_manually);
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        use storage_keys::*;
        let characters = load_or(cc.storage, CHARACTERS, character::State::new);
        let rle_manually = load_or(cc.storage, MARKED_MANUALLY, Vec::new);
        let marked_manually = crate::rle::decode(&rle_manually);
        let options = load_or(cc.storage, OPTIONS, || DEFAULT_OPTIONS);

        Self {
            tool: MouseTool::Drag,
            currently_marked: Vec::new(),

            cop_hull_data: graph::CopsHull::new(),
            escapable: graph::EscapableNodes::new(),
            escapable_grid: graph::EscapableDirections::new(),
            dilemma: graph::DilemmaNodes::new(),
            plane_cop_strat: graph::PlaneCopStat::new(),
            min_cop_dist: Vec::new(),
            max_cop_dist: Vec::new(),
            cop_advantage: Vec::new(),
            marked_manually,

            queue: VecDeque::new(),

            characters,
            options,
            last_change_frame: 0,
            frame_number: 0,

            worker: BruteforceComputationState::new(),

            take_screenshot: false,
            screenshot_name: String::new(),
        }
    }

    pub fn register_change_now(&mut self) {
        self.last_change_frame = self.frame_number;
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map) {
        let mut change = self.options.draw_menu(ui);

        //everything going on here happens on a nother thread -> no need to recompute our data
        //-> no need to log wether something changed
        let nr_cops = self.characters.active_cops().count();
        self.worker.draw_menu(nr_cops, ui, map);
        if NATIVE {
            ui.collapsing("📷 Screenshots", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Name: ");
                    ui.text_edit_singleline(&mut self.screenshot_name);
                });
                self.take_screenshot = ui.button("Aufnehmen").clicked();
                ui.add_space(5.0);
            });
        }

        change |=
            self.characters
                .draw_menu(ui, &mut self.options.character_style, map, &mut self.queue);
        if change {
            self.register_change_now();
        }
    }

    fn change_tool_to(&mut self, new: MouseTool) {
        if new != self.tool && new != MouseTool::Drag {
            self.options.show_manual_marker_window = true;
        }
        self.tool = new;
    }

    pub fn draw_mouse_tool_controls(&mut self, ui: &mut Ui) {
        let new = self.tool.draw_mouse_tool_controls(ui);
        self.change_tool_to(new);
    }

    pub fn draw_windows(&mut self, ctx: &egui::Context) {
        let opts = &mut self.options;
        let automatic_markers_shown = opts.vertex_color_info() != VertexColorInfo::None;
        let hull_shown = opts.show_convex_hull;
        let mut new_tool = self.tool;
        egui::Window::new("Manuelle Marker")
            .open(&mut opts.show_manual_marker_window)
            .constrain_to(ctx.screen_rect())
            .show(ctx, |ui| {
                new_tool = new_tool.draw_mouse_tool_controls(ui);
                ui.horizontal(|ui| {
                    ui.add(Checkbox::new(
                        &mut opts.combine_manual_marker_colors,
                        "kombiniere Marker",
                    ))
                    .on_hover_text(
                        "Die Farben aller Marker eines Knotens werden gemischt angezeigt\n\
                        und alle Marker haben die Größe des ersten Markers.",
                    );
                    if opts.combine_manual_marker_colors {
                        style::draw_options(ui, &mut opts.manual_marker_sizes[0], &mut [], &[]);
                    }
                });

                for (i, size) in izip!(0.., &mut opts.manual_marker_sizes) {
                    ui.horizontal(|ui| {
                        let bit_i = 1u8 << i;
                        let hover_choose = format!("wähle Farbe (f + {})", i + 1);
                        if ui
                            .radio(opts.active_manual_marker == i, "")
                            .on_hover_text(hover_choose)
                            .clicked()
                        {
                            opts.active_manual_marker = i;
                            opts.shown_manual_markers |= bit_i;
                        }
                        let mut show = opts.shown_manual_markers & bit_i != 0;
                        if ui.checkbox(&mut show, "").on_hover_text("Anzeigen").clicked() {
                            opts.shown_manual_markers ^= bit_i;
                            debug_assert_eq!(show, (opts.shown_manual_markers & bit_i) != 0);
                        };

                        {
                            let color = &mut opts.manual_marker_colors[i];
                            ui.color_edit_button_srgba(color);
                            if !opts.combine_manual_marker_colors {
                                let active = &mut opts.manual_marker_colors[i..(i + 1)];
                                let default = &color::HAND_PICKED_MARKER_COLORS[i..(i + 1)];
                                style::draw_options(ui, size, active, default);
                            } else if ui.button("Reset").clicked() {
                                *color = color::HAND_PICKED_MARKER_COLORS[i];
                            }
                        }
                        if ui.button(" 🗑 ").on_hover_text("diese Marker löschen").clicked() {
                            let mask = u8::MAX - bit_i;
                            for marker in &mut self.marked_manually {
                                *marker &= mask;
                            }
                        }

                        let what_set = match () {
                            () if automatic_markers_shown => "automatische Marker",
                            () if hull_shown => "Konvexe Hülle",
                            () => "aktive manuelle Marker",
                        };
                        enum Op {
                            Add,
                            Prod,
                            Sub,
                        }
                        for operation in [Op::Add, Op::Prod, Op::Sub] {
                            let hint = match operation {
                                Op::Add => format!("füge {what_set} hinzu (Vereinigung)"),
                                Op::Prod => format!("behalte nur {what_set} (Schnitt)"),
                                Op::Sub => format!("entferne {what_set} (Differenz)"),
                            };
                            let name = match operation {
                                Op::Add => " + ",
                                Op::Prod => " · ",
                                Op::Sub => " - ",
                            };
                            if ui.button(name).on_hover_text(hint).clicked() {
                                let apply = |marker: &mut u8, other: bool| match operation {
                                    Op::Add => {
                                        if other {
                                            *marker |= bit_i;
                                        }
                                    },
                                    Op::Prod => {
                                        if !other && (*marker & bit_i != 0) {
                                            *marker -= bit_i;
                                        }
                                    },
                                    Op::Sub => {
                                        if other && (*marker & bit_i != 0) {
                                            *marker -= bit_i;
                                        }
                                    },
                                };
                                if automatic_markers_shown {
                                    let iter =
                                        izip!(&self.currently_marked, &mut self.marked_manually);
                                    for (&set, marker) in iter {
                                        apply(marker, set);
                                    }
                                } else if hull_shown {
                                    let iter =
                                        izip!(self.cop_hull_data.hull(), &mut self.marked_manually);
                                    for (&hull, marker) in iter {
                                        apply(marker, hull.contained());
                                    }
                                } else {
                                    let mask = 1u8 << opts.active_manual_marker;
                                    for marker in &mut self.marked_manually {
                                        apply(marker, *marker & mask != 0);
                                    }
                                }
                            }
                        }
                    });
                }
            });
        self.change_tool_to(new_tool);
    }

    fn update_min_cop_dist(&mut self, edges: &EdgeList) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        min_cop_dist.clear();
        min_cop_dist.resize(edges.nr_vertices(), isize::MAX);
        let mut active_cops = self.characters.active_cops();
        if let Some(cop) = active_cops.next() {
            min_cop_dist.clone_from_slice(cop.dists());
        }
        for cop in active_cops {
            for (this, curr_min) in izip!(cop.dists(), &mut min_cop_dist) {
                if this < curr_min {
                    *curr_min = *this;
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
    }

    fn update_max_cop_dist(&mut self, edges: &EdgeList) {
        let mut max_cop_dist = std::mem::take(&mut self.max_cop_dist);
        max_cop_dist.clear();
        max_cop_dist.resize(edges.nr_vertices(), isize::MIN);
        let mut active_cops = self.characters.active_cops();
        if let Some(cop) = active_cops.next() {
            max_cop_dist.clone_from_slice(cop.dists());
        }
        for cop in active_cops {
            for (this, curr_max) in izip!(cop.dists(), &mut max_cop_dist) {
                if this > curr_max {
                    *curr_max = *this;
                }
            }
        }
        self.max_cop_dist = max_cop_dist;
    }

    /// this is not a direct implementation of the characterisation of
    /// "winning stage II vertices" given in the thesis,
    /// but instead an equilavent formulation which gives rise to a linear time algorithm:
    /// the function `f: vertices -> N` is computed as follows.
    /// `
    /// f(v) := if not v in cop hull interior {
    ///     -min dist(v, cop) over all cops
    /// } else {
    ///     (min f(v') over all v' neighbors of v) + 1
    /// }
    /// `
    fn update_cop_advantage(&mut self, edges: &EdgeList) {
        assert_eq!(edges.nr_vertices(), self.cop_hull_data.hull().len());
        assert_eq!(edges.nr_vertices(), self.min_cop_dist.len());

        self.cop_advantage.clear();
        self.cop_advantage.resize(edges.nr_vertices(), isize::MAX);
        self.queue.clear();

        let iter = izip!(
            0..,
            self.cop_hull_data.hull(),
            self.cop_advantage.iter_mut(),
            self.min_cop_dist.iter(),
        );
        for (v, &hull, adv, &cop_dist) in iter {
            if hull.on_boundary() {
                self.queue.push_back(v);
            }
            if !hull.in_interieur() {
                *adv = -cop_dist;
            }
        }
        edges.calc_distances_to(&mut self.queue, &mut self.cop_advantage);
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
            &mut self.queue,
            vertices_outside_hull,
            &self.min_cop_dist,
        );
        //self.cop_hull_data.update(
        //    self.characters.cops(),
        //    con.edges,
        //    &self.min_cop_dist,
        //    &mut self.queue,
        //    vertices_outside_hull,
        //);
    }

    fn update_escapable(&mut self, con: &DrawContext<'_>) {
        self.escapable.update(
            con.map.shape(),
            self.characters.cops(),
            &self.cop_hull_data,
            con.edges,
            &mut self.queue,
        );
    }

    fn update_escapable_grid(&mut self, con: &DrawContext<'_>) {
        let active_cops = self.characters.active_cops().collect_vec();
        self.escapable_grid.update(
            con.map.data(),
            &mut self.queue,
            &self.cop_hull_data,
            &active_cops,
        );
    }

    fn update_dilemma(&mut self, con: &DrawContext<'_>) {
        let active_cops = self.characters.active_cops().collect_vec();
        self.dilemma.update(
            con.edges,
            self.cop_hull_data.hull(),
            &self.escapable_grid,
            &mut self.queue,
            &active_cops,
        );
    }

    fn update_plane_cop_strat(&mut self, con: &DrawContext<'_>) {
        self.plane_cop_strat.update(
            con.map.shape(),
            con.edges,
            self.cop_hull_data.hull(),
            self.escapable.escapable(),
            &self.characters,
            &mut self.queue,
        );
    }

    pub fn adjust_to_new_map(&mut self, map: &Embedding3D) {
        for ch in self.characters.all_mut() {
            ch.adjust_to_new_map(map, &mut self.queue);
        }

        let nr_vertices = map.nr_vertices();
        if self.marked_manually.len() != nr_vertices {
            self.characters.forget_move_history();
            self.marked_manually.clear();
            self.marked_manually.resize(nr_vertices, 0);
        }
        self.register_change_now();
    }

    /// recomputes everything
    pub fn definitely_update(&mut self, con: &DrawContext<'_>) {
        self.update_min_cop_dist(con.edges);
        self.update_max_cop_dist(con.edges);
        self.update_convex_cop_hull(con);
        self.update_escapable(con);
        self.update_escapable_grid(con);
        self.update_dilemma(con);
        self.update_cop_advantage(con.edges);
        self.update_plane_cop_strat(con);
    }

    /// recomputes only things currently shown or required by things currently shown
    /// and only if something relevant (e.g. a cop's position) changed
    fn maybe_update(&mut self, con: &DrawContext<'_>) {
        use VertexColorInfo as Color;
        use VertexSymbolInfo as Symbol;

        let nr_vertices = con.edges.nr_vertices();
        let robber_moved = self.characters.robber_changed;
        let cop_moved = self.characters.cop_changed;

        let color = self.options.vertex_color_info();
        let symbol = self.options.vertex_number_info();
        let show_debug = matches!(color, Color::Debugging) || matches!(symbol, Symbol::Debugging);

        debug_assert_eq!(self.cop_advantage.len(), nr_vertices);
        let update_cop_advantage = show_debug
            || matches!(color, Color::Escape1)
            || matches!(symbol, Symbol::RobberAdvantage);

        debug_assert_eq!(self.plane_cop_strat.danger_zones().len(), nr_vertices);
        let update_plane_cop_strat = show_debug || matches!(color, Color::CopStratPlaneDanger);

        debug_assert_eq!(self.escapable.escapable().len(), nr_vertices);
        let update_escapable = show_debug
            || update_plane_cop_strat
            || matches!(color, Color::Escape2)
            || matches!(symbol, Symbol::Escape2);

        debug_assert_eq!(self.dilemma.overlap.len(), nr_vertices);
        let update_dilemma = show_debug
            || matches!(color, Color::Escape3Grid | Color::Escape23Grid)
            || matches!(symbol, Symbol::Escape3Grid | Symbol::Escape23Grid);

        debug_assert_eq!(self.escapable_grid.esc_directions.len(), nr_vertices);
        let update_esc_grid = show_debug
            || update_plane_cop_strat
            || update_dilemma
            || matches!(
                color,
                Color::Escape2Grid | Color::Escape2 | Color::EscapeConeGrid
            )
            || matches!(
                symbol,
                Symbol::Escape2Grid | Symbol::Escape2 | Symbol::EscapeConeGrid
            );

        debug_assert_eq!(self.cop_hull_data.hull().len(), nr_vertices);
        let update_hull = show_debug
            || update_cop_advantage
            || update_escapable
            || update_esc_grid
            || matches!(color, Color::SafeOutside | Color::SafeBoundary)
            || self.options.show_convex_hull
            || self.options.show_hull_boundary;

        debug_assert_eq!(self.min_cop_dist.len(), nr_vertices);
        let update_min_cop_dist = show_debug
            || update_hull
            || matches!(
                color,
                Color::MinCopDist | Color::NearNodes | Color::CopsVoronoi
            )
            || symbol == Symbol::MinCopDist;

        debug_assert_eq!(self.max_cop_dist.len(), nr_vertices);
        let update_max_cop_dist = show_debug
            || self.options.vertex_number_info() == Symbol::MaxCopDist
            || color == Color::MaxCopDist;

        if cop_moved && update_max_cop_dist {
            self.update_max_cop_dist(con.edges);
        }
        if cop_moved && update_min_cop_dist {
            self.update_min_cop_dist(con.edges);
        }
        if cop_moved && update_hull {
            self.update_convex_cop_hull(con);
        }
        if cop_moved && update_escapable {
            self.update_escapable(con);
        }
        if cop_moved && update_esc_grid {
            self.update_escapable_grid(con);
        }
        if cop_moved && update_dilemma {
            self.update_dilemma(con);
        }
        if (cop_moved || robber_moved) && update_cop_advantage {
            self.update_cop_advantage(con.edges);
        }
        if cop_moved && update_plane_cop_strat {
            self.update_plane_cop_strat(con);
        }
    }

    fn change_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>, set: bool) {
        let bit = 1u8 << self.options.active_manual_marker;
        if !con.positions.is_empty() {
            let (vertex, dist) = con.find_closest_vertex(screen_pos);
            if dist <= 35.0 * con.scale {
                if set {
                    self.marked_manually[vertex] |= bit;
                } else if self.marked_manually[vertex] & bit != 0 {
                    self.marked_manually[vertex] -= bit;
                }
            }
        }
    }

    fn add_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, true);
    }

    fn remove_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, false);
    }

    fn paint_bucket_at(&mut self, edges: &EdgeList, screen_pos: Pos2, con: &DrawContext<'_>) {
        let bit = 1u8 << self.options.active_manual_marker;
        if con.positions.is_empty() {
            return;
        }
        let (v0, dist) = con.find_closest_vertex(screen_pos);
        if dist > 35.0 * con.scale {
            return;
        }
        // the action will eighter remove a connected component or fill it in,
        // depending on the current state on the vertex which is clicked.
        let erase = self.marked_manually[v0] & bit != 0;
        let mut queue = std::mem::take(&mut self.queue);
        queue.push_back(v0);
        while let Some(v) = queue.pop_front() {
            let is_colored = self.marked_manually[v] & bit != 0;
            if erase && is_colored {
                // we act as if we have eight independent canvasses wich just two colors each.
                self.marked_manually[v] -= bit;
                queue.extend(edges.neighbors_of(v));
            }
            if !erase && !is_colored {
                // we act as if we have eight independent canvasses wich just two colors each.
                self.marked_manually[v] |= bit;
                queue.extend(edges.neighbors_of(v));
            }
        }
        self.queue = queue;
    }

    fn screenshot_as_tikz(&mut self, con: &DrawContext<'_>) {
        if !self.take_screenshot || !NATIVE {
            return;
        }
        let name = &self.screenshot_name[..];
        let file_name = if name.is_empty() {
            use chrono::{Datelike, Local, Timelike};
            let now = Local::now();
            let date = now.date_naive();
            let date_str = format!("{}-{}-{}", date.year(), date.month(), date.day());
            let time = now.time();
            let time_str = format!("{}-{}-{}", time.hour(), time.minute(), time.second());
            std::path::PathBuf::from(format!("screenshots/{date_str}--{time_str}.tex"))
        } else {
            std::path::PathBuf::from(format!("screenshots/{name}.tex",))
        };
        let header = {
            let shape = con.map.shape().to_sting();
            let res = con.map.resolution();
            let color_info = self.options.vertex_color_info().name_str();
            let number_info = self.options.vertex_number_info().name_str();
            let cam_x = con.cam().position.x;
            let cam_y = con.cam().position.y;
            let angle = con.cam().z_angle();
            let zoom = con.cam().zoom;
            let active_police_vertices = {
                let vertices = self.characters.active_cop_vertices();
                let nr_seps = vertices.len().saturating_sub(1);
                let seps = std::iter::repeat(", ".to_string()).take(nr_seps);
                String::from_iter(vertices.iter().map(|&v| v.to_string()).interleave(seps))
            };
            let inactive_police_vertices = {
                let vertices = self
                    .characters
                    .all()
                    .iter()
                    .filter_map(|c| (!c.is_active()).then_some(c.vertex().to_string()))
                    .collect_vec();
                let nr_seps = vertices.len().saturating_sub(1);
                let seps = std::iter::repeat(", ".to_string()).take(nr_seps);
                String::from_iter(vertices.into_iter().interleave(seps))
            };
            let robber_vertex = self
                .characters
                .all()
                .first()
                .map_or("<Keiner>".to_string(), |r| r.vertex().to_string());
            format!(
                "\n\
                % Form: {shape}\n\
                % Auflösung: {res}\n\
                % gezeigte Infopunkte: {color_info}\n\
                % gezeige Infozahlen: {number_info}\n\
                % Kameraposition: ({cam_x}, {cam_y}) Winkel: {angle} Zoom: {zoom}\n\
                % Positionen aktiver Polizisten: [{active_police_vertices}]\n\
                % Positionen inaktiver Polizisten: [{inactive_police_vertices}]\n\
                % Räuberknoten: {robber_vertex}\n\
                \n\
                \n"
            )
        };
        super::tikz::draw_to_file(
            file_name,
            header,
            &con.painter,
            *con.screen(),
            character::emojis_as_latex_commands(),
        );
    }

    pub fn process_general_input(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        use egui::{Key, PointerButton};
        let tool = self.tool;
        let mut change = false;
        let held_key = ui.input(|info| {
            if info.key_pressed(Key::F3) {
                self.characters.show_allowed_next_steps ^= true;
            }
            if info.key_pressed(Key::F4) {
                self.characters.show_past_steps ^= true;
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Z) {
                self.characters.undo_move(con.edges, con.positions, &mut self.queue);
                change = true;
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Y) {
                self.characters.redo_move(con.edges, con.positions, &mut self.queue);
                change = true;
            }
            if info.modifiers.ctrl && info.key_pressed(Key::R) {
                self.characters
                    .continue_move_pattern(con.edges, con.positions, &mut self.queue);
                change = true;
            }

            let pointer_pos = info.pointer.latest_pos()?;
            if !con.response.contains_pointer() {
                return None;
            }

            if info.key_pressed(Key::F2) {
                let (v, dist) = con.find_closest_vertex(pointer_pos);
                if dist <= 25.0 * con.scale {
                    if !self.characters.remove_cop_at_vertex(v) {
                        self.characters.new_character_at(pointer_pos, con.map);
                    }
                    change = true;
                }
            }

            let mouse_down = info.pointer.button_down(PointerButton::Primary);
            if tool == MouseTool::Draw && mouse_down || info.key_pressed(Key::M) {
                self.add_marker_at(pointer_pos, con);
            }
            if tool == MouseTool::Erase && mouse_down || info.key_pressed(Key::N) {
                self.remove_marker_at(pointer_pos, con);
            }
            if tool == MouseTool::Paintbucket
                && info.pointer.button_released(PointerButton::Primary)
            {
                self.paint_bucket_at(con.edges, pointer_pos, con);
            }

            const NUMS: [Key; 8] = {
                use egui::Key::*;
                [Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8]
            };
            if info.key_down(Key::Q) {
                for (n, &key) in izip!(2.., &NUMS[..6]) {
                    if info.key_pressed(key) {
                        self.options.last_selected_vertex_color_infos[..n].rotate_right(1);
                        change = true;
                    }
                }
                return Some(Key::Q);
            }
            if info.key_down(Key::W) {
                for (n, &key) in izip!(2.., &NUMS[..6]) {
                    if info.key_pressed(key) {
                        self.options.last_selected_vertex_number_infos[..n].rotate_right(1);
                        change = true;
                    }
                }
                return Some(Key::W);
            }
            if info.key_down(Key::F) {
                for (n, key) in izip!(0.., NUMS) {
                    if info.key_pressed(key) {
                        self.options.active_manual_marker = n;
                    }
                }
                return Some(Key::F);
            }
            if info.key_down(Key::E) {
                for (new_tool, &key) in izip!(MouseTool::ALL, &NUMS[..4]) {
                    if info.key_pressed(key) {
                        if new_tool == tool && tool == MouseTool::Drag {
                            self.options.show_manual_marker_window ^= true;
                        }
                        self.change_tool_to(new_tool);
                    }
                }
                return Some(Key::E);
            }
            None
        });

        if let Some(key) = held_key {
            let id = egui::Id::new((&self.options as *const _, "tooltip-fast-switch"));
            egui::show_tooltip(ui.ctx(), ui.layer_id(), id, |ui| {
                let opts = &self.options;
                let add_unwrapped = |ui: &mut Ui, txt| {
                    ui.add(Label::new(txt).wrap_mode(egui::TextWrapMode::Extend));
                };
                match key {
                    Key::Q => {
                        for n in 0..7 {
                            let s = opts.last_selected_vertex_color_infos[n].name_str();
                            if n == 0 {
                                add_unwrapped(ui, s.to_string());
                                ui.separator();
                            } else {
                                add_unwrapped(ui, format!("{n}: {s}"));
                            }
                        }
                    },
                    Key::W => {
                        for n in 0..7 {
                            let s = opts.last_selected_vertex_number_infos[n].name_str();
                            if n == 0 {
                                add_unwrapped(ui, s.to_string());
                                ui.separator();
                            } else {
                                add_unwrapped(ui, format!("{n}: {s}"));
                            }
                        }
                    },
                    Key::F => {
                        const SIZE: Vec2 = Vec2::new(28.0, 14.0); //14.0 is default text size
                        for (n, &c) in izip!(1.., &opts.manual_marker_colors) {
                            ui.horizontal(|ui| {
                                add_unwrapped(ui, format!("{n}: "));
                                egui::color_picker::show_color(ui, c, SIZE);
                                if n - 1 == opts.active_manual_marker {
                                    add_unwrapped(ui, String::from("⬅"));
                                }
                            });
                        }
                    },
                    Key::E => {
                        ui.add(
                            Label::new(egui::RichText::new(tool.symbol()).size(30.0))
                                .wrap_mode(egui::TextWrapMode::Extend),
                        );
                    },
                    _ => unreachable!(),
                }
            });
        }
        if change {
            self.register_change_now();
        }
    }

    fn draw_convex_cop_hull(&self, con: &DrawContext<'_>) {
        if self.options.show_convex_hull {
            let color = self.options.hull_style.colors[0];
            let size = con.scale * 8.0 * self.options.hull_style.size;
            for (&in_hull, &pos, &vis) in
                izip!(self.cop_hull_data.hull(), con.positions, con.visible)
            {
                if vis && in_hull.contained() {
                    let draw_pos = con.cam().transform(pos);
                    let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                    con.painter.add(marker_circle);
                }
            }
        }
        if self.options.show_hull_boundary {
            let color = self.options.hull_boundary_style.colors[0];
            let size = con.scale * 4.0 * self.options.hull_boundary_style.size;
            for segment in self.cop_hull_data.safe_boundary_parts() {
                for &v in segment {
                    if con.visible[v] {
                        let draw_pos = con.vertex_draw_pos(v);
                        let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                        con.painter.add(marker_circle);
                    }
                }
            }
        }
    }

    fn draw_vertices(&self, con: &DrawContext<'_>) {
        if !self.options.draw_vertices {
            return;
        }
        let size = 4.0 * con.scale * self.options.vertex_style.size;
        let color = self.options.vertex_style.colors[0];
        for (&vis, &pos) in izip!(con.visible, con.positions) {
            if vis {
                let draw_pos = con.cam().transform(pos);
                let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                con.painter.add(marker_circle);
            }
        }
    }

    fn draw_green_circles(&mut self, ui: &Ui, con: &DrawContext<'_>) {
        let dark = ui.ctx().style().visuals.dark_mode;
        let colors = if dark {
            &color::DARK_MARKER_COLORS_F32
        } else {
            &color::BRIGHT_MARKER_COLORS_F32
        };

        self.currently_marked.clear();
        self.currently_marked.resize(con.edges.nr_vertices(), false);
        let utils_iter = izip!(&mut self.currently_marked, con.positions, con.visible);

        let size = 12.0 * self.options.automatic_marker_style.size * con.scale;
        let draw_circle_at = |pos, color| {
            let draw_pos = con.cam().transform(pos);
            let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
            con.painter.add(marker_circle);
        };
        macro_rules! draw_if {
            ($cond:expr, $util:expr, $f_color:expr) => {
                if $cond {
                    let (re, &pos, &vis) = $util;
                    *re = true;
                    if vis {
                        let color = $f_color;
                        draw_circle_at(pos, color());
                    }
                }
            };
            ($cond:expr, $util:expr) => {
                let c = self.options.automatic_marker_style.colors[0];
                draw_if!($cond, $util, || c);
            };
        }
        match self.options.vertex_color_info() {
            VertexColorInfo::NearNodes => {
                if let Some(r) = self.characters.active_robber() {
                    for (r_dist, c_dist, util) in izip!(r.dists(), &self.min_cop_dist, utils_iter) {
                        draw_if!(r_dist < c_dist, util);
                    }
                }
            },
            VertexColorInfo::Escape1 => {
                for (&adv, &hull, util) in
                    izip!(&self.cop_advantage, self.cop_hull_data.hull(), utils_iter)
                {
                    draw_if!(hull.contained() && adv < -1, util);
                }
            },
            VertexColorInfo::RobberDist => {
                if let Some(r) = self.characters.active_robber() {
                    let bnd = self.options.marked_robber_dist;
                    for (&dist, util) in izip!(r.dists(), utils_iter) {
                        draw_if!(dist == bnd, util);
                    }
                }
            },
            VertexColorInfo::MinCopDist => {
                for (&dist, util) in izip!(&self.min_cop_dist, utils_iter) {
                    draw_if!(dist == self.options.marked_cop_dist, util);
                }
            },
            VertexColorInfo::MaxCopDist => {
                for (&dist, util) in izip!(&self.max_cop_dist, utils_iter) {
                    draw_if!(dist == self.options.marked_cop_dist, util);
                }
            },
            VertexColorInfo::AnyCopDist => {
                let active_dists =
                    self.characters.active_cops().map(Character::dists).collect_vec();
                for (v, util) in izip!(0.., utils_iter) {
                    let mut any_picked = false;
                    let picked = active_dists.iter().map(|d| {
                        let res = d[v] == self.options.marked_cop_dist;
                        any_picked |= res;
                        res
                    });
                    //caution: don't evaluate color lazily, else `any_picked` is always false.
                    let color = color::blend_picked(colors, picked);
                    draw_if!(any_picked, util, || color);
                }
            },
            VertexColorInfo::Escape2 => {
                let escs = if self.escapable_grid.graph.represents_current_map {
                    &self.escapable_grid.esc_components[..]
                } else {
                    self.escapable.escapable()
                };
                for (&esc, util) in izip!(escs, utils_iter) {
                    let color = || color::u32_marker_color(esc, colors);
                    draw_if!(esc != 0, util, color);
                }
            },
            VertexColorInfo::Escape2Grid => {
                let sat = if dark { 750 } else { 350 };
                let colors = color::sample_color_wheel::<6>(sat, 4);
                let shown = self.options.shown_escape_directions;
                for (&esc, util) in izip!(&self.escapable_grid.esc_directions, utils_iter) {
                    let esc_shown = esc.0 & shown;
                    let color = || color::u8_marker_color(esc_shown, &colors);
                    draw_if!(esc_shown != 0, util, color);
                }
            },
            VertexColorInfo::EscapeConeGrid => {
                let sat = if dark { 750 } else { 350 };
                let colors = color::sample_color_wheel::<6>(sat, 4);
                let shown = self.options.shown_escape_directions;
                for (&esc_cone, &esc, &h, util) in izip!(
                    &self.escapable_grid.cone_esc_directions,
                    &self.escapable_grid.esc_directions,
                    self.cop_hull_data.hull(),
                    utils_iter
                ) {
                    let esc_shown = (if h.contained() { esc_cone.0 } else { esc.0 }) & shown;
                    let color = || color::u8_marker_color(esc_shown, &colors);
                    draw_if!(esc_shown != 0, util, color);
                }
            },
            VertexColorInfo::Escape3Grid => {
                for (&esc, util) in izip!(&self.dilemma.dilemma_regions, utils_iter) {
                    let color = || color::u32_marker_color(esc, colors);
                    draw_if!(esc != 0, util, color);
                }
            },
            VertexColorInfo::Escape23Grid => {
                for (&esc2, &esc3, util) in izip!(
                    &self.escapable_grid.esc_components,
                    &self.dilemma.dilemma_regions,
                    utils_iter
                ) {
                    let esc = esc2 | esc3;
                    let color = || color::blend_picked(colors, [esc2 != 0, esc3 != 0].into_iter());
                    draw_if!(esc != 0, util, color);
                }
            },
            VertexColorInfo::BruteForceRes => {
                let (mut active_cops, game_type) = self.characters.police_state(con);
                if let Some(bf::Outcome::RobberWins(data)) = &self.worker.result_for(&game_type) {
                    let safe_vertices = data.safe_vertices(&mut active_cops);
                    for (safe, util) in izip!(safe_vertices, utils_iter) {
                        draw_if!(safe, util);
                    }
                }
            },
            VertexColorInfo::VertexEquivalenceClasses => {
                if let SymGroup::Explicit(equiv) = con.sym_group() {
                    for (&repr, &pos, &vis) in
                        izip!(equiv.vertex_representatives(), con.positions, con.visible)
                    {
                        if vis {
                            //every (visible) vertex is marked, no need to record this.
                            let color = colors[repr % colors.len()].into();
                            draw_circle_at(pos, color);
                        }
                    }
                }
            },
            VertexColorInfo::RobberVertexClass => {
                if let (Some(r), SymGroup::Explicit(sym)) =
                    (self.characters.active_robber(), con.sym_group())
                {
                    let v0 = r.vertex();
                    for auto in sym.all_automorphisms() {
                        let sym_v = auto.apply_forward(v0);
                        self.currently_marked[sym_v] = true;
                        if con.visible[sym_v] {
                            let sym_pos = con.positions[sym_v];
                            draw_circle_at(sym_pos, self.options.automatic_marker_style.colors[0]);
                        }
                    }
                }
            },
            VertexColorInfo::CopsRotatedToEquivalence => {
                let (mut active_cops, game_type) = self.characters.police_state(con);
                if game_type.nr_cops > bf::MAX_COPS {
                    return;
                }
                let rotated = con.sym_group().to_representative(&mut active_cops);
                for &v in &rotated[..] {
                    self.currently_marked[v] = true;
                    let pos = con.positions[v];
                    if con.visible[v] {
                        draw_circle_at(pos, self.options.automatic_marker_style.colors[0]);
                    } else {
                        //only draw half size
                        let draw_pos = con.cam().transform(pos);
                        let marker_circle = egui::Shape::circle_filled(
                            draw_pos,
                            con.scale * 3.0,
                            self.options.automatic_marker_style.colors[0],
                        );
                        con.painter.add(marker_circle);
                    }
                }
            },
            VertexColorInfo::RobberCone => {
                if let Some(g) = graph::grid::GridGraph::try_from(con.map.data()) {
                    if let Some(r) = self.characters.active_robber() {
                        let robber_coords = g.coordinates_of(r.vertex());
                        let shown = graph::grid::Dirs(self.options.shown_escape_directions);
                        for (v, util) in izip!(0.., utils_iter) {
                            let v_coords = g.coordinates_of(v);
                            let dirs_to_v = (v_coords - robber_coords).dirs(g.norm);
                            draw_if!(shown.intersection(dirs_to_v) == dirs_to_v, util);
                        }
                    }
                }
            },
            VertexColorInfo::Debugging => {
                //for &v in self.escapable.inner_connecting_line() {
                //    if con.visible[v] {
                //        self.currently_marked[v] = true;
                //        let pos = con.positions[v];
                //        draw_circle_at(pos, self.options.automatic_marker_color);
                //    }
                //}
                //if let Some(g) = graph::grid::GridGraph::try_from(con.map.data()) {
                //    if let Some(r) = self.characters.active_robber() {
                //        let v = g.coordinates_of(r.vertex());
                //        for (dist, color) in izip!(0.., color::HAND_PICKED_MARKER_COLORS) {
                //            for nxy in g.dist_neighbors_of(v, dist) {
                //                let n = g.index_of(nxy);
                //                if con.visible[n] {
                //                    draw_circle_at(con.positions[n], color);
                //                }
                //            }
                //        }
                //    }
                //}
                //for (&esc, util) in izip!(&self.dilemma.overlap, utils_iter) {
                //    let color = || color::u32_marker_color(esc, colors);
                //    draw_if!(esc != 0, util, color);
                //}

                let mut bnd = crate::graph::CopsHull::new();
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
                let mut queue = VecDeque::new();
                bnd.update(
                    self.characters.cops(),
                    con.edges,
                    &mut queue,
                    vertices_outside_hull,
                    &self.min_cop_dist,
                );
                for (segment, &f32color) in izip!(bnd.safe_boundary_parts(), colors.iter().cycle())
                {
                    let color: Color32 = f32color.into();
                    for &v in segment {
                        if con.visible[v] {
                            let draw_pos = con.vertex_draw_pos(v);
                            let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                            con.painter.add(marker_circle);
                        }
                    }
                }
            },
            VertexColorInfo::SafeOutside => {
                for (&in_hull, &dist, util) in izip!(
                    self.cop_hull_data.hull(),
                    self.cop_hull_data.dist_to_hull(),
                    utils_iter
                ) {
                    draw_if!(in_hull.outside() && dist >= 2, util);
                }
            },
            VertexColorInfo::SafeBoundary => {
                for (&in_hull, &dist, util) in
                    izip!(self.cop_hull_data.hull(), &self.min_cop_dist, utils_iter)
                {
                    draw_if!(in_hull.on_boundary() && dist >= 2, util);
                }
            },
            VertexColorInfo::CopsVoronoi => {
                let active_dists =
                    self.characters.active_cops().map(Character::dists).collect_vec();

                let base_color = self.options.automatic_marker_style.colors[0];
                let color_3exact = base_color;
                let color_3about = base_color.gamma_multiply(0.85);
                let color_2exact = base_color.gamma_multiply(0.6);
                let color_2about = base_color.gamma_multiply(0.3);
                let choose_color = |w| match w {
                    w if w >= 300 => color_3exact,
                    w if w >= 201 => color_3about,
                    200 => color_2exact,
                    w if w >= 102 => color_3about,
                    _ => color_2about,
                };

                for (v, &min_dist, util) in izip!(0.., &self.min_cop_dist, utils_iter) {
                    // sometimes the change from region "owned" by one cop to region
                    // "owned" by the next happens between vertices.
                    // thus we also look for near misses.
                    let weight: usize = active_dists
                        .iter()
                        .map(|d| match d[v] {
                            n if n == min_dist => 100,
                            n if n == min_dist + 1 => 1,
                            _ => 0,
                        })
                        .sum();
                    draw_if!(weight > 100, util, || choose_color(weight));
                }
            },
            VertexColorInfo::CopStratPlaneDanger => {
                for (&dang, util) in izip!(self.plane_cop_strat.danger_zones(), utils_iter) {
                    let color = || color::u32_marker_color(dang, colors);
                    draw_if!(dang != 0, util, color);
                }
            },
            VertexColorInfo::SpecificVertex => {
                if let Some(&pos) = con.positions.get(self.options.specific_shown_vertex) {
                    draw_circle_at(pos, self.options.automatic_marker_style.colors[0]);
                }
            },
            VertexColorInfo::None => {},
        }
    }

    fn draw_numbers(&self, ui: &Ui, con: &DrawContext<'_>) {
        use graph::grid::Dirs;

        let font = egui::FontId::proportional(12.0 * self.options.number_style.size * con.scale);
        let color = if ui.ctx().style().visuals.dark_mode {
            color::WHITE
        } else {
            color::BLACK
        };
        let draw_text_at = |pos: Pos3, txt: String| {
            if !txt.is_empty() {
                use egui::text::LayoutJob;
                let mut layout_job = LayoutJob::simple_singleline(txt, font.clone(), color);
                layout_job.halign = egui::Align::Center;
                let galley = ui.fonts(|f| f.layout_job(layout_job));
                let screen_pos = con.cam().transform(pos);
                // shift pos upwards (negative y direction), so text is centered on vertex
                // why the heck is the best value not 0.5 btw?
                let above_pos = screen_pos - font.size * Vec2::new(0.0, 0.53);
                let text =
                    egui::Shape::Text(egui::epaint::TextShape::new(above_pos, galley, color));
                con.painter.add(text);
            }
        };
        macro_rules! draw {
            ($iter:expr, $is_shown:expr) => {
                for (val, &pos, &vis) in izip!($iter, con.positions, con.visible) {
                    if vis && $is_shown(&val) {
                        draw_text_at(pos, val.to_string());
                    }
                }
            };
            ($iter:expr) => {
                let show = |_: &_| true;
                draw!($iter, show);
            };
        }
        let draw_isize_slice = |numbers: &[isize]| {
            let show = |&&num: &&_| num != isize::MAX;
            draw!(numbers, show);
        };
        let draw_arrows = |directions: &[Dirs], mask: Dirs| {
            if let Some(g) = graph::grid::GridGraph::try_from(con.map.data()) {
                if g.side_len() < 3 {
                    return;
                }
                let stroke_dirs = {
                    let v0_xy = graph::grid::Coords { x: 1, y: 1 };
                    let v0 = g.unchecked_index_of(v0_xy);
                    let v0_pos = con.cam().transform(con.positions[v0]);
                    g.norm
                        .unit_directions()
                        .iter()
                        .map(|&dir| {
                            let neigh = g.unchecked_index_of(v0_xy + dir);
                            let neigh_pos = con.cam().transform(con.positions[neigh]);
                            (neigh_pos - v0_pos) * 0.35
                        })
                        .collect_vec()
                };
                let stroke_size = 1.85 * con.scale;
                let stroke = egui::Stroke::new(stroke_size, color);
                for (v, &val, &vis) in izip!(0.., directions, con.visible) {
                    let shown_val = val.intersection(mask);
                    if vis && shown_val.nonempty() {
                        let v_pos = con.cam().transform(con.positions[v]);
                        for (mask, &dir) in izip!(Dirs::unit_bits(g.norm), &stroke_dirs) {
                            if mask.intersection(shown_val).nonempty() {
                                super::add_arrow(&con.painter, v_pos, dir, stroke, 2.0);
                            }
                        }
                    }
                }
            }
        };

        match self.options.vertex_number_info() {
            VertexSymbolInfo::Indices => {
                draw!(0..);
            },
            VertexSymbolInfo::BruteforceCopMoves => {
                let (active_cops, game_type) = self.characters.police_state(con);
                if let Some(strat) = self.worker.strats_for(&game_type) {
                    let (autos, cop_positions) = strat.pack(&active_cops);
                    let nr_moves_left = strat.time_to_win.nr_moves_left(cop_positions);
                    let auto = autos[0];
                    let show = |&m: &_| m != bf::UTime::MAX;
                    draw!(auto.forward().map(|v| nr_moves_left[v]), show);
                }
            },
            VertexSymbolInfo::Debugging => {
                //draw!(self.escapable.owners());
                //draw_arrows(&self.dilemma.shadow_dirs, Dirs(u8::MAX));
                draw_arrows(&self.dilemma.overlap_dirs, Dirs(u8::MAX));
                //draw_arrows(&self.escapable_grid.strong_esc_directions, Dirs(u8::MAX));
                draw_isize_slice(&self.dilemma.taken_steps);
            },
            VertexSymbolInfo::Escape2 => {
                let escs = if self.escapable_grid.graph.represents_current_map {
                    &self.escapable_grid.esc_components[..]
                } else {
                    self.escapable.escapable()
                };
                draw!(escs.iter().map(|&x| -> String {
                    const NAMES: [char; 32] = [
                        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E',
                        'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
                        'U', 'V',
                    ];
                    izip!(NAMES, 0..)
                        .filter_map(|(name, i)| ((1u32 << i) & x != 0).then_some(name))
                        .collect()
                }));
            },
            VertexSymbolInfo::Escape2Grid => {
                let mask = self.options.shown_escape_directions;
                let directions = &self.escapable_grid.esc_directions;
                draw_arrows(directions, Dirs(mask));
            },
            VertexSymbolInfo::EscapeConeGrid => {
                let mask = self.options.shown_escape_directions;
                let combined = izip!(
                    &self.escapable_grid.cone_esc_directions,
                    &self.escapable_grid.esc_directions,
                    self.cop_hull_data.hull()
                )
                .map(|(&cone, &esc, &h)| if h.contained() { cone } else { esc })
                .collect_vec();
                draw_arrows(&combined, Dirs(mask));
            },
            VertexSymbolInfo::Escape3Grid => {
                let directions = &self.dilemma.dilemma_dirs;
                draw_arrows(directions, Dirs(u8::MAX));
            },
            VertexSymbolInfo::Escape23Grid => {
                let combined = izip!(
                    &self.dilemma.dilemma_dirs,
                    &self.escapable_grid.esc_directions
                )
                .map(|(&a, &b)| a.union(b))
                .collect_vec();
                draw_arrows(&combined, Dirs(u8::MAX));
            },
            VertexSymbolInfo::MaxCopDist => {
                draw_isize_slice(&self.max_cop_dist);
            },
            VertexSymbolInfo::MinCopDist => {
                draw_isize_slice(&self.min_cop_dist);
            },
            VertexSymbolInfo::RobberAdvantage => {
                draw_isize_slice(&self.cop_advantage);
            },
            VertexSymbolInfo::RobberDist => {
                if let Some(r) = self.characters.active_robber() {
                    draw_isize_slice(r.dists());
                }
            },
            VertexSymbolInfo::VertexEquivalenceClass => {
                if let SymGroup::Explicit(e) = con.sym_group() {
                    draw!(e.vertex_representatives());
                }
            },
            VertexSymbolInfo::None => {},
        }
    }

    fn draw_best_cop_moves(&self, con: &DrawContext<'_>) {
        if !self.options.show_cop_strat {
            return;
        }
        let Some(robber) = self.characters.active_robber() else {
            return;
        };
        let robber_v = robber.vertex();
        let (mut active_cops, game_type) = self.characters.police_state(con);
        let Some(strat) = self.worker.strats_for(&game_type) else {
            return;
        };

        let (_, curr_cop_positions) = strat.pack(&active_cops);
        let (curr_autos, curr_repr) = strat.symmetry.power_repr(&mut active_cops);
        let to_curr = |v| curr_autos[0].apply_forward(v);
        let from_curr = |v| curr_autos[0].apply_backward(v);

        debug_assert!(active_cops[..].iter().all(|&c| curr_repr.contains(&to_curr(c))));

        let robber_v_in_curr = to_curr(robber_v);
        let curr_nr_moves_left =
            strat.time_to_win.nr_moves_left(curr_cop_positions)[robber_v_in_curr];
        if matches!(curr_nr_moves_left, 0 | bf::UTime::MAX) {
            return;
        }

        for neigh_cops in strat.cop_moves.raw_lazy_cop_moves_from(con.edges, curr_repr) {
            let (neigh_transforms, neigh_index) = strat.pack(&neigh_cops);
            let transformed_robber_v = neigh_transforms[0].apply_forward(robber_v_in_curr);
            let nr_moves_left = strat.time_to_win.nr_moves_left(neigh_index);
            let neigh_chances = con
                .edges
                .neighbors_of(transformed_robber_v)
                .map(|v| nr_moves_left[v])
                .fold(nr_moves_left[transformed_robber_v], bf::UTime::max);

            if neigh_chances == curr_nr_moves_left - 1 {
                let i = izip!(&curr_repr[..], &neigh_cops[..]).position(|(a, b)| a != b).unwrap();
                let curr_v = from_curr(curr_repr[i]);
                let next_v = from_curr(neigh_cops[i]);
                debug_assert!(con.edges.has_edge(curr_v, next_v));
                debug_assert!(self.characters.active_cops().any(|c| c.vertex() == curr_v));
                if !con.visible[curr_v] || !con.visible[next_v] {
                    continue;
                }

                let curr_pos = con.vertex_draw_pos(curr_v);
                let next_pos = con.vertex_draw_pos(next_v);
                super::add_arrow(
                    &con.painter,
                    curr_pos,
                    next_pos - curr_pos,
                    egui::Stroke {
                        width: con.scale * 1.6,
                        color: Color32::from_rgb(150, 150, 255),
                    },
                    4.0,
                );
            }
        }
    }

    fn draw_manual_markers_combined(&self, con: &DrawContext<'_>) {
        let size = 4.5 * self.options.manual_marker_sizes[0] * con.scale;
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
                let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                con.painter.add(marker_circle);
            }
        }
    }

    fn draw_manual_markers_seperated(&self, con: &DrawContext<'_>) {
        let draw_sizes = {
            let mut res = self.options.manual_marker_sizes;
            for s in &mut res {
                *s *= 4.5 * con.scale;
            }
            res
        };
        let mask = self.options.shown_manual_markers;
        for (&vis, &marked, &pos) in izip!(con.visible, &self.marked_manually, con.positions) {
            let masked = marked & mask;
            if vis && masked != 0 {
                let draw_pos = con.cam().transform(pos);
                let mut single_mask: u8 = 1;
                for (&color, &size) in izip!(&self.options.manual_marker_colors, &draw_sizes) {
                    if masked & single_mask != 0 {
                        let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                        con.painter.add(marker_circle);
                    }
                    single_mask <<= 1;
                }
            }
        }
    }

    fn draw_manual_markers(&self, con: &DrawContext<'_>) {
        if self.options.combine_manual_marker_colors {
            self.draw_manual_markers_combined(con);
        } else {
            self.draw_manual_markers_seperated(con);
        }
    }

    fn choose_pointer_symbol(&mut self, ctx: &egui::Context, con: &DrawContext<'_>) {
        if con.response.contains_pointer() {
            let icon = match self.tool {
                MouseTool::Drag => egui::CursorIcon::Default,
                MouseTool::Draw => egui::CursorIcon::Crosshair,
                MouseTool::Erase => egui::CursorIcon::Crosshair,
                MouseTool::Paintbucket => egui::CursorIcon::Crosshair,
            };
            ctx.set_cursor_icon(icon);
        }
    }

    pub fn update_and_draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        self.choose_pointer_symbol(ui.ctx(), con);
        self.process_general_input(ui, con);
        if self.last_change_frame == self.frame_number {
            self.definitely_update(con);
        } else {
            self.maybe_update(con);
        }
        self.characters.robber_changed = false;
        self.characters.cop_changed = false;

        self.draw_vertices(con);
        self.draw_convex_cop_hull(con);
        self.draw_green_circles(ui, con);
        self.draw_manual_markers(con);
        let ch_style = &self.options.character_style;
        self.characters.draw_tails(ch_style, con);
        self.characters.draw_allowed_next_steps(ch_style, con);
        self.draw_best_cop_moves(con);
        self.draw_numbers(ui, con);

        let drag = self.tool == MouseTool::Drag;
        self.characters.update_and_draw(ui, ch_style, con, drag, &mut self.queue);

        self.screenshot_as_tikz(con);

        self.frame_number += 1;
    }
}
