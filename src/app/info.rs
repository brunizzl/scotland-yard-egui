use std::collections::VecDeque;

use itertools::{Itertools, izip};

use strum::IntoEnumIterator;

use egui::{Checkbox, Color32, ComboBox, Label, Ui, Vec2};

use crate::geo::Pos3;
use crate::graph::{
    self, Automorphism, EdgeList, Embedding3D, SymGroup, SymmetryGroup, bruteforce as bf,
};

use super::{
    DrawContext, NATIVE, add_disabled_drag_value, add_drag_value,
    bruteforce_state::BruteforceComputationState,
    character::{self, Character, CharactersStyle},
    color, load_or,
    manual_markers::{ManualMarkerOptions, ManualMarkers},
    map,
    style::Style,
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
            SafeOutside => "Winning Outside",
            SafeBoundary => "Winning Stage I",
            Escape1 => "Winning Stage II",
            Escape2 => "Winning Proximity",
            Escape2Grid => "Winning Direction (Gitter)",
            EscapeConeGrid => "Winning Cones (Gitter)",
            Escape3Grid => "Winning Dilemma (Gitter)",
            Escape23Grid => "Winning Cones + Dilemma (Gitter)",
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

    fn selectable_with(self, map: &map::Map) -> bool {
        use graph::Shape::*;
        let shape = map.shape();
        match self {
            VertexColorInfo::Escape2Grid | VertexColorInfo::EscapeConeGrid => {
                matches!(shape, TriangTorus | TriangGrid | SquareTorus | SquareGrid)
            },
            VertexColorInfo::Escape3Grid
            | VertexColorInfo::Escape23Grid
            | VertexColorInfo::RobberCone => shape == &TriangGrid,
            VertexColorInfo::RobberVertexClass | VertexColorInfo::VertexEquivalenceClasses => {
                matches!(map.data().sym_group(), SymGroup::Explicit(_))
            },
            VertexColorInfo::CopsRotatedToEquivalence => {
                !matches!(map.data().sym_group(), SymGroup::None(_))
            },
            // don't ever show this info (obsolete)
            VertexColorInfo::CopStratPlaneDanger => false,
            // show the remaining options all the time
            _ => true,
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
            Escape2 => {
                "jedes benachbarte Cop-Paar auf dem Hüllenrand hat einen Namen in { 0 .. 9, A .. }. \
            Der Marker listet alle Paare auf, zwischen denen der Räuber durchschlüpfen kann."
            },
            Escape2Grid => "alle Richtungen, in die der Räuber fliehen kann.",
            EscapeConeGrid => "jede Richtung, die in einem Fluchtkegel enthalten ist als Pfeil.",
            Escape23Grid => "Kombination aus Fluchtoption 2 + 3 auf Gitter.",
            Escape3Grid => "eine dieser Richtungen führt garantiert wieder auf einen Dilemmaknoten",
            MinCopDist => "punktweises Minimum aus den Abständen aller Cops",
            MaxCopDist => "punktweises Maximum aus den Abständen aller Cops",
            VertexEquivalenceClass => {
                "Für symmetrische Graphen werden Knoten, die mit einer symmetrierespektierenden \
            Rotation + Spiegelung auf einander abgebildet werden, in die selbe Klasse gesteckt. \
            Das macht Bruteforce etwas weniger speicherintensiv."
            },
            RobberDist => "Abstand von Räuberposition zu jedem Knoten",
            BruteforceCopMoves => {
                "Wenn Cops optimal ziehen, lebt Räuber noch maximal so viele Züge"
            },
            Debugging => {
                "Überraschungsinfo, die zum letzten Kompilierzeitpunkt \
            gerade spannend zum debuggen war"
            },
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

    fn selectable_with(self, map: &map::Map) -> bool {
        use graph::Shape::*;
        let shape = map.shape();
        match self {
            VertexSymbolInfo::Escape2Grid | VertexSymbolInfo::EscapeConeGrid => {
                matches!(shape, TriangTorus | TriangGrid | SquareTorus | SquareGrid)
            },
            VertexSymbolInfo::Escape3Grid | VertexSymbolInfo::Escape23Grid => shape == &TriangGrid,
            VertexSymbolInfo::VertexEquivalenceClass => {
                matches!(map.data().sym_group(), SymGroup::Explicit(_))
            },
            // show the remaining options all the time
            _ => true,
        }
    }
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
struct Options {
    vertex_style: Style<1>,
    automatic_marker_style: Style<1>,
    hull_style: Style<1>,
    hull_boundary_style: Style<1>,
    number_style: Style<0>,
    character_style: CharactersStyle,
    manual: ManualMarkerOptions,

    /// currently selected one at index 0
    last_selected_vertex_color_infos: [VertexColorInfo; 7],
    /// currently selected one at index 0
    last_selected_vertex_number_infos: [VertexSymbolInfo; 7],

    //these are only used, when the respective VertexColorInfo(s) is/are active
    /// determines cop dist marked in VertexColorInfo::{Max/Min/Any}CopDist
    marked_cop_dist: isize,
    /// determines max dist marked in [`VertexColorInfo::RobberDist`]
    marked_robber_dist: isize,
    #[serde(skip)]
    specific_shown_vertex: usize,
    /// the last two bits are always zero, therefore valid value for [`crate::graph::grid::Dirs`]
    shown_escape_directions: u8,
    /// escpected in `0..=32`. if 0, all bits are shown, else just the chosen one
    #[serde(skip)]
    shown_u32_bit: isize,

    show_convex_hull: bool,
    show_hull_boundary: bool,
    draw_vertices: bool,
    show_cop_strat: bool,
    show_manual_marker_window: bool,
}

const DEFAULT_OPTIONS: Options = Options {
    vertex_style: Style::new(&[Color32::GRAY]),
    automatic_marker_style: Style::new(&[color::GREEN]),
    hull_style: Style::new(&[color::LIGHT_BLUE]),
    hull_boundary_style: Style::new(&[color::WHITE]),
    number_style: Style::new(&[]),
    character_style: CharactersStyle::DEFAULT,
    manual: ManualMarkerOptions {
        sizes: [1.0; 8],
        colors: color::HAND_PICKED_MARKER_COLORS,
        shown: u8::MAX,
        layers: [0; 8],
    },

    marked_cop_dist: 10,
    marked_robber_dist: 10,
    specific_shown_vertex: 0,
    shown_escape_directions: 63,
    shown_u32_bit: 0,

    last_selected_vertex_color_infos: [VertexColorInfo::None; 7],
    last_selected_vertex_number_infos: [VertexSymbolInfo::None; 7],

    show_convex_hull: false,
    show_hull_boundary: false,
    draw_vertices: false,
    show_cop_strat: false,
    show_manual_marker_window: false,
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
        enum ShownValue {
            None,
            MarkedCopDist,
            MarkedRobberDist,
            VertexIndex,
            DirectionBits,
            U32Bits,
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
                VertexColorInfo::Escape3Grid | VertexColorInfo::Escape2 => ShownValue::U32Bits,
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
                add_drag_value(ui, &mut self.marked_cop_dist, "Abstand", 0..=1000, 1);
            },
            ShownValue::MarkedRobberDist => {
                add_drag_value(ui, &mut self.marked_robber_dist, "Abstand", 0..=1000, 1);
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
                    if ui.button(" << ").clicked() {
                        shown = shown.rotate_left_hex();
                    }
                    ui.add(egui::DragValue::new(&mut shown.0).range(0..=63).binary(6, true));
                    if ui.button(" >> ").clicked() {
                        shown = shown.rotate_right_hex();
                    }
                    ui.label("Richtungen").on_hover_text(
                        "Richtungen werden als Bitset gespeichert. \
                        Bits für Richtungen e₃ und -e₃ werden auf Vierecksgitter ignoriert.",
                    );
                });
                self.shown_escape_directions = shown.0;
            },
            ShownValue::U32Bits => {
                ui.horizontal(|ui| {
                    if ui.button(" - ").clicked() {
                        self.shown_u32_bit = isize::max(self.shown_u32_bit - 1, 0);
                    }
                    ui.add(
                        egui::DragValue::new(&mut self.shown_u32_bit)
                            .range(0..=32)
                            .custom_formatter(|x, _| {
                                if x == 0.0 {
                                    "Alle".to_string()
                                } else {
                                    x.to_string()
                                }
                            }),
                    );
                    if ui.button(" + ").clicked() {
                        self.shown_u32_bit = isize::min(self.shown_u32_bit + 1, 32);
                    }
                    ui.label("Komponente");
                });
            },
            ShownValue::None => {
                add_disabled_drag_value(ui);
            },
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map) -> bool {
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
            ComboBox::from_id_salt(&self.last_selected_vertex_color_infos as *const _)
                .selected_text(self.vertex_color_info().name_str())
                .show_ui(ui, |ui| {
                    super::style::close_options_menu();

                    let mut curr = self.vertex_color_info();
                    for val in VertexColorInfo::iter() {
                        let mut text = egui::RichText::new(val.name_str());
                        if !val.selectable_with(map) {
                            text = text.weak();
                        }
                        ui.radio_value(&mut curr, val, text)
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
            ComboBox::from_id_salt(&self.last_selected_vertex_number_infos as *const _)
                .selected_text(self.vertex_number_info().name_str())
                .show_ui(ui, |ui| {
                    super::style::close_options_menu();

                    let mut curr = self.vertex_number_info();
                    for val in VertexSymbolInfo::iter() {
                        let mut text = egui::RichText::new(val.name_str());
                        if !val.selectable_with(map) {
                            text = text.weak();
                        }
                        ui.radio_value(&mut curr, val, text)
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
    pub fn symbol(self) -> &'static str {
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
    pub manual_markers: ManualMarkers,

    /// this is only used as intermediary variable during computations.
    /// to not reallocate between frames/ algorithms, the storage is kept and passed to where needed.
    pub queue: VecDeque<usize>,

    pub characters: character::State,

    options: Options,
    frame_number: usize,
    last_change_frame: usize,

    worker: BruteforceComputationState,

    take_tikz_screenshot: bool,
    take_cetz_screenshot: bool,
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
            manual_markers: ManualMarkers::new(),

            queue: VecDeque::new(),

            characters: character::State::new(),
            options: DEFAULT_OPTIONS,
            last_change_frame: 0,
            frame_number: 0,

            worker: BruteforceComputationState::new(),

            take_tikz_screenshot: false,
            take_cetz_screenshot: false,
            screenshot_name: String::new(),
        }
    }
}

impl Info {
    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, OPTIONS, &self.options);
        eframe::set_value(storage, CHARACTERS, &self.characters);
        let rle_manually = crate::rle::encode(self.manual_markers.curr());
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
            manual_markers: ManualMarkers::new_init(marked_manually),

            queue: VecDeque::new(),

            characters,
            options,
            last_change_frame: 0,
            frame_number: 0,

            worker: BruteforceComputationState::new(),

            take_tikz_screenshot: false,
            take_cetz_screenshot: false,
            screenshot_name: String::new(),
        }
    }

    pub fn register_change_now(&mut self) {
        self.last_change_frame = self.frame_number;
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, map: &map::Map) {
        let mut change = self.options.draw_menu(ui, map);

        //everything going on here happens on a nother thread -> no need to recompute our data
        //-> no need to log wether something changed
        let nr_cops = self.characters.active_cops().count();
        self.worker.draw_menu(nr_cops, self.characters.rules(), ui, map);
        if NATIVE {
            ui.collapsing("📷 Screenshots", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Name: ");
                    ui.text_edit_singleline(&mut self.screenshot_name);
                });
                ui.horizontal(|ui| {
                    ui.label("Aufnehmen: ");
                    self.take_tikz_screenshot = ui.button("TikZ").clicked();
                    self.take_cetz_screenshot = ui.button("CeTZ").clicked();
                });
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
        let automatic_shown = self.options.vertex_color_info() != VertexColorInfo::None;
        let hull_shown = self.options.show_convex_hull;
        let mut new_tool = self.tool;
        egui::Window::new("Manuelle Marker")
            .open(&mut self.options.show_manual_marker_window)
            .constrain_to(ctx.content_rect())
            .show(ctx, |ui| {
                new_tool = new_tool.draw_mouse_tool_controls(ui);
                self.manual_markers.draw_selection_window_contents(
                    ui,
                    &mut self.options.manual,
                    automatic_shown,
                    hull_shown,
                    &self.currently_marked,
                    self.cop_hull_data.hull(),
                );
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
    ///
    /// f(v) := if not v in cop hull interior {
    ///     -min dist(v, cop) over all cops
    /// } else {
    ///     (min f(u) over all u neighbors of v) + 1
    /// }
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
                    if dist > best.1 { (v, dist) } else { best }
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
        self.dilemma.update(
            con.edges,
            self.cop_hull_data.hull(),
            &self.escapable_grid,
            &mut self.queue,
            self.characters.active_cops(),
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
        if self.manual_markers.curr().len() != nr_vertices {
            self.characters.forget_move_history();
            self.manual_markers.update_len(nr_vertices);
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
            || update_dilemma
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
        if cop_moved && update_cop_advantage {
            self.update_cop_advantage(con.edges);
        }
        if cop_moved && update_plane_cop_strat {
            self.update_plane_cop_strat(con);
        }
    }

    fn take_screenshot(&mut self, con: &DrawContext<'_>, text_shift: Vec2) {
        if !NATIVE || (!self.take_tikz_screenshot && !self.take_cetz_screenshot) {
            return;
        }
        let is_tikz = self.take_tikz_screenshot;
        let file_name = {
            let name = &self.screenshot_name[..];
            let file_end = if is_tikz { "tex" } else { "typ" };
            if name.is_empty() {
                use chrono::{Datelike, Local, Timelike};
                let now = Local::now();
                let date = now.date_naive();
                let date_str = format!("{}-{}-{}", date.year(), date.month(), date.day());
                let time = now.time();
                let time_str = format!("{}-{}-{}", time.hour(), time.minute(), time.second());
                std::path::PathBuf::from(format!("screenshots/{date_str}--{time_str}.{file_end}"))
            } else {
                std::path::PathBuf::from(format!("screenshots/{name}.{file_end}",))
            }
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
                let seps = std::iter::repeat_n(", ".to_string(), nr_seps);
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
                let seps = std::iter::repeat_n(", ".to_string(), nr_seps);
                String::from_iter(vertices.into_iter().interleave(seps))
            };
            let robber_vertex = self
                .characters
                .all()
                .first()
                .map_or("<Keiner>".to_string(), |r| r.vertex().to_string());
            let comment = if is_tikz { "% " } else { "//" };
            format!(
                "\n\
                {comment} Form: {shape}\n\
                {comment} Auflösung: {res}\n\
                {comment} gezeigte Infopunkte: {color_info}\n\
                {comment} gezeige Infozahlen: {number_info}\n\
                {comment} Kameraposition: ({cam_x}, {cam_y}) Winkel: {angle} Zoom: {zoom}\n\
                {comment} Positionen aktiver Polizisten: [{active_police_vertices}]\n\
                {comment} Positionen inaktiver Polizisten: [{inactive_police_vertices}]\n\
                {comment} Räuberknoten: {robber_vertex}\n\
                \n\
                \n"
            )
        };
        if is_tikz {
            super::tikz::draw_to_file(
                file_name,
                header,
                &con.painter,
                *con.screen(),
                text_shift,
                character::emojis_as_latex_commands(),
            );
        } else {
            super::cetz::draw_to_file(file_name, header, &con.painter, *con.screen(), text_shift);
        }
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
            let drag_active = tool == MouseTool::Drag;
            if info.modifiers.ctrl && info.key_pressed(Key::Z) {
                if drag_active {
                    self.characters.undo_move(con.edges, con.positions, &mut self.queue);
                    change = true;
                } else {
                    self.manual_markers.undo();
                }
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Y) {
                if drag_active {
                    self.characters.redo_move(con.edges, con.positions, &mut self.queue);
                    change = true;
                } else {
                    self.manual_markers.redo();
                }
            }
            if info.modifiers.ctrl && info.key_pressed(Key::R) && drag_active {
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
                self.manual_markers.add_at(pointer_pos, con);
            }
            if tool == MouseTool::Erase && mouse_down || info.key_pressed(Key::N) {
                self.manual_markers.remove_at(pointer_pos, con);
            }
            if tool == MouseTool::Paintbucket
                && info.pointer.button_released(PointerButton::Primary)
            {
                self.manual_markers.paint_bucket_at(pointer_pos, con, &mut self.queue);
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
                        self.manual_markers.active_bit = n;
                    }
                }
                return Some(Key::F);
            }
            if info.key_down(Key::E) {
                for (new_tool, &key) in izip!(MouseTool::ALL, &NUMS[..4]) {
                    if info.key_pressed(key) {
                        if new_tool == tool && drag_active {
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
            let anchor = if ui.input(|info| info.pointer.has_pointer()) {
                egui::PopupAnchor::Pointer
            } else {
                egui::PopupAnchor::Position(ui.ctx().content_rect().center())
            };
            egui::Tooltip::always_open(ui.ctx().clone(), ui.layer_id(), id, anchor).show(|ui| {
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
                        for (n, &c) in izip!(1.., &opts.manual.colors) {
                            ui.horizontal(|ui| {
                                add_unwrapped(ui, format!("{n}: "));
                                egui::color_picker::show_color(ui, c, SIZE);
                                if n - 1 == self.manual_markers.active_bit {
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
        let u32_mask = if self.options.shown_u32_bit > 0 {
            1u32 << (self.options.shown_u32_bit - 1)
        } else {
            u32::MAX
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
                    let masked_esc = esc & u32_mask;
                    let color = || color::u32_marker_color(masked_esc, colors);
                    draw_if!(masked_esc != 0, util, color);
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
                    let masked_esc = esc & u32_mask;
                    let color = || color::u32_marker_color(masked_esc, colors);
                    draw_if!(masked_esc != 0, util, color);
                }
            },
            VertexColorInfo::Escape23Grid => {
                let sat = if dark { 750 } else { 350 };
                let colors = color::sample_color_wheel::<6>(sat, 4);
                let shown = self.options.shown_escape_directions;
                for (&esc, util) in izip!(&self.dilemma.all_dirs, utils_iter) {
                    let esc_shown = esc.0 & shown;
                    let color = || color::u8_marker_color(esc_shown, &colors);
                    draw_if!(esc_shown != 0, util, color);
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
                if let Some(g) = graph::grid::GridGraph::try_from(con.map.data())
                    && let Some(r) = self.characters.active_robber()
                {
                    let robber_coords = g.coordinates_of(r.vertex());
                    let shown = graph::grid::Dirs(self.options.shown_escape_directions);
                    for (v, util) in izip!(0.., utils_iter) {
                        let v_coords = g.coordinates_of(v);
                        let dirs_to_v = (v_coords - robber_coords).dirs(g.norm);
                        draw_if!(shown.intersection(dirs_to_v) == dirs_to_v, util);
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
                for (&esc, util) in izip!(&self.dilemma.overlap, utils_iter) {
                    let color = || color::u32_marker_color(esc, colors);
                    draw_if!(esc != 0, util, color);
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

    /// returns how the drawn text is shifted relatve to the vertex, in screen coordinates
    fn draw_numbers(&self, ui: &Ui, con: &DrawContext<'_>) -> Vec2 {
        use graph::grid::Dirs;

        let font = egui::FontId::proportional(12.0 * self.options.number_style.size * con.scale);
        // shift pos upwards (negative y direction), so text is centered on vertex
        // why the heck is the best value not 0.5 btw?
        let text_shift = font.size * Vec2::new(0.0, -0.53);
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
                let galley = ui.fonts_mut(|f| f.layout_job(layout_job));
                let screen_pos = con.cam().transform(pos);
                let above_pos = screen_pos + text_shift;
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
                //draw_arrows(&self.escapable_grid.strong_esc_directions, Dirs(u8::MAX));
                let show = |&&num: &&_| num != isize::MIN;
                draw!(&self.dilemma.energy, show);
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
                let mask = self.options.shown_escape_directions;
                let directions = &self.dilemma.dilemma_dirs;
                draw_arrows(directions, Dirs(mask));
            },
            VertexSymbolInfo::Escape23Grid => {
                let mask = self.options.shown_escape_directions;
                let combined = izip!(
                    &self.dilemma.dilemma_dirs,
                    &self.escapable_grid.esc_directions
                )
                .map(|(&a, &b)| a.union(b))
                .collect_vec();
                draw_arrows(&combined, Dirs(mask));
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

        text_shift
    }

    fn draw_best_cop_moves(&self, con: &DrawContext<'_>) {
        if !self.options.show_cop_strat {
            return;
        }
        let Some(robber_v) = self.characters.active_robber().map(Character::vertex) else {
            return;
        };
        let (cops_now, game_type) = self.characters.police_state(con);
        // rs prefix / postfix is short for "round start",
        // which is the point in time just after the robber made his (currently) last move.
        // this is the starting point from which the currently progressing cop move is computed.
        let Some(cops_rs) = self.characters.police_state_round_start() else {
            return;
        };
        let Some(strat) = self.worker.strats_for(&game_type) else {
            return;
        };

        let curr_nr_moves_left = strat.times_for(&mut cops_rs.clone()).nth(robber_v).unwrap();
        if matches!(curr_nr_moves_left, 0 | bf::UTime::MAX) {
            return;
        }

        // perhaps TODO if performance is bad:
        // use current partial move as starting state to dismiss moves starting from cops_rs
        // that do not result in all cops which already moved in cops_now to occupy their current positions.
        let neigh_cop_states = game_type.rules.raw_cop_moves_from(con.edges, cops_rs);

        let mut neigh_times = Vec::new();
        for mut neigh_cops in neigh_cop_states {
            // filter for moves that respect the current partially advanced cop state
            if !izip!(&*cops_rs, &*neigh_cops, &*cops_now)
                .all(|(rs, neigh, now)| now == rs || now == neigh)
            {
                continue;
            }

            neigh_times.clear();
            neigh_times.extend(strat.times_for(&mut neigh_cops));
            let best_robber_response = con
                .edges
                .neighbors_of(robber_v)
                .map(|v| neigh_times[v])
                .fold(neigh_times[robber_v], bf::UTime::max);

            // filter for moves that bring the cops closer to winning
            if best_robber_response != curr_nr_moves_left - 1 {
                debug_assert!(best_robber_response >= curr_nr_moves_left);
                continue;
            }

            let arrow_stroke = {
                // add some randomness to arrow color to distinguish different movement options.
                let hash = neigh_cops.iter().fold(0, |a, b| a ^ b);
                let delta = ((hash | (hash >> 6) | (hash >> 12)) as u8) & 0b111111;
                let color = Color32::from_rgba_unmultiplied(100 + delta, 100 - delta, 255, 200);
                let width = con.scale * 2.5;
                egui::Stroke { width, color }
            };
            for (&rs_v, &next_v) in izip!(&*cops_rs, &*neigh_cops) {
                if rs_v == next_v || !con.visible[rs_v] || !con.visible[next_v] {
                    continue;
                }
                let arrow_start = con.vertex_draw_pos(rs_v);
                let arrow_dir = con.vertex_draw_pos(next_v) - arrow_start;
                super::add_arrow(&con.painter, arrow_start, arrow_dir, arrow_stroke, 3.0);
            }
        }
    }

    fn choose_pointer_symbol(&self, ctx: &egui::Context, con: &DrawContext<'_>) {
        if con.response.contains_pointer() {
            let icon = match self.tool {
                MouseTool::Drag => egui::CursorIcon::Default,
                MouseTool::Draw | MouseTool::Erase | MouseTool::Paintbucket => {
                    egui::CursorIcon::Crosshair
                },
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
        self.manual_markers.draw(con, &self.options.manual);
        let ch_style = &self.options.character_style;
        self.characters.draw_tails(ch_style, con);
        self.characters.draw_allowed_next_steps(ch_style, con);
        self.draw_best_cop_moves(con);
        let text_shift = self.draw_numbers(ui, con);

        let drag = self.tool == MouseTool::Drag;
        self.characters.update_and_draw(ui, ch_style, con, drag, &mut self.queue);

        self.take_screenshot(con, text_shift);

        self.frame_number += 1;
    }
}
