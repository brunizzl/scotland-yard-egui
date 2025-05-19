use std::{fs, io::Write, path::PathBuf, thread, time::Instant};

use itertools::izip;
use serde::{Deserialize, Serialize};

use super::*;
use crate::graph::{bruteforce as bf, Automorphism, ExplicitClasses, SymmetryGroup};
use crate::graph::{Embedding3D, NoSymmetry};

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct GameType {
    pub nr_cops: usize,
    pub resolution: usize,
    pub shape: crate::graph::Shape,
}

impl GameType {
    fn file_name(&self, folder: &str) -> PathBuf {
        PathBuf::from(format!(
            "{}/{}-{}-{}.msgpack",
            folder,
            self.nr_cops,
            self.shape.to_sting(),
            self.resolution
        ))
    }

    fn as_tuple_string(&self) -> String {
        format!(
            "({}, {}, {})",
            self.nr_cops,
            self.shape.to_sting(),
            self.resolution
        )
    }

    fn create_edges(&self) -> EdgeList {
        let embedding = Embedding3D::new_map_from(self.shape, self.resolution);
        embedding.into_edges()
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum WorkTask {
    Compute,
    ComputeStrat,
    Verify,
    Load,
    LoadStrat,
    Store,
    StoreStrat,
}

/// sometimes, e.g. when storing or when verifying, we both have a result and could encounter an error.
/// thus: this is not eighter / or, but can be both.
struct WorkResult {
    robber_strat: Option<GameOutcome>,
    police_strat: Option<bf::CopStrategy>,
    error: Option<String>,
}

impl WorkResult {
    fn new_err<E: ToString>(err: E) -> Self {
        Self {
            robber_strat: None,
            police_strat: None,
            error: Some(err.to_string()),
        }
    }

    fn new_res_err<E: ToString>(outcome: GameOutcome, err: E) -> Self {
        Self {
            robber_strat: Some(outcome),
            police_strat: None,
            error: Some(err.to_string()),
        }
    }

    fn new_strat_err<E: ToString>(strat: bf::CopStrategy, err: E) -> Self {
        Self {
            robber_strat: None,
            police_strat: Some(strat),
            error: Some(err.to_string()),
        }
    }

    fn new_strat(strat: bf::CopStrategy) -> Self {
        Self {
            robber_strat: None,
            police_strat: Some(strat),
            error: None,
        }
    }

    fn new_res(outcome: GameOutcome) -> Self {
        Self {
            robber_strat: Some(outcome),
            police_strat: None,
            error: None,
        }
    }
}

impl From<(Result<bf::Outcome, String>, Confidence)> for WorkResult {
    fn from((res, confidence): (Result<bf::Outcome, String>, Confidence)) -> Self {
        let (result, error) = match res {
            Ok(outcome) => (Some(GameOutcome { confidence, outcome }), None),
            Err(err) => (None, Some(err)),
        };
        Self {
            robber_strat: result,
            police_strat: None,
            error,
        }
    }
}

impl From<Result<bf::CopStrategy, String>> for WorkResult {
    fn from(res: Result<bf::CopStrategy, String>) -> Self {
        let (cop_strat, error) = match res {
            Ok(strat) => (Some(strat), None),
            Err(err) => (None, Some(err)),
        };
        Self {
            robber_strat: None,
            police_strat: cop_strat,
            error,
        }
    }
}

use bf::thread_manager::Command;
use bf::thread_manager::ExternManager as Manager;
struct Worker {
    game_type: GameType,
    task: WorkTask,
    handle: thread::JoinHandle<WorkResult>,

    manager: Option<Manager>,
}

/// the bruteforce algorithm is quite large to ensure correctness from just looking at the results
/// and/or looking at the code.
/// to improve confidence in the correctness, one can compute the same result with and without using symmetry.
#[derive(Serialize, Deserialize, PartialEq, Eq)]
pub enum Confidence {
    SymmetryOnly,
    NoSymmetry,
    Both,
    Err(String),
}

impl Confidence {
    fn new_err(msg: &str) -> Self {
        Self::Err(msg.to_string())
    }
}

#[derive(Serialize, Deserialize)]
struct GameOutcome {
    confidence: Confidence,
    outcome: bf::Outcome,
}

use std::collections::BTreeMap;
pub struct BruteforceComputationState {
    workers: Vec<Worker>,
    abort_worker: Option<(GameType, WorkTask, Instant)>,
    curr_game_type: GameType,
    robber_strat_stored: bool,
    cops_strat_stored: bool,
    robber_strats: BTreeMap<GameType, GameOutcome>,
    cop_strats: BTreeMap<GameType, bf::CopStrategy>,
    errors: Vec<(GameType, String)>,
}

impl BruteforceComputationState {
    pub fn new() -> Self {
        Self {
            workers: Vec::new(),
            abort_worker: None,
            curr_game_type: GameType {
                nr_cops: usize::MAX,
                resolution: 0,
                shape: crate::graph::Shape::Icosahedron,
            },
            robber_strat_stored: false,
            cops_strat_stored: false,
            robber_strats: BTreeMap::new(),
            cop_strats: BTreeMap::new(),
            errors: Vec::new(),
        }
    }

    pub fn result_for(&self, game_type: &GameType) -> Option<&bf::Outcome> {
        self.robber_strats.get(game_type).map(|o| &o.outcome)
    }

    pub fn strats_for(&self, game_type: &GameType) -> Option<&bf::CopStrategy> {
        self.cop_strats.get(game_type)
    }

    fn process_result(&mut self, game_type: GameType, res: WorkResult) {
        if let Some(err) = res.error {
            self.errors.push((game_type, err));
        }

        if let Some(strat) = res.robber_strat {
            self.robber_strats.insert(game_type, strat);
        }
        if let Some(strat) = res.police_strat {
            self.cop_strats.insert(game_type, strat);
        }
    }

    fn employ_worker<F>(
        &mut self,
        game_type: GameType,
        work: F,
        reciever: Option<Manager>,
        task: WorkTask,
    ) where
        F: FnOnce() -> WorkResult + Send + 'static,
    {
        if NATIVE {
            //use threads natively
            self.workers.push(Worker {
                game_type,
                task,
                handle: thread::spawn(work),
                manager: reciever,
            });
        } else {
            //wasm doesn't like threads -> just block gui
            self.process_result(game_type, work());
        }
    }

    fn check_on_workers(&mut self) {
        let mut done = None;
        for (i, worker) in izip!(0.., &mut self.workers) {
            if worker.handle.is_finished() {
                done = Some(i);
            }
            if let Some(r) = worker.manager.as_mut() {
                r.update();
            }
        }
        if let Some(i) = done {
            let worker = self.workers.remove(i);
            match worker.handle.join() {
                Ok(ok) => {
                    self.process_result(worker.game_type, ok);
                },
                Err(_) => {
                    let msg = "Thread hatte Panik :(".into();
                    self.errors.push((worker.game_type, msg));
                },
            }
        }
    }

    fn start_computation(&mut self, nr_cops: usize, map: &map::Map) {
        let game_type = GameType {
            nr_cops,
            resolution: map.resolution(),
            shape: map.shape(),
        };
        let edges = map.edges().clone();
        let (here, mut there) = bf::thread_manager::build_managers();
        let here = Some(here);
        macro_rules! employ {
            ($sym: expr, $transform_sym: expr) => {
                let cloned_sym = $sym.clone();
                let work = move || {
                    let sym = $transform_sym(cloned_sym);
                    let res = bf::compute_safe_robber_positions(nr_cops, edges, sym, &mut there);
                    (res, Confidence::SymmetryOnly).into()
                };
                self.employ_worker(game_type, work, here, WorkTask::Compute);
            };
        }
        match map.data().sym_group() {
            SymGroup::Explicit(equiv) => {
                employ!(equiv, |x| x);
            },
            SymGroup::Torus6(torus) => {
                employ!(torus, |t| ExplicitClasses::from(&t));
            },
            SymGroup::Torus4(torus) => {
                employ!(torus, |t| ExplicitClasses::from(&t));
            },
            SymGroup::None(none) => {
                employ!(*none, |x| x);
            },
        }
    }

    fn start_strat_computation(&mut self, nr_cops: usize, map: &map::Map) {
        let game_type = GameType {
            nr_cops,
            resolution: map.resolution(),
            shape: map.shape(),
        };
        let edges = map.edges().clone();
        let (here, there) = bf::thread_manager::build_managers();
        let here = Some(here);
        match map.data().sym_group() {
            SymGroup::Explicit(equiv) => {
                let sym = equiv.clone();
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, &there).into();
                self.employ_worker(game_type, work, here, WorkTask::ComputeStrat);
            },
            SymGroup::Torus6(torus) => {
                let sym = ExplicitClasses::from(torus);
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, &there).into();
                self.employ_worker(game_type, work, here, WorkTask::ComputeStrat);
            },
            SymGroup::Torus4(torus) => {
                let sym = ExplicitClasses::from(torus);
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, &there).into();
                self.employ_worker(game_type, work, here, WorkTask::ComputeStrat);
            },
            SymGroup::None(none) => {
                let sym = *none;
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, &there).into();
                self.employ_worker(game_type, work, here, WorkTask::ComputeStrat);
            },
        }
    }

    fn verify_robber_win_eq(sym: &bf::RobberWinData, no_sym: &bf::RobberWinData) -> Confidence {
        debug_assert_eq!(sym.cop_moves.nr_cops(), no_sym.cop_moves.nr_cops());
        debug_assert_eq!(
            sym.cop_moves.nr_map_vertices(),
            no_sym.cop_moves.nr_map_vertices()
        );
        for cops_index in no_sym.cop_moves.all_positions() {
            let mut cops = no_sym.cop_moves.eager_unpack(cops_index);
            let safe_with_sym = sym.safe_vertices(&mut cops);
            let safe_without_sym = no_sym.safe_vertices(&mut cops);
            for (v, s1, s2) in izip!(0.., safe_with_sym, safe_without_sym) {
                if s1 != s2 {
                    let (autos, repr) = sym.symmetry.power_repr(&mut cops);
                    let v_rot = autos[0].apply_forward(v);
                    return Confidence::Err(format!(
                        "Fehler: Cops {cops:?} repräsentiert von {repr:?} \
                        uneinig in Knoten {v} (rotiert = {v_rot})."
                    ));
                }
            }
        }
        Confidence::Both
    }

    fn verify_result(&mut self, game_type: GameType, mut outcome: GameOutcome) {
        let (here, there) = bf::thread_manager::build_managers();
        let here = Some(here);
        let work = move || {
            let edges = game_type.create_edges();
            let nr_map_vertices = edges.nr_vertices();
            let sym = NoSymmetry::new(nr_map_vertices);
            let res_without =
                bf::compute_safe_robber_positions(game_type.nr_cops, edges, sym, &there);
            let outcome_without = match res_without {
                Ok(ok) => ok,
                Err(err) => return WorkResult::new_res_err(outcome, err),
            };
            there.update("vergleiche Ergebnisse").ok();
            use bf::Outcome::*;
            outcome.confidence = match (&outcome.outcome, &outcome_without) {
                (CopsWin, CopsWin) => Confidence::Both,
                (CopsWin, RobberWins(_)) => Confidence::new_err("Ohne sym. gewinnt Räuber"),
                (RobberWins(_), CopsWin) => Confidence::new_err("Ohne sym. gewinnen Cops"),
                (RobberWins(sym), RobberWins(no_sym)) => Self::verify_robber_win_eq(sym, no_sym),
            };
            WorkResult::new_res(outcome)
        };
        self.employ_worker(game_type, work, here, WorkTask::Verify);
    }

    fn save_result(&mut self, game_type: GameType, outcome: GameOutcome) {
        let work = move || {
            let path = game_type.file_name("bruteforce");
            let file = match fs::File::create(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_res_err(outcome, err),
            };
            let mut file_buffer = std::io::BufWriter::new(file);
            if let Err(err) = rmp_serde::encode::write(&mut file_buffer, &outcome) {
                return WorkResult::new_res_err(outcome, err);
            }
            if let Err(err) = file_buffer.flush() {
                return WorkResult::new_res_err(outcome, err);
            }
            WorkResult::new_res(outcome)
        };
        self.employ_worker(game_type, work, None, WorkTask::Store);
    }

    fn save_strat(&mut self, game_type: GameType, strat: bf::CopStrategy) {
        let work = move || {
            let path = game_type.file_name("bruteforce-police");
            let file = match fs::File::create(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_strat_err(strat, err),
            };
            let mut file_buffer = std::io::BufWriter::new(file);
            if let Err(err) = rmp_serde::encode::write(&mut file_buffer, &strat) {
                return WorkResult::new_strat_err(strat, err);
            }
            if let Err(err) = file_buffer.flush() {
                return WorkResult::new_strat_err(strat, err);
            }
            WorkResult::new_strat(strat)
        };
        self.employ_worker(game_type, work, None, WorkTask::StoreStrat);
    }

    fn load_result_from(&mut self, game_type: GameType) {
        let (here, there) = bf::thread_manager::build_managers();
        let work = move || {
            let path = game_type.file_name("bruteforce");
            let file = match fs::File::open(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_err(err),
            };
            let buff_reader = std::io::BufReader::new(file);
            there.log.send("deserialisiere".into()).ok();
            match rmp_serde::decode::from_read::<_, GameOutcome>(buff_reader) {
                Ok(res) => {
                    let error = if let bf::Outcome::RobberWins(data) = &res.outcome {
                        let edges = game_type.create_edges();
                        bf::verify_continuity(data, &edges, &there).err()
                    } else {
                        None
                    };
                    WorkResult {
                        robber_strat: Some(res),
                        police_strat: None,
                        error,
                    }
                },
                Err(err) => WorkResult::new_err(err),
            }
        };
        self.employ_worker(game_type, work, Some(here), WorkTask::Load);
    }

    fn load_strat_from(&mut self, game_type: GameType) {
        let work = move || {
            let path = game_type.file_name("bruteforce-police");
            let file = match fs::File::open(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_err(err),
            };
            let buff_reader = std::io::BufReader::new(file);
            match rmp_serde::decode::from_read::<_, bf::CopStrategy>(buff_reader) {
                Ok(mut res) => {
                    res.compute_serde_skipped();
                    WorkResult::new_strat(res)
                },
                Err(err) => WorkResult::new_err(err),
            }
        };
        self.employ_worker(game_type, work, None, WorkTask::LoadStrat);
    }

    fn draw_result(ui: &mut Ui, game_type: &GameType, outcome: &GameOutcome) {
        let outcome_str = match &outcome.outcome {
            bf::Outcome::CopsWin => "verliert",
            bf::Outcome::RobberWins(_) => "gewinnt",
        };

        let cops_str = if game_type.nr_cops == 1 {
            "einen Cop".to_owned()
        } else {
            game_type.nr_cops.to_string() + " Cops"
        };

        let confidence_str = match &outcome.confidence {
            Confidence::Both => "(verifiziert)".to_string(),
            Confidence::NoSymmetry => "(Algo ohne Symmetrie)".to_string(),
            Confidence::SymmetryOnly => "(Algo mit Symmetrie)".to_string(),
            Confidence::Err(err) => format!("Validierungsfehler: {err}"),
        };
        ui.add(
            Label::new(format!(
                "Räuber {outcome_str} gegen {cops_str} auf {} mit Auflösung {} {confidence_str}",
                game_type.shape.to_sting(),
                game_type.resolution
            ))
            .wrap_mode(egui::TextWrapMode::Extend),
        );
    }

    fn draw_strat(ui: &mut Ui, game_type: &GameType, strat: &bf::CopStrategy) {
        let one_cop = game_type.nr_cops == 1;
        let cops_str = if one_cop {
            "ein Cop".to_owned()
        } else {
            game_type.nr_cops.to_string() + " Cops"
        };
        let outcome = if strat.cops_win { "gewinn" } else { "verlier" };
        let outcome_end = if one_cop { "t" } else { "en" };

        ui.add(
            Label::new(format!(
                "{} {}{} auf {} mit Auflösung {} (max. {} Züge)",
                cops_str,
                outcome,
                outcome_end,
                game_type.shape.to_sting(),
                game_type.resolution,
                strat.max_moves,
            ))
            .wrap_mode(egui::TextWrapMode::Extend),
        );
    }

    fn draw_results(&mut self, ui: &mut Ui) {
        enum Action {
            Verify,
            Store,
            Delete,
        }
        ui.add_space(5.0);
        let mut action = None;
        for (game_type, outcome) in &self.robber_strats {
            Self::draw_result(ui, game_type, outcome);
            ui.horizontal(|ui| {
                if NATIVE && ui.button("speichern").clicked() {
                    action = Some((Action::Store, *game_type));
                } else if outcome.confidence == Confidence::SymmetryOnly
                    && ui.button("verifizieren").clicked()
                {
                    action = Some((Action::Verify, *game_type));
                } else if ui.button("löschen").clicked() {
                    action = Some((Action::Delete, *game_type));
                }
            });
            ui.add_space(5.0);
        }
        if let Some((a, game_type)) = action {
            let outcome = self.robber_strats.remove(&game_type).unwrap();
            match a {
                Action::Delete => {},
                Action::Store => self.save_result(game_type, outcome),
                Action::Verify => self.verify_result(game_type, outcome),
            }
        }

        let mut action = None;
        for (game_type, strat) in &self.cop_strats {
            Self::draw_strat(ui, game_type, strat);
            ui.horizontal(|ui| {
                if NATIVE && ui.button("speichern").clicked() {
                    action = Some((Action::Store, *game_type));
                } else if ui.button("löschen").clicked() {
                    action = Some((Action::Delete, *game_type));
                }
                ui.menu_button("meiste Züge", |ui| {
                    for extreme_pos in &strat.extreme_positions {
                        ui.label(format!("{extreme_pos:?}"));
                    }
                });
            });
            ui.add_space(5.0);
        }
        if let Some((a, game_type)) = action {
            let strat = self.cop_strats.remove(&game_type).unwrap();
            match a {
                Action::Delete | Action::Verify => {},
                Action::Store => self.save_strat(game_type, strat),
            }
        }
    }

    fn draw_workers(&mut self, ui: &mut Ui) {
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
        let mouse_down = ui.input(|i| i.pointer.button_down(egui::PointerButton::Primary));

        for worker in &mut self.workers {
            let paused = worker.manager.as_ref().is_some_and(|m| m.is_paused());
            let animation = match (ui.input(|i| i.time) * 5.0) as isize % 9 {
                _ if paused => " . . ",
                0 => ".    ",
                1 => "..   ",
                2 => "...  ",
                3 => ".... ",
                4 => " ... ",
                5 => "  .. ",
                6 => "   . ",
                _ => "     ",
            };

            let task_str = match worker.task {
                WorkTask::Compute => "rechne (R) ",
                WorkTask::ComputeStrat => "rechne (C) ",
                WorkTask::Verify => "verifiziere ",
                WorkTask::Load | WorkTask::LoadStrat => "lade ",
                WorkTask::Store | WorkTask::StoreStrat => "speichere ",
            };

            ui.add(
                Label::new(format!(
                    "{} {} {}",
                    animation,
                    task_str,
                    worker.game_type.as_tuple_string()
                ))
                .wrap_mode(egui::TextWrapMode::Extend),
            );

            if let Some(r) = &mut worker.manager {
                ui.add(Label::new(r.last_log()).wrap_mode(egui::TextWrapMode::Extend));
                ui.horizontal(|ui| {
                    {
                        let (label, cmd) = if paused {
                            (" ▶ ", Command::Work)
                        } else {
                            (" ⏸ ", Command::Pause)
                        };
                        if ui.button(label).clicked() {
                            r.send_command(cmd).ok();
                        }
                    }
                    let abort_button = ui.add_enabled(paused, Button::new("abbrechen"));
                    if abort_button.contains_pointer() && paused && mouse_down {
                        if let Some((gt, t, start)) = self.abort_worker {
                            if gt == worker.game_type && t == worker.task {
                                let time_since_click = Instant::now() - start;
                                let ratio = time_since_click.as_secs_f32() / 5.0;
                                if ratio < 1.0 {
                                    let text = format!("{}%", (ratio * 100.0) as isize);
                                    abort_button.on_hover_text(text);
                                } else {
                                    r.send_command(Command::Abort).ok();
                                }
                            }
                        } else {
                            // only set to this if mouse was released after last abort
                            assert!(self.abort_worker.is_none());
                            self.abort_worker =
                                Some((worker.game_type, worker.task, Instant::now()));
                        }
                    }
                });
            }
            ui.add_space(5.0);
        }
        if !mouse_down {
            // only set to none if mouse is no longer pressed
            // -> require mouse release to abort multiple workers
            self.abort_worker = None;
        }
    }

    fn draw_errors(&mut self, ui: &mut Ui) {
        let mut delete = None;
        for (i, (game_type, err)) in izip!(0.., &self.errors) {
            ui.add_space(5.0);
            ui.add(
                Label::new(format!(
                    "Fehler für {}:\n{}",
                    game_type.as_tuple_string(),
                    err
                ))
                .wrap_mode(egui::TextWrapMode::Extend),
            );
            if ui.button("löschen").clicked() {
                delete = Some(i);
            }
        }
        if let Some(i) = delete {
            self.errors.remove(i);
        }
    }

    pub fn draw_menu(&mut self, nr_cops: usize, ui: &mut Ui, map: &map::Map) {
        ui.collapsing("Bruteforce", |ui| {
            self.check_on_workers();
            let game_type = GameType {
                nr_cops,
                resolution: map.resolution(),
                shape: map.shape(),
            };
            if game_type != self.curr_game_type {
                self.curr_game_type = game_type;
                self.robber_strat_stored = NATIVE && game_type.file_name("bruteforce").exists();
                self.cops_strat_stored =
                    NATIVE && game_type.file_name("bruteforce-police").exists();
            }

            ui.horizontal(|ui| {
                let nr_active = self.workers.len();
                if NATIVE && nr_active > 0 {
                    ui.menu_button(format!("{nr_active} aktiv"), |ui| {
                        egui::ScrollArea::vertical().show(ui, |ui| self.draw_workers(ui));
                    });
                } else if NATIVE {
                    ui.add_enabled(false, Button::new("0 aktiv"));
                }

                let nr_done = self.robber_strats.len() + self.cop_strats.len();
                if nr_done > 0 {
                    ui.menu_button(format!("{nr_done} fertig"), |ui| {
                        egui::ScrollArea::vertical().show(ui, |ui| self.draw_results(ui));
                    });
                } else {
                    ui.add_enabled(false, Button::new("0 fertig"));
                }

                let nr_errs = self.errors.len();
                if nr_errs > 0 {
                    ui.menu_button(format!("{nr_errs} 💥"), |ui| {
                        egui::ScrollArea::vertical().show(ui, |ui| self.draw_errors(ui));
                    });
                } else {
                    ui.add_enabled(false, Button::new("0 💥"));
                }
            });
            ui.add_space(5.0);

            let disclaimer = "WARNUNG: weil WASM keine Threads mag, blockt \
                die Websiteversion bei dieser Rechnung die GUI.\n\
                Ausserdem: WASM is 32 bit, kann also nur 4GiB RAM benutzen, was die spannenden \
                Bruteforceberechnungen nicht möglich macht.";

            ui.label("Räuberstrategie:");
            let computing_robber_strat = self.workers.iter().any(|w| {
                w.game_type == game_type && matches!(w.task, WorkTask::Compute | WorkTask::Load)
            });
            ui.horizontal(|ui| {
                let curr_known = self.robber_strats.contains_key(&game_type);
                let enable_compute = !computing_robber_strat && !curr_known;
                let compute_button = Button::new("berechnen");
                if ui
                    .add_enabled(enable_compute, compute_button)
                    .on_hover_text(disclaimer)
                    .clicked()
                {
                    self.start_computation(nr_cops, map);
                }
                let enable_load = self.robber_strat_stored && !curr_known;
                let load_button = Button::new("laden 🖴");
                if ui.add_enabled(enable_load, load_button).clicked() {
                    self.load_result_from(game_type);
                }
            });
            ui.add_space(5.0);
            ui.label("Copstrategie:");
            let computing_cop_strat = self.workers.iter().any(|worker| {
                worker.game_type == game_type && worker.task == WorkTask::ComputeStrat
            });
            ui.horizontal(|ui| {
                let curr_known = self.cop_strats.contains_key(&game_type);
                let enable_compute = !computing_cop_strat && !curr_known;
                let compute_button = Button::new("berechnen");
                if ui
                    .add_enabled(enable_compute, compute_button)
                    .on_hover_text(disclaimer)
                    .clicked()
                {
                    self.start_strat_computation(nr_cops, map);
                }
                let enable_load = self.cops_strat_stored && !curr_known;
                let load_button = Button::new("laden 🖴");
                if ui.add_enabled(enable_load, load_button).clicked() {
                    self.load_strat_from(game_type);
                }
            });
        });
    }
}
