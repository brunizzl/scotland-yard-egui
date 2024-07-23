use std::{fs, io::Write, path::PathBuf, thread};

use itertools::izip;
use serde::{Deserialize, Serialize};

use super::*;
use crate::graph::bruteforce as bf;
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
    result: Option<GameOutcome>,
    cop_strat: Option<bf::CopStrategy>,
    error: Option<String>,
}

impl WorkResult {
    fn new_err<E: ToString>(err: E) -> Self {
        Self {
            result: None,
            cop_strat: None,
            error: Some(err.to_string()),
        }
    }

    fn new_res_err<E: ToString>(outcome: GameOutcome, err: E) -> Self {
        Self {
            result: Some(outcome),
            cop_strat: None,
            error: Some(err.to_string()),
        }
    }

    fn new_strat_err<E: ToString>(strat: bf::CopStrategy, err: E) -> Self {
        Self {
            result: None,
            cop_strat: Some(strat),
            error: Some(err.to_string()),
        }
    }

    fn new_strat(strat: bf::CopStrategy) -> Self {
        Self {
            result: None,
            cop_strat: Some(strat),
            error: None,
        }
    }

    fn new_res(outcome: GameOutcome) -> Self {
        Self {
            result: Some(outcome),
            cop_strat: None,
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
        Self { result, cop_strat: None, error }
    }
}

impl From<Result<bf::CopStrategy, String>> for WorkResult {
    fn from(res: Result<bf::CopStrategy, String>) -> Self {
        let (cop_strat, error) = match res {
            Ok(strat) => (Some(strat), None),
            Err(err) => (None, Some(err)),
        };
        Self { result: None, cop_strat, error }
    }
}

use std::sync::mpsc;
type RcvStr = mpsc::Receiver<String>;
struct Worker {
    game_type: GameType,
    task: WorkTask,
    handle: thread::JoinHandle<WorkResult>,

    reciever: Option<RcvStr>,
    status: Option<String>,
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
    curr_game_type: GameType,
    robber_strat_stored: bool,
    cops_strat_stored: bool,
    results: BTreeMap<GameType, GameOutcome>,
    cop_strats: BTreeMap<GameType, bf::CopStrategy>,
    error: Option<(GameType, String)>,
}

impl BruteforceComputationState {
    pub fn new() -> Self {
        Self {
            workers: Vec::new(),
            curr_game_type: GameType {
                nr_cops: usize::MAX,
                resolution: 0,
                shape: crate::graph::Shape::Icosahedron,
            },
            robber_strat_stored: false,
            cops_strat_stored: false,
            results: BTreeMap::new(),
            cop_strats: BTreeMap::new(),
            error: None,
        }
    }

    pub fn result_for(&self, game_type: &GameType) -> Option<&bf::Outcome> {
        self.results.get(game_type).map(|o| &o.outcome)
    }

    pub fn strats_for(&self, game_type: &GameType) -> Option<&bf::CopStrategy> {
        self.cop_strats.get(game_type)
    }

    fn process_result(&mut self, game_type: GameType, res: WorkResult) {
        if let Some(err) = res.error {
            self.error = Some((game_type, err));
        } else if res.result.is_some() || res.cop_strat.is_some() {
            self.error = None;
        }

        if let Some(outcome) = res.result {
            self.results.insert(game_type, outcome);
        }
        if let Some(strat) = res.cop_strat {
            self.cop_strats.insert(game_type, strat);
        }
    }

    fn employ_worker<F>(
        &mut self,
        game_type: GameType,
        work: F,
        reciever: Option<RcvStr>,
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
                reciever,
                status: None,
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
            if let Some(rcv) = &worker.reciever {
                while let Ok(msg) = rcv.try_recv() {
                    worker.status = Some(msg);
                }
            }
        }
        if let Some(i) = done {
            let worker = self.workers.remove(i);
            if let Ok(ok) = worker.handle.join() {
                self.process_result(worker.game_type, ok);
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
        let (send, recieve) = mpsc::channel();
        let (send, recieve) = (Some(send), Some(recieve));
        match map.data().sym_group() {
            SymGroup::Explicit(equiv) => {
                let sym = equiv.clone();
                let work = move || {
                    let res = bf::compute_safe_robber_positions(nr_cops, edges, sym, send);
                    (res, Confidence::SymmetryOnly).into()
                };
                self.employ_worker(game_type, work, recieve, WorkTask::Compute);
            },
            SymGroup::Torus6(torus) => {
                let sym = torus.clone();
                let work = move || {
                    let res = bf::compute_safe_robber_positions(nr_cops, edges, sym, send);
                    (res, Confidence::SymmetryOnly).into()
                };
                self.employ_worker(game_type, work, recieve, WorkTask::Compute);
            },
            SymGroup::Torus4(torus) => {
                let sym = torus.clone();
                let work = move || {
                    let res = bf::compute_safe_robber_positions(nr_cops, edges, sym, send);
                    (res, Confidence::SymmetryOnly).into()
                };
                self.employ_worker(game_type, work, recieve, WorkTask::Compute);
            },
            SymGroup::None(none) => {
                let sym = *none;
                let work = move || {
                    let res = bf::compute_safe_robber_positions(nr_cops, edges, sym, send);
                    (res, Confidence::NoSymmetry).into()
                };
                self.employ_worker(game_type, work, recieve, WorkTask::Compute);
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
        let (send, recieve) = mpsc::channel();
        let (send, recieve) = (Some(send), Some(recieve));
        match map.data().sym_group() {
            SymGroup::Explicit(equiv) => {
                let sym = equiv.clone();
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, send).into();
                self.employ_worker(game_type, work, recieve, WorkTask::ComputeStrat);
            },
            SymGroup::Torus6(torus) => {
                let sym = torus.clone();
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, send).into();
                self.employ_worker(game_type, work, recieve, WorkTask::ComputeStrat);
            },
            SymGroup::Torus4(torus) => {
                let sym = torus.clone();
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, send).into();
                self.employ_worker(game_type, work, recieve, WorkTask::ComputeStrat);
            },
            SymGroup::None(none) => {
                let sym = *none;
                let work = move || bf::compute_cop_strategy(nr_cops, edges, sym, send).into();
                self.employ_worker(game_type, work, recieve, WorkTask::ComputeStrat);
            },
        }
    }

    fn verify_robber_win(sym: &bf::RobberWinData, no_sym: &bf::RobberWinData) -> Confidence {
        debug_assert_eq!(sym.cop_moves.nr_cops(), no_sym.cop_moves.nr_cops());
        debug_assert_eq!(
            sym.cop_moves.nr_map_vertices(),
            no_sym.cop_moves.nr_map_vertices()
        );
        let nr_cops = sym.cop_moves.nr_cops();
        let nr_map_vertices = sym.cop_moves.nr_map_vertices();

        let mut confidence = Confidence::SymmetryOnly;
        for cop_config in no_sym.cop_moves.all_positions_unpacked() {
            let cop_config = &cop_config[..nr_cops];
            let (_, all_index) = no_sym.pack(cop_config.iter().copied());
            let (autos, sym_index) = sym.pack(cop_config.iter().copied());
            for auto in autos {
                let sym_robber_range = sym.safe.robber_indices_at(sym_index);
                let all_robber_range = no_sym.safe.robber_indices_at(all_index);
                for v in 0..nr_map_vertices {
                    let v_rot = auto.dyn_apply_forward(v);
                    if sym.safe.robber_safe_at(sym_robber_range.at(v_rot))
                        != no_sym.safe.robber_safe_at(all_robber_range.at(v))
                    {
                        confidence = Confidence::Err(format!(
                            "Fehler: Konfig {:?} uneinig in Knoten {} (rotiert = {})",
                            cop_config, v, v_rot
                        ));
                    }
                }
            }
        }
        if confidence == Confidence::SymmetryOnly {
            confidence = Confidence::Both;
        }
        confidence
    }

    fn verify_result(&mut self, game_type: GameType, mut outcome: GameOutcome) {
        let (send, recieve) = mpsc::channel();
        let work = move || {
            let edges = game_type.create_edges();
            let nr_map_vertices = edges.nr_vertices();
            let sym = NoSymmetry::new(nr_map_vertices);
            let res_without = bf::compute_safe_robber_positions(
                game_type.nr_cops,
                edges,
                sym,
                Some(send.clone()),
            );
            let outcome_without = match res_without {
                Ok(ok) => ok,
                Err(err) => return WorkResult::new_res_err(outcome, err),
            };
            send.send("vergleiche Ergebnisse".into()).ok();
            use bf::Outcome::*;
            outcome.confidence = match (&outcome.outcome, &outcome_without) {
                (CopsWin, CopsWin) => Confidence::Both,
                (CopsWin, RobberWins(_)) => Confidence::new_err("Ohne sym. gewinnt Räuber"),
                (RobberWins(_), CopsWin) => Confidence::new_err("Ohne sym. gewinnen Cops"),
                (RobberWins(sym), RobberWins(no_sym)) => Self::verify_robber_win(sym, no_sym),
            };
            WorkResult::new_res(outcome)
        };
        self.employ_worker(game_type, work, Some(recieve), WorkTask::Verify);
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
        let work = move || {
            let path = game_type.file_name("bruteforce");
            let file = match fs::File::open(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_err(err),
            };
            let buff_reader = std::io::BufReader::new(file);
            match rmp_serde::decode::from_read(buff_reader) {
                Ok(res) => WorkResult::new_res(res),
                Err(err) => WorkResult::new_err(err),
            }
        };
        self.employ_worker(game_type, work, None, WorkTask::Load);
    }

    fn load_strat_from(&mut self, game_type: GameType) {
        let work = move || {
            let path = game_type.file_name("bruteforce-police");
            let file = match fs::File::open(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_err(err),
            };
            let buff_reader = std::io::BufReader::new(file);
            match rmp_serde::decode::from_read(buff_reader) {
                Ok(res) => WorkResult::new_strat(res),
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

        ui.label(format!(
            "Räuber {} gegen {} auf {} mit Auflösung {}",
            outcome_str,
            cops_str,
            game_type.shape.to_sting(),
            game_type.resolution
        ));
        ui.label(match &outcome.confidence {
            Confidence::Both => "(verifiziert)".to_string(),
            Confidence::NoSymmetry => "(Algo ohne Symmetrie)".to_string(),
            Confidence::SymmetryOnly => "(Algo mit Symmetrie)".to_string(),
            Confidence::Err(err) => format!("Validierungsfehler: {err}"),
        });
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

        ui.label(format!(
            "{} {}{} auf {} mit Auflösung {} (max. {} Züge)",
            cops_str,
            outcome,
            outcome_end,
            game_type.shape.to_sting(),
            game_type.resolution,
            strat.max_moves,
        ));
    }

    fn draw_results(&mut self, ui: &mut Ui) {
        enum Action {
            Verify,
            Store,
            Delete,
        }
        ui.add_space(5.0);
        let mut action = None;
        for (game_type, outcome) in &self.results {
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
            let outcome = self.results.remove(&game_type).unwrap();
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
            let mut computing_curr = false;
            let mut computing_curr_strat = false;
            for worker in &self.workers {
                if worker.game_type == game_type {
                    computing_curr |= matches!(worker.task, WorkTask::Compute | WorkTask::Load);
                    computing_curr_strat |= worker.task == WorkTask::ComputeStrat;
                }
                ui.horizontal(|ui| {
                    ui.add(egui::widgets::Spinner::new());
                    let task_str = match worker.task {
                        WorkTask::Compute | WorkTask::ComputeStrat => "rechne ",
                        WorkTask::Verify => "verifiziere ",
                        WorkTask::Load | WorkTask::LoadStrat => "lade ",
                        WorkTask::Store | WorkTask::StoreStrat => "speichere ",
                    };
                    ui.label(format!(
                        "{} {}",
                        task_str,
                        worker.game_type.as_tuple_string()
                    ));
                });
                if let Some(msg) = &worker.status {
                    ui.label(msg);
                }
                ui.add_space(5.0);
            }

            let disclaimer = "WARNUNG: weil WASM keine Threads mag, blockt \
                die Websiteversion bei dieser Rechnung die GUI.\n\
                Ausserdem: WASM is 32 bit, kann also nur 4GiB RAM benutzen, was die spannenden \
                Bruteforceberechnungen nicht möglich macht.";

            ui.label("Räuberstrategie:");
            ui.horizontal(|ui| {
                let curr_known = self.results.contains_key(&game_type);
                let enable_compute = !computing_curr && !curr_known;
                let compute_button = Button::new("Berechnen");
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
            ui.horizontal(|ui| {
                let curr_known = self.cop_strats.contains_key(&game_type);
                let enable_compute = !computing_curr_strat && !curr_known;
                let compute_button = Button::new("Berechnen");
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
            ui.add_space(5.0);

            self.draw_results(ui);

            if let Some((game_type, err)) = &self.error {
                ui.label(format!(
                    "Fehler für {}:\n{}",
                    game_type.as_tuple_string(),
                    err
                ));
            }
        });
    }
}
