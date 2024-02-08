use std::{fs, io::Write, path::PathBuf, thread};

use serde::{Deserialize, Serialize};

use super::*;
use crate::graph::bruteforce as bf;
use crate::graph::NoSymmetry;

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct GameType {
    pub nr_cops: usize,
    pub resolution: usize,
    pub shape: map::Shape,
}

impl GameType {
    fn file_name(&self) -> PathBuf {
        PathBuf::from(format!(
            "bruteforce/{}-{}-{}.msgpack",
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
        let embedding = map::new_map_from(self.shape, self.resolution);
        embedding.into_edges()
    }
}

const NATIVE: bool = cfg!(not(target_arch = "wasm32"));

#[derive(Clone, Copy, PartialEq, Eq)]
enum WorkTask {
    Compute,
    Verify,
    Load,
    Store,
}

/// sometimes, e.g. when storing or when verifying, we both have a result and could encounter an error.
/// thus: this is not eighter / or, but can be both.
struct WorkResult {
    result: Option<GameOutcome>,
    error: Option<String>,
}

impl WorkResult {
    fn new_err<E: ToString>(err: E) -> Self {
        Self {
            result: None,
            error: Some(err.to_string()),
        }
    }

    fn new_both<E: ToString>(outcome: GameOutcome, err: E) -> Self {
        Self {
            result: Some(outcome),
            error: Some(err.to_string()),
        }
    }

    fn new_success(outcome: GameOutcome) -> Self {
        Self {
            result: Some(outcome),
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
        Self { result, error }
    }
}

use std::sync::mpsc;
type RcvStr = mpsc::Receiver<&'static str>;
struct Worker {
    game_type: GameType,
    task: WorkTask,
    handle: thread::JoinHandle<WorkResult>,

    reciever: Option<RcvStr>,
    status: Option<&'static str>,
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
    results: BTreeMap<GameType, GameOutcome>,
    error: Option<(GameType, String)>,
}

impl BruteforceComputationState {
    pub fn new() -> Self {
        Self {
            workers: Vec::new(),
            results: BTreeMap::new(),
            error: None,
        }
    }

    pub fn result_for(&self, game_type: &GameType) -> Option<&bf::Outcome> {
        self.results.get(game_type).map(|o| &o.outcome)
    }

    fn process_result(&mut self, game_type: GameType, res: WorkResult) {
        if let Some(err) = res.error {
            self.error = Some((game_type, err));
        } else if res.result.is_some() {
            self.error = None;
        }

        if let Some(outcome) = res.result {
            self.results.insert(game_type, outcome);
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
        let mut temp_workers = Vec::with_capacity(self.workers.len());
        for worker in std::mem::take(&mut self.workers) {
            let Worker {
                game_type,
                task,
                handle,
                reciever: mut status_reciever,
                mut status,
            } = worker;
            if handle.is_finished() {
                if let Ok(ok) = handle.join() {
                    self.process_result(game_type, ok);
                }
            } else {
                if let Some(rcv) = &mut status_reciever {
                    if let Ok(msg) = rcv.try_recv() {
                        status = Some(msg);
                    }
                }
                temp_workers.push(Worker {
                    game_type,
                    task,
                    handle,
                    reciever: status_reciever,
                    status,
                });
            }
        }
        self.workers = temp_workers;
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
                Err(err) => return WorkResult::new_both(outcome, err),
            };
            send.send("vergleiche Ergebnisse").ok();
            use bf::Outcome::*;
            outcome.confidence = match (&outcome.outcome, &outcome_without) {
                (CopsWin, CopsWin) => Confidence::Both,
                (CopsWin, RobberWins(_)) => Confidence::new_err("Ohne sym. gewinnt Räuber"),
                (RobberWins(_), CopsWin) => Confidence::new_err("Ohne sym. gewinnen Cops"),
                (RobberWins(sym), RobberWins(no_sym)) => Self::verify_robber_win(sym, no_sym),
            };
            WorkResult::new_success(outcome)
        };
        self.employ_worker(game_type, work, Some(recieve), WorkTask::Verify);
    }

    fn save_result(&mut self, game_type: GameType, outcome: GameOutcome) {
        let work = move || {
            let path = game_type.file_name();
            let file = match fs::File::create(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_both(outcome, err),
            };
            let mut file_buffer = std::io::BufWriter::new(file);
            if let Err(err) = rmp_serde::encode::write(&mut file_buffer, &outcome) {
                return WorkResult::new_both(outcome, err);
            }
            if let Err(err) = file_buffer.flush() {
                return WorkResult::new_both(outcome, err);
            }
            WorkResult::new_success(outcome)
        };
        self.employ_worker(game_type, work, None, WorkTask::Store);
    }

    fn load_result_from(&mut self, game_type: GameType) {
        let work = move || {
            let path = game_type.file_name();
            let file = match fs::File::open(path) {
                Ok(file) => file,
                Err(err) => return WorkResult::new_err(err),
            };
            let buff_reader = std::io::BufReader::new(file);
            match rmp_serde::decode::from_read(buff_reader) {
                Ok(res) => WorkResult::new_success(res),
                Err(err) => WorkResult::new_err(err),
            }
        };
        self.employ_worker(game_type, work, None, WorkTask::Load);
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
            "Räuber {} gegen {} auf {} mit {} Knoten",
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

    fn draw_results(&mut self, ui: &mut Ui) {
        ui.add_space(5.0);
        let mut temp_results = BTreeMap::new();
        for (game_type, outcome) in std::mem::take(&mut self.results) {
            Self::draw_result(ui, &game_type, &outcome);
            ui.horizontal(|ui| {
                if NATIVE && ui.button("speichern").clicked() {
                    self.save_result(game_type, outcome);
                } else if outcome.confidence == Confidence::SymmetryOnly
                    && ui.button("verifizieren").clicked()
                {
                    self.verify_result(game_type, outcome);
                } else if !ui.button("löschen").clicked() {
                    temp_results.insert(game_type, outcome);
                }
            });
            ui.add_space(5.0);
        }
        self.results = temp_results;
    }

    pub fn draw_menu(&mut self, nr_cops: usize, ui: &mut Ui, map: &map::Map) {
        ui.collapsing("Bruteforce", |ui| {
            self.check_on_workers();
            let game_type = GameType {
                nr_cops,
                resolution: map.resolution(),
                shape: map.shape(),
            };
            let mut computing_curr = false;
            for worker in &self.workers {
                if worker.game_type == game_type && worker.task == WorkTask::Compute {
                    computing_curr = true;
                }
                ui.horizontal(|ui| {
                    ui.add(egui::widgets::Spinner::new());
                    let task_str = match worker.task {
                        WorkTask::Compute => "rechne ",
                        WorkTask::Verify => "verifiziere ",
                        WorkTask::Load => "lade ",
                        WorkTask::Store => "speichere ",
                    };
                    ui.label(format!(
                        "{} {}",
                        task_str,
                        worker.game_type.as_tuple_string()
                    ));
                });
                if let Some(msg) = worker.status {
                    ui.label(msg);
                }
                ui.add_space(5.0);
            }
            let already_know_curr = self.results.contains_key(&game_type);
            if !computing_curr
                && !already_know_curr
                && ui
                    .button("Starte Rechnung")
                    .on_hover_text(
                        "WARNUNG: weil WASM keine Threads mag, blockt \
                die Websiteversion bei dieser Rechnung die GUI.\n\
                Ausserdem: WASM is 32 bit, kann also nur 4GiB RAM benutzen, was die spannenden \
                Bruteforceberechnungen nicht in RAM möglich macht.",
                    )
                    .clicked()
            {
                self.start_computation(nr_cops, map);
            } else if NATIVE && !already_know_curr && ui.button("laden 🖴").clicked() {
                self.load_result_from(game_type);
            }

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
