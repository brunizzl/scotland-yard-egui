use std::{fs, io::Write, path::PathBuf, thread};

use serde::{Deserialize, Serialize};

use super::*;
use crate::graph::bruteforce as bf;

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq)]
pub struct GameType {
    pub nr_cops: usize,
    pub nr_vertices: usize,
    pub shape: map::Shape,
}

impl GameType {
    fn file_name(&self) -> PathBuf {
        PathBuf::from(format!(
            "bruteforce/{}-{}-{}.msgpack",
            self.nr_cops,
            self.shape.to_str(),
            self.nr_vertices
        ))
    }

    fn as_tuple_string(&self) -> String {
        format!(
            "({}, {}, {})",
            self.nr_cops,
            self.shape.to_str(),
            self.nr_vertices
        )
    }
}

const NATIVE: bool = cfg!(not(target_arch = "wasm32"));

#[derive(Clone, Copy, PartialEq, Eq)]
enum WorkTask {
    Compute,
    Load,
    Store,
}

struct WorkResult {
    result: Option<bf::Outcome>,
    error: Option<String>,
}

impl WorkResult {
    fn new_err<E: ToString>(err: E) -> Self {
        Self {
            result: None,
            error: Some(err.to_string()),
        }
    }

    fn new_both<E: ToString>(outcome: bf::Outcome, err: E) -> Self {
        Self {
            result: Some(outcome),
            error: Some(err.to_string()),
        }
    }

    fn new_success(outcome: bf::Outcome) -> Self {
        Self {
            result: Some(outcome),
            error: None,
        }
    }
}

impl From<Result<bf::Outcome, String>> for WorkResult {
    fn from(res: Result<bf::Outcome, String>) -> Self {
        let (result, error) = match res {
            Ok(ok) => (Some(ok), None),
            Err(err) => (None, Some(err)),
        };
        Self { result, error }
    }
}

pub struct BruteforceWorker {
    worker: Option<(GameType, thread::JoinHandle<WorkResult>, WorkTask)>,
    result: Option<(GameType, bf::Outcome)>,
    error: Option<(GameType, String)>,
}

impl BruteforceWorker {
    pub fn new() -> Self {
        Self {
            worker: None,
            result: None,
            error: None,
        }
    }

    fn current_worker_task(&self) -> Option<WorkTask> {
        self.worker.as_ref().map(|w| w.2)
    }

    pub fn result(&self) -> Option<&(GameType, bf::Outcome)> {
        self.result.as_ref()
    }

    fn process_result(
        &mut self,
        game_type: GameType,
        res: WorkResult,
    ) -> Option<(GameType, bf::Outcome)> {
        if let Some(err) = res.error {
            self.error = Some((game_type, err));
        } else if res.result.is_some() {
            self.error = None;
        }

        if let Some(outcome) = res.result {
            std::mem::replace(&mut self.result, Some((game_type, outcome)))
        } else {
            None
        }
    }

    fn employ_worker<F>(&mut self, game_type: GameType, work: F, job_description: WorkTask)
    where
        F: FnOnce() -> WorkResult + Send + 'static,
    {
        assert!(self.current_worker_task().is_none());

        if NATIVE {
            //use threads natively
            self.worker = Some((game_type, thread::spawn(work), job_description));
        } else {
            //wasm doesn't like threads -> just block gui
            let _ = self.process_result(game_type, work());
        }
    }

    fn check_on_worker(&mut self) {
        if let Some((game_type, handle, task)) = std::mem::take(&mut self.worker) {
            if handle.is_finished() {
                if let Ok(ok) = handle.join() {
                    self.process_result(game_type, ok);
                } else {
                    self.worker = None;
                }
            } else {
                self.worker = Some((game_type, handle, task));
            }
        }
    }

    fn start_computation(&mut self, nr_cops: usize, map: &map::Map) {
        if self.current_worker_task().is_some() {
            return;
        }
        let game_type = GameType {
            nr_cops,
            nr_vertices: map.edges().nr_vertices(),
            shape: map.shape(),
        };
        match map.data().sym_group() {
            SymGroup::Explicit(equiv) => {
                let sym = bf::SymmetricMap {
                    edges: map.edges().clone(),
                    symmetry: equiv.clone(),
                };
                let work = move || bf::compute_safe_robber_positions(nr_cops, sym).into();
                self.employ_worker(game_type, work, WorkTask::Compute);
            },
            SymGroup::None(none) => {
                let sym = bf::SymmetricMap {
                    edges: map.edges().clone(),
                    symmetry: *none,
                };
                let work = move || bf::compute_safe_robber_positions(nr_cops, sym).into();
                self.employ_worker(game_type, work, WorkTask::Compute);
            },
        }
    }

    fn save_result(&mut self) {
        if self.current_worker_task().is_some() {
            return;
        }
        if let Some((game_type, outcome)) = std::mem::take(&mut self.result) {
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
            self.employ_worker(game_type, work, WorkTask::Store);
        }
    }

    fn load_result_from(&mut self, game_type: GameType) {
        if self.current_worker_task().is_some() {
            return;
        }
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
        self.employ_worker(game_type, work, WorkTask::Load);
    }

    fn draw_result(&self, ui: &mut Ui) {
        if let Some((game_type, outcome)) = &self.result {
            let outcome_str = match outcome {
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
                game_type.shape.to_str(),
                game_type.nr_vertices
            ));
        } else {
            ui.label("noch keine beendete Rechnung");
        }

        if let Some((game_type, err)) = &self.error {
            ui.label(format!(
                "Fehler für {}:\n{}",
                game_type.as_tuple_string(),
                err
            ));
        }
    }

    pub fn draw_menu(&mut self, nr_cops: usize, ui: &mut Ui, map: &map::Map) {
        ui.collapsing("Bruteforce", |ui| {
            if let Some((_, _, curr_task)) = &self.worker {
                ui.horizontal(|ui| {
                    ui.label(match curr_task {
                        WorkTask::Compute => "rechne ",
                        WorkTask::Load => "lade ",
                        WorkTask::Store => "speichere ",
                    });
                    ui.add(egui::widgets::Spinner::new());
                });
                self.check_on_worker();
            } else if ui
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
            } else if NATIVE && self.result.is_some() && ui.button("speichere Daten").clicked() {
                self.save_result();
            } else if NATIVE && ui.button("lade Daten").clicked() {
                let game_type = GameType {
                    nr_cops,
                    nr_vertices: map.data().nr_vertices(),
                    shape: map.shape(),
                };
                self.load_result_from(game_type);
            }

            self.draw_result(ui);
        });
    }
}
