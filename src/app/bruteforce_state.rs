use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
    thread,
};

use itertools::izip;

use super::*;
use crate::graph::*;

const NATIVE: bool = cfg!(not(target_arch = "wasm32"));

#[derive(Clone, Copy, PartialEq, Eq)]
enum WorkTask {
    Compute,
    ComputeVerification,
    Verify,
    Load,
    Store,
}

pub struct BruteforceWorker {
    worker: Option<(
        thread::JoinHandle<BruteForceResult<ExplicitClasses>>,
        WorkTask,
    )>,
    result: BruteForceResult<ExplicitClasses>,
}

impl BruteforceWorker {
    pub fn new() -> Self {
        Self {
            worker: None,
            result: BruteForceResult::None,
        }
    }

    fn current_worker_task(&self) -> Option<WorkTask> {
        self.worker.as_ref().map(|w| w.1)
    }

    pub fn result(&self) -> &BruteForceResult<ExplicitClasses> {
        &self.result
    }

    fn employ_worker<F>(&mut self, work: F, job_description: WorkTask)
    where
        F: FnOnce() -> BruteForceResult<ExplicitClasses> + Send + 'static,
    {
        assert!(self.current_worker_task().is_none());

        if NATIVE {
            //use threads natively
            self.worker = Some((thread::spawn(work), job_description));
        } else {
            //wasm doesn't like threads -> just block gui
            self.result = work();
        }
    }

    fn check_on_worker(&mut self) {
        if let Some((handle, curr_task)) = std::mem::take(&mut self.worker) {
            if handle.is_finished() {
                let old_result = std::mem::replace(
                    &mut self.result,
                    match handle.join() {
                        Err(_) => BruteForceResult::Error("Programmabsturz".to_owned()),
                        Ok(res) => res,
                    },
                );
                if curr_task == WorkTask::Compute {
                    if let BruteForceResult::RobberWins(data) = &self.result {
                        let nr_cops = data.nr_cops;
                        if NATIVE && data.validation == WinValidation::SymmetryOnly {
                            let no_sym = SymmetricMap {
                                shape: data.shape(),
                                edges: data.cop_moves.map().edges.clone(),
                                symmetry: NoSymmetry::new(data.nr_map_vertices()),
                            };
                            let work = move || {
                                compute_safe_robber_positions(nr_cops, no_sym).into_explicit()
                            };
                            self.employ_worker(work, WorkTask::ComputeVerification);
                        }
                    }
                }
                if curr_task == WorkTask::ComputeVerification {
                    //swap back to display old result, as new result contains same information (assuming correctness), but is way larger.
                    let new_result = std::mem::replace(&mut self.result, old_result);
                    if let BruteForceResult::RobberWins(data) = &self.result {
                        let nr_cops = data.nr_cops;
                        if data.validation != WinValidation::SymmetryOnly
                            || nr_cops > 3
                            || data.nr_map_vertices() > 500
                        {
                            return;
                        }
                        if let BruteForceResult::RobberWins(all_data) = new_result {
                            let mut sym_res = std::mem::take(&mut self.result);
                            let work = move || {
                                let BruteForceResult::RobberWins(sym_data) = &mut sym_res else {
                                    return sym_res;
                                };
                                for cop_config in all_data.cop_moves.all_positions_unpacked() {
                                    let cop_config = &cop_config[..nr_cops];
                                    let (_, all_index) =
                                        all_data.cop_moves.pack(cop_config.iter().copied());
                                    let (autos, sym_index) =
                                        sym_data.cop_moves.pack(cop_config.iter().copied());
                                    for auto in autos {
                                        let sym_robber_range =
                                            sym_data.safe.robber_indices_at(sym_index);
                                        let all_robber_range =
                                            all_data.safe.robber_indices_at(all_index);
                                        for (v, v_rot) in izip!(0.., auto.forward()) {
                                            if sym_data
                                                .safe
                                                .robber_safe_at(sym_robber_range.at(v_rot))
                                                != all_data
                                                    .safe
                                                    .robber_safe_at(all_robber_range.at(v))
                                            {
                                                sym_data.validation = WinValidation::Error(format!(
                                                    "Fehler: Konfig {:?} uneinig in Knoten {} (rotiert = {})",
                                                    cop_config,
                                                    v,
                                                    v_rot
                                                ));
                                            }
                                        }
                                    }
                                }
                                if sym_data.validation == WinValidation::SymmetryOnly {
                                    sym_data.validation = WinValidation::Both;
                                }
                                sym_res
                            };
                            self.employ_worker(work, WorkTask::Verify);
                        }
                    }
                }
            } else {
                self.worker = Some((handle, curr_task));
            }
        }
    }

    fn start_computation(&mut self, nr_cops: usize, map: &map::Map) {
        if self.current_worker_task().is_some() {
            return;
        }
        if let SymGroup::Explicit(equiv) = map.data().sym_group() {
            let sym = SymmetricMap {
                shape: map.shape(),
                edges: map.edges().clone(),
                symmetry: equiv.clone(),
            };
            let compute = move || compute_safe_robber_positions(nr_cops, sym);
            self.employ_worker(compute, WorkTask::Compute);
        } else {
            let sym = SymmetricMap {
                shape: map.shape(),
                edges: map.edges().clone(),
                symmetry: NoSymmetry::new(map.data().nr_vertices()),
            };
            let compute = move || compute_safe_robber_positions(nr_cops, sym).into_explicit();
            self.employ_worker(compute, WorkTask::Compute);
        }
    }

    fn save_result_as(&mut self, path: &Path) {
        if self.current_worker_task().is_some() {
            return;
        }
        let Ok(file) = fs::File::create(path) else {
            return;
        };
        let res = std::mem::take(&mut self.result);
        let work = move || {
            let mut file_buffer = std::io::BufWriter::new(file);
            rmp_serde::encode::write(&mut file_buffer, &res).ok();
            file_buffer.flush().ok();
            res
        };
        self.employ_worker(work, WorkTask::Store);
    }

    fn load_result_from(&mut self, path: &Path) {
        if self.current_worker_task().is_some() {
            return;
        }
        let Ok(file) = fs::File::open(path) else {
            return;
        };
        let work = move || {
            let buff_reader = std::io::BufReader::new(file);
            let Ok(res) = rmp_serde::decode::from_read(buff_reader) else {
                return BruteForceResult::Error("Fehler beim decoden".to_string());
            };
            res
        };
        self.employ_worker(work, WorkTask::Load);
    }

    fn draw_result(&self, ui: &mut Ui) {
        let write_cops = |nr_cops: usize| {
            if nr_cops == 1 {
                "einen Cop".to_owned()
            } else {
                nr_cops.to_string() + " Cops"
            }
        };
        match &self.result {
            BruteForceResult::None => ui.label("Noch keine beendete Rechnung"),
            BruteForceResult::Error(what) => {
                ui.label("Fehler bei letzter Rechnung: \n".to_owned() + what)
            },
            BruteForceResult::CopsWin(nr_cops, nr_vertices, _) => ui.label(format!(
                "Räuber verliert gegen {} auf {} Knoten",
                write_cops(*nr_cops),
                nr_vertices
            )),
            BruteForceResult::RobberWins(data) => ui.label(format!(
                "Räuber gewinnt gegen {} auf {} Knoten\n{}",
                write_cops(data.nr_cops),
                data.nr_map_vertices(),
                match &data.validation {
                    WinValidation::NoSymmetry => "(Algo ohne Symmetrie)".to_string(),
                    WinValidation::SymmetryOnly => "(Algo mit Symmetrie)".to_string(),
                    WinValidation::Both => "(Algos mit und ohne Symmetrie)".to_string(),
                    WinValidation::Error(what) => format!("Fehler: {}", what),
                }
            )),
        };
    }

    fn file_name_of(nr_cops: usize, nr_vertices: usize, shape: map::Shape) -> PathBuf {
        PathBuf::from(format!(
            "bruteforce/{}-{}-{}.msgpack",
            nr_cops,
            shape.to_str(),
            nr_vertices
        ))
    }

    pub fn draw_menu(&mut self, nr_cops: usize, ui: &mut Ui, map: &map::Map) {
        ui.collapsing("Bruteforce", |ui| {
            if let Some((_, curr_task)) = &self.worker {
                ui.horizontal(|ui| {
                    ui.label(match curr_task {
                        WorkTask::Compute => "rechne ",
                        WorkTask::Load => "lade ",
                        WorkTask::Store => "speichere ",
                        WorkTask::ComputeVerification => "berechne ohne Sym. ",
                        WorkTask::Verify => "verifiziere ",
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
            } else if NATIVE
                && !matches!(
                    self.result,
                    BruteForceResult::None | BruteForceResult::Error(_)
                )
                && ui.button("speichere Daten").clicked()
            {
                let (nr_cops, nr_vertices, shape) = match &self.result {
                    BruteForceResult::CopsWin(nr_cops, nr_vertices, shape) => {
                        (*nr_cops, *nr_vertices, *shape)
                    },
                    BruteForceResult::RobberWins(data) => {
                        (data.nr_cops, data.nr_map_vertices(), data.shape())
                    },
                    _ => panic!(),
                };
                let path = Self::file_name_of(nr_cops, nr_vertices, shape);
                self.save_result_as(&path);
            } else if NATIVE && ui.button("lade Daten").clicked() {
                let path = Self::file_name_of(nr_cops, map.data().nr_vertices(), map.shape());
                self.load_result_from(&path);
            }

            self.draw_result(ui);
        });
    }
}
