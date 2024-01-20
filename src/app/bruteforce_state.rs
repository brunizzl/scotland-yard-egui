
use std::thread;

use itertools::izip;

use crate::graph::*;
use super::*;

pub struct BruteforceWorker {
    worker: Option<thread::JoinHandle<BruteForceResult<ExplicitClasses>>>,
    result: BruteForceResult<ExplicitClasses>,    
}

impl BruteforceWorker {
    pub fn new() -> Self {
        Self { worker: None, result: BruteForceResult::None }
    }

    pub fn in_computation(&self) -> bool {
        self.worker.is_some()
    }

    pub fn result(&self) -> &BruteForceResult<ExplicitClasses> {
        &self.result
    }

    fn start_computation(&mut self, nr_cops: usize, map: &map::Map) {
        if self.in_computation() {
            return;
        }
        if let Some(equiv) = map.data().equivalence() {
            let sym = SymmetricMap {
                shape: map.shape(),
                edges: map.edges().clone(),
                symmetry: equiv.clone(),
            };
            let no_sym = SymmetricMap {
                shape: map.shape(),
                edges: map.edges().clone(),
                symmetry: NoSymmetry::new(map.data().nr_vertices()),
            };
            let compute = move || {
                let nr_vertices = sym.edges.nr_vertices();
                let mut with_sym = compute_safe_robber_positions(nr_cops, sym);
                if nr_cops > 3 || nr_vertices > 500 {
                    return with_sym;
                }
                let without_sym = compute_safe_robber_positions(nr_cops, no_sym);
                if let (
                    BruteForceResult::RobberWins(sym_data), 
                    BruteForceResult::RobberWins(all_data)
                ) = (&mut with_sym, &without_sym) {
                    for cop_config in all_data.cop_moves.all_positions_unpacked() {
                        let cop_config = &cop_config[..nr_cops];
                        let (_, all_index) = all_data.cop_moves.pack(cop_config.iter().copied());
                        let (autos, sym_index) = sym_data.cop_moves.pack(cop_config.iter().copied());
                        for auto in autos {
                            let sym_robber_range = sym_data.safe.robber_indices_at(sym_index);
                            let all_robber_range = all_data.safe.robber_indices_at(all_index);
                            for (v, v_rot) in izip!(0.., auto.forward()) {
                                if sym_data.safe.robber_safe_at(sym_robber_range.at(v_rot)) 
                                != all_data.safe.robber_safe_at(all_robber_range.at(v)) {
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
                    if let WinValidation::SymmetryOnly = &sym_data.validation {
                        sym_data.validation = WinValidation::Both;
                    }
                }
                with_sym
            };
            //use threads natively
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.worker = Some(thread::spawn(compute));
            }
            //wasm doesn't like threads -> just block gui                 
            #[cfg(target_arch = "wasm32")]
            {
                self.result = compute();
            }
        }
        else {
            let sym = SymmetricMap {
                shape: map.shape(),
                edges: map.edges().clone(),
                symmetry: NoSymmetry::new(map.data().nr_vertices()),
            };
            let compute = move || compute_safe_robber_positions(nr_cops, sym).to_explicit();
            //use threads natively
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.worker = Some(thread::spawn(compute));
            }
            //wasm doesn't like threads -> just block gui                 
            #[cfg(target_arch = "wasm32")]
            {
                self.result = compute();
            }

        }
    }

    pub fn draw_menu(&mut self, nr_characters: usize, ui: &mut Ui, map: &map::Map) {
        ui.collapsing("Bruteforce", |ui|{
            if self.worker.is_some() {
                let float = ui.ctx().animate_value_with_time(
                    Id::new(&self as *const _), 4e20, 1e20);

                ui.label(format!("Berechne Wert {}",
                    match (float as isize) % 8 { 
                        0 => "", 
                        1 => " .", 
                        2 => " . .", 
                        3 => " . . .", 
                        4 => " . . . .", 
                        5 => "   . . .", 
                        6 => "     . .", 
                        _ => "       ."
                    }));
            }
            else if ui.button("Starte Rechnung")
                .on_hover_text("WARNUNG: weil WASM keine Threads mag, blockt \
                die Websiteversion bei dieser Rechnung die GUI.\n\
                Ausserdem: WASM is 32 bit, kann also nur 4GiB RAM benutzen, was die spannenden \
                Bruteforceberechnungen nicht in RAM möglich macht.").clicked() 
            {
                let _ = ui.ctx().animate_value_with_time(
                    Id::new(&self as *const _), 0.0, 0.0);

                self.start_computation(nr_characters, map);
            }
            self.worker = match std::mem::take(&mut self.worker) {
                None => None,
                Some(handle) => if handle.is_finished() {
                    self.result = match handle.join() {
                        Err(_) => BruteForceResult::Error("Programmabsturz".to_owned()),
                        Ok(res) => res,
                    };
                    None
                }
                else {
                    Some(handle)
                }
            };

            let write_cops = |nr_cops: usize| if nr_cops == 1 { 
                "einen Cop".to_owned() 
            } else { 
                nr_cops.to_string() + " Cops" 
            };
            match &self.result {
                BruteForceResult::None => ui.label("Noch keine beendete Rechnung"),
                BruteForceResult::Error(what) => ui.label("Fehler bei letzter Rechnung: \n".to_owned() + what),
                BruteForceResult::CopsWin(nr_cops, nr_vertices, _) => ui.label(
                    format!(
                        "Räuber verliert gegen {} auf {} Knoten", 
                        write_cops(*nr_cops), 
                        nr_vertices
                    )
                ),
                BruteForceResult::RobberWins(data) => ui.label(
                    format!(
                        "Räuber gewinnt gegen {} auf {} Knoten\n{}", 
                        write_cops(data.nr_cops), 
                        data.nr_map_vertices(),
                        match &data.validation {
                            WinValidation::NoSymmetry => "(Algo ohne Symmetrie)".to_string(),
                            WinValidation::SymmetryOnly => "(Algo mit Symmetrie)".to_string(),
                            WinValidation::Both => "(Algos mit und ohne Symmetrie)".to_string(),
                            WinValidation::Error(what) => format!("Fehler: {}", what),
                        }
                    )
                )
            }
        });
    }
}


