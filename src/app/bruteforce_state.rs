
use std::thread;

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

    fn start_computation(&mut self, nr_characters: usize, map: &map::Map) {
        if self.in_computation() {
            return;
        }
        if let Some(equiv) = map.data().equivalence() {
            let sym = SymmetricMap {
                shape: map.shape(),
                edges: map.edges().clone(),
                symmetry: equiv.clone(),
            };
            let compute = move || compute_safe_robber_positions(nr_characters, sym);
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
            let compute = move || compute_safe_robber_positions(nr_characters, sym).to_explicit();
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
                    format!("Räuber verliert gegen {} auf {} Knoten", write_cops(*nr_cops), nr_vertices)),
                BruteForceResult::RobberWins(nr_cops, _, safe, _) => ui.label(
                    format!("Räuber gewinnt gegen {} auf {} Knoten", write_cops(*nr_cops), safe.nr_map_vertices()))
            }
        });
    }
}


