use std::{
    fs,
    io::{Read, Write},
};

use chrono::{SubsecRound, Timelike};
use itertools::{Itertools, izip};

use egui::{Ui, vec2};
use serde::{Deserialize, Serialize};

use super::{Camera3D, NATIVE, character, info, load_or, manual_markers::ManualMarkers, map};

#[derive(Deserialize, Serialize)]
struct SavedState {
    name: String,
    saved_at: std::time::SystemTime,
    shape: crate::graph::Shape,
    resolution: isize,
    characters: character::State,
    manual_markers: Vec<(usize, u8)>, //run length encoding of info's manual markers
}

impl SavedState {
    fn set(&mut self, map: &mut map::Map, info: &mut info::Info, cam: &mut Camera3D) {
        map.change_to(self.shape.clone(), self.resolution);
        info.characters = self.characters.clone_without_distances();
        info.manual_markers = ManualMarkers::new_init(crate::rle::decode(&self.manual_markers));
        info.adjust_to_new_map(map.data());
        cam.adjust_to_new_map(map.shape());
    }

    fn path(&self) -> std::path::PathBuf {
        let saved_at = chrono::DateTime::<chrono::Local>::from(self.saved_at);
        let date = saved_at.date_naive();
        let secs = saved_at.time().num_seconds_from_midnight();
        std::path::PathBuf::from(format!(
            "saves/{}-{}-{date}-{secs}.ron",
            self.shape.to_sting(),
            self.resolution,
        ))
    }

    fn store_in_filesystem(&self) {
        if !NATIVE {
            return;
        }
        let store = || -> Result<(), ron::Error> {
            let config = ron::ser::PrettyConfig::new()
                .compact_arrays(true)
                .new_line("\n".into())
                .indentor("\t".into());
            let as_str = ron::ser::to_string_pretty(self, config)?;

            let mut file = fs::File::create(self.path())?;
            file.write_all(as_str.as_bytes())?;
            Ok(())
        };
        if let Err(e) = store() {
            log::debug!("Fehler beim speichern von Speicherstand: {e}");
        }
    }

    fn remove_from_filesystem(&self) {
        if !NATIVE {
            return;
        }
        if let Err(e) = std::fs::remove_file(self.path()) {
            log::debug!("Fehler beim löschen von Speicherstand: {e}");
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, strum_macros::EnumIter)]
enum OrdBy {
    Name,
    Date,
    Shape,
    Res,
    NrCops,
}

impl OrdBy {
    const fn name(self) -> &'static str {
        match self {
            Self::Name => "Name",
            Self::Date => "Datum",
            Self::Shape => "Form",
            Self::Res => "Aufl.",
            Self::NrCops => "Nr Cops",
        }
    }
}

pub struct SavedStates {
    new_name: String,
    saves: Vec<SavedState>,
    deleted: Option<SavedState>,
    active: Option<usize>,
    ord: OrdBy,
    reverse_ord: bool,
}

impl SavedStates {
    fn load_from_filesystem() -> Vec<SavedState> {
        let mut res = Vec::new();
        let Ok(path) = fs::read_dir("./saves/") else {
            return res;
        };
        let try_load = |entry: Result<fs::DirEntry, _>| -> ron::Result<SavedState> {
            let entry = entry?;
            let mut file = fs::File::open(entry.path())?;
            let mut s = String::new();
            let file_str = file.read_to_string(&mut s).map(|_| s)?;
            ron::from_str(&file_str).map_err(|e| e.code)
        };
        for entry in path {
            match try_load(entry) {
                Ok(safe) => res.push(safe),
                Err(e) => log::debug!("Fehler beim laden von Speicherstand: {e}"),
            }
        }
        res
    }

    const SAVES_STORAGE_KEY: &'static str = "app::save-states";
    const ORDER_STORAGE_KEY: &'static str = "app::save-states::order";

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let saves = if NATIVE {
            Self::load_from_filesystem()
        } else {
            load_or(cc.storage, Self::SAVES_STORAGE_KEY, Vec::new)
        };
        let (active, ord) = load_or(cc.storage, Self::ORDER_STORAGE_KEY, || (None, OrdBy::Name));
        let mut result = Self {
            new_name: String::new(),
            saves,
            deleted: None,
            active,
            ord,
            reverse_ord: false,
        };
        result.sort();
        result
    }

    fn sort(&mut self) {
        let mut with_old_positions = izip!(self.saves.drain(..), 0..).collect_vec();
        with_old_positions.sort_by(|(a, _), (b, _)| match self.ord {
            OrdBy::Name => a.name.cmp(&b.name),
            OrdBy::Date => a.saved_at.cmp(&b.saved_at),
            OrdBy::Shape => a.shape.cmp(&b.shape),
            OrdBy::Res => a.resolution.cmp(&b.resolution),
            OrdBy::NrCops => (a.characters.all().len()).cmp(&b.characters.all().len()),
        });
        if self.reverse_ord {
            with_old_positions.reverse();
        }
        if let Some(i) = self.active {
            self.active = with_old_positions.iter().position(|(_, j)| i == *j);
        }
        self.saves.extend(with_old_positions.into_iter().map(|(s, _)| s));
    }

    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if let Some(s) = &self.deleted {
            s.remove_from_filesystem()
        }
        // in native, we save each entry at the moment of creation and as their own file,
        // thus it is then not required to duplicate that info in the general app data
        if !NATIVE {
            eframe::set_value(storage, Self::SAVES_STORAGE_KEY, &self.saves);
        }
        eframe::set_value(storage, Self::ORDER_STORAGE_KEY, &(self.active, self.ord));
    }

    fn delete(&mut self, i: usize) {
        if let Some(s) = &self.deleted {
            s.remove_from_filesystem();
        }
        self.deleted = Some(self.saves.remove(i));
        match &mut self.active {
            Some(a) if *a == i => self.active = None,
            Some(a) if *a > i => *a -= 1,
            _ => {},
        }
    }

    fn add(&mut self, map: &map::Map, info: &info::Info) {
        let name = if self.new_name.is_empty() {
            "unbenannt".to_string()
        } else {
            self.new_name.clone()
        };
        let saved_at = if NATIVE {
            std::time::SystemTime::now()
        } else {
            std::time::SystemTime::UNIX_EPOCH
        };
        let map_shape = map.shape().clone();
        let map_resolution = map.resolution() as isize;
        let characters = info.characters.clone_without_distances();
        let manual_markers = crate::rle::encode(info.manual_markers.curr());
        let new_save = SavedState {
            name,
            saved_at,
            shape: map_shape,
            resolution: map_resolution,
            characters,
            manual_markers,
        };
        new_save.store_in_filesystem();
        self.active = Some(self.saves.len());
        self.saves.push(new_save);
        self.sort();
    }

    pub fn update(
        &mut self,
        ui: &mut Ui,
        map: &mut map::Map,
        info: &mut info::Info,
        cam: &mut Camera3D,
    ) {
        if let Some(save) = self.active.and_then(|i| self.saves.get(i)) {
            let same_shape = &save.shape == map.shape();
            let same_res = save.resolution == map.resolution() as isize;
            if !same_shape || !same_res {
                self.active = None;
            }
        }

        fn highlight(button: egui::Response, highlight: bool) -> egui::Response {
            if highlight {
                button.highlight()
            } else {
                button
            }
        }

        crate::app::menu_button_closing_outside(ui, " 🖴  laden / speichern", |ui| {
            let text_galleys = {
                let max_width = ui.content_rect().width() * 0.9;
                self.saves
                    .iter()
                    .map(|s| {
                        use egui::{RichText, TextStyle, WidgetText};
                        let time = if NATIVE {
                            let saved_at = chrono::DateTime::<chrono::Local>::from(s.saved_at);
                            let date = saved_at.date_naive();
                            let time = saved_at.time().round_subsecs(0);
                            format!(", {date} um {time}")
                        } else {
                            String::new()
                        };
                        let text = {
                            let shape = s.shape.emoji();
                            let res = s.resolution;
                            let name = s.name.as_str();
                            let nr_chars = s.characters.all().len();
                            format!("{name}\n{nr_chars:2} Figuren auf {shape} ({res}){time}")
                        };
                        let widget = WidgetText::RichText(RichText::new(text).into());
                        widget.into_galley(
                            ui,
                            Some(egui::TextWrapMode::Extend),
                            max_width,
                            TextStyle::Body,
                        )
                    })
                    .collect_vec()
            };
            {
                let max_name_len = text_galleys.iter().map(|g| g.rect.width()).fold(0.0, f32::max);
                //assume button for delete fits in 50 units.
                let space = max_name_len + 50.0;
                ui.allocate_at_least(vec2(space, 0.0), egui::Sense::hover());
            }

            ui.horizontal(|ui| {
                ui.label("speichern als: ");
                ui.text_edit_singleline(&mut self.new_name);
                if ui.button("Ok").clicked() {
                    self.add(map, info);
                }
            });

            ui.separator();
            ui.horizontal(|ui| {
                use strum::IntoEnumIterator;
                ui.label("sortieren:");
                for ord in OrdBy::iter() {
                    if !NATIVE && ord == OrdBy::Date {
                        //date is scuffed in wasm -> skip
                        continue;
                    }
                    if highlight(ui.button(ord.name()), self.ord == ord).clicked() {
                        self.ord = ord;
                        self.sort();
                    }
                }

                ui.add_space(8.0);
                let (reversed_symbol, reversed_info) = match self.reverse_ord {
                    false => ("▪◾◼", "Aktuell: Aufsteigend"),
                    true => ("◼◾▪", "Aktuell: Absteigend"),
                };
                if ui.button(reversed_symbol).on_hover_text(reversed_info).clicked() {
                    self.reverse_ord ^= true;
                    self.sort();
                }
            });

            ui.separator();
            if let Some(del) = &self.deleted {
                let name = del.name.clone();
                if ui.button(format!("\"{name}\" wiederherstellen")).clicked() {
                    self.saves.push(self.deleted.take().unwrap());
                    self.sort();
                }
            }
            egui::ScrollArea::vertical().show(ui, |ui| {
                for (i, text) in izip!(0.., text_galleys) {
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        let button = highlight(ui.button(text), Some(i) == self.active);
                        if button.on_hover_text("laden").clicked() {
                            self.saves[i].set(map, info, cam);
                            self.active = Some(i);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button(" 🗑 ").on_hover_text("löschen").clicked() {
                                self.delete(i);
                            }
                        });
                    });
                }
            });
        });
    }
}
