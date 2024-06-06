use std::{
    fs,
    io::{Read, Write},
};

use chrono::{SubsecRound, Timelike};
use itertools::{izip, Itertools};

use egui::{vec2, Ui};

use super::{character, info, load_or, map, NATIVE};

#[derive(serde::Deserialize, serde::Serialize)]
struct SavedState {
    name: String,
    saved_at: std::time::SystemTime,
    shape: map::Shape,
    resolution: isize,
    characters: character::State,
    manual_markers: Vec<(usize, u8)>, //run length encoding of info's manual markers
}

impl SavedState {
    fn set(&mut self, map: &mut map::Map, info: &mut info::Info) {
        map.change_to(self.shape, self.resolution);
        info.characters = self.characters.clone_without_distances();
        info.marked_manually = crate::rle::decode(&self.manual_markers);
        map.adjust_info(info);
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

pub struct SavedStates {
    new_name: String,
    saves: Vec<SavedState>,
    deleted: Option<SavedState>,
    active: Option<usize>,
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
            let file_str = {
                let mut s = String::new();
                file.read_to_string(&mut s).map(|_| s)?
            };
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

    const STORAGE_KEY: &'static str = "app::save-states";

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut saves: Vec<SavedState> = Vec::new();
        if NATIVE {
            saves = Self::load_from_filesystem();
        }
        if saves.is_empty() {
            saves = load_or(cc.storage, Self::STORAGE_KEY, Vec::new);
            for save in &saves {
                save.store_in_filesystem();
            }
        }
        Self {
            new_name: String::new(),
            saves,
            deleted: None,
            active: None,
        }
    }

    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if let Some(s) = &self.deleted {
            s.remove_from_filesystem()
        }
        // in native, we save each entry at the moment of creation and as their own file,
        // thus it is then not required to duplicate that info in the general app data
        if !NATIVE {
            eframe::set_value(storage, Self::STORAGE_KEY, &self.saves);
        }
    }

    fn delete(&mut self, i: usize) {
        if let Some(s) = &self.deleted {
            s.remove_from_filesystem()
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
        let map_shape = map.shape();
        let map_resolution = map.resolution() as isize;
        let characters = info.characters.clone_without_distances();
        let manual_markers = crate::rle::encode(&info.marked_manually);
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
    }

    fn sort_by_key<F, K>(&mut self, mut f: F)
    where
        F: FnMut(&SavedState) -> K,
        K: Ord,
    {
        if let Some(i) = self.active {
            let mut to_sort = izip!(std::mem::take(&mut self.saves), 0..).collect_vec();
            to_sort.sort_by_key(|(s, _)| f(s));
            self.active = Some(to_sort.iter().position(|(_, j)| i == *j).unwrap());
            self.saves.extend(to_sort.into_iter().map(|(s, _)| s));
        } else {
            self.saves.sort_by_key(f);
        }
    }

    pub fn update(&mut self, ui: &mut Ui, map: &mut map::Map, info: &mut info::Info) {
        if let Some(save) = self.active.map(|i| &self.saves[i]) {
            let same_shape = save.shape == map.shape();
            let same_res = save.resolution == map.resolution() as isize;
            if !same_shape || !same_res {
                self.active = None;
            }
        }

        ui.menu_button(" 🖴  laden / speichern", |ui| {
            let text_galleys = {
                let max_width = ui.ctx().available_rect().width() * 0.9;
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
                        let widget = WidgetText::RichText(RichText::new(text));
                        widget.into_galley(ui, Some(false), max_width, TextStyle::Body)
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
                ui.label("sortieren:");
                if ui.button("Name").clicked() {
                    //as i understand, we can't return a non-static reference here :(
                    //rust std library: please fix.
                    self.sort_by_key(|s| s.name.clone());
                }
                if ui.button("Form").clicked() {
                    self.sort_by_key(|s| s.shape);
                }
                if ui.button("Aufl.").clicked() {
                    self.sort_by_key(|s| s.resolution);
                }
                if ui.button("Datum").clicked() {
                    self.sort_by_key(|s| s.saved_at);
                }
                if ui.button("Nr Cops").clicked() {
                    self.sort_by_key(|s| s.characters.all().len());
                }
            });

            ui.separator();
            if self.deleted.is_some() {
                let name = self.deleted.as_ref().unwrap().name.clone();
                if ui.button(format!("\"{name}\" wiederherstellen")).clicked() {
                    self.saves.push(self.deleted.take().unwrap());
                    if NATIVE {
                        self.sort_by_key(|s| s.saved_at);
                    }
                }
            }
            egui::ScrollArea::vertical().show(ui, |ui| {
                let mut delete = None;
                for (i, text) in izip!(0.., text_galleys) {
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        let button = if Some(i) == self.active {
                            ui.button(text).highlight()
                        } else {
                            ui.button(text)
                        };
                        if button.on_hover_text("laden").clicked() {
                            self.saves[i].set(map, info);
                            self.active = Some(i);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button(" 🗑 ").on_hover_text("löschen").clicked() {
                                delete = Some(i);
                            }
                        });
                    });
                }
                if let Some(i) = delete {
                    self.delete(i);
                }
            });
        });
    }
}
