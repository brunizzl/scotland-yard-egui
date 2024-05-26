use chrono::SubsecRound;
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
    manual_markers: Vec<(usize, u8)>, //run length encoding info's manual markers
}

impl SavedState {
    fn load(&mut self, map: &mut map::Map, info: &mut info::Info) {
        map.change_to(self.shape, self.resolution);
        info.characters = self.characters.clone_without_distances();
        info.marked_manually = crate::rle::decode(&self.manual_markers);
        map.adjust_info(info);
    }
}

pub struct SavedStates {
    new_name: String,
    saves: Vec<SavedState>,
    deleted: Option<SavedState>,
}

impl SavedStates {
    const STORAGE_KEY: &'static str = "app::save-states";

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let saves = load_or(cc.storage, Self::STORAGE_KEY, Vec::new);
        Self {
            new_name: String::new(),
            saves,
            deleted: None,
        }
    }

    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, Self::STORAGE_KEY, &self.saves);
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
        let new_safe = SavedState {
            name,
            saved_at,
            shape: map_shape,
            resolution: map_resolution,
            characters,
            manual_markers,
        };
        self.saves.push(new_safe);
    }

    pub fn update(&mut self, ui: &mut Ui, map: &mut map::Map, info: &mut info::Info) {
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
                        let shape = s.shape.emoji();
                        let text = format!("{}\n{} ({}){}", s.name, shape, s.resolution, time);
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
            if self.deleted.is_some() {
                let name = self.deleted.as_ref().unwrap().name.clone();
                if ui.button(format!("\"{name}\" wiederherstellen")).clicked() {
                    self.saves.push(self.deleted.take().unwrap());
                    if NATIVE {
                        self.saves.sort_by_key(|s| s.saved_at);
                    }
                }
            }
            egui::ScrollArea::vertical().show(ui, |ui| {
                let mut delete = None;
                for (i, text) in izip!(0.., text_galleys) {
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui.button(text).on_hover_text("laden").clicked() {
                            self.saves[i].load(map, info);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button(" 🗑 ").on_hover_text("löschen").clicked() {
                                delete = Some(i);
                            }
                        });
                    });
                }
                if let Some(i) = delete {
                    self.deleted = Some(self.saves.remove(i));
                }
            });
        });
    }
}
