use chrono::SubsecRound;
use itertools::{izip, Itertools};

use egui::{vec2, Ui};

use super::{character, info, load_or, map};

#[derive(serde::Deserialize, serde::Serialize)]
struct SavedState {
    name: String,
    saved_at: std::time::SystemTime,
    map_shape: map::Shape,
    map_resolution: isize,
    characters: character::State,
    manual_markers: Vec<(usize, u8)>, //run length encoding info's manual markers
}

impl SavedState {
    fn load(&mut self, map: &mut map::Map, info: &mut info::Info) {
        map.change_to(self.map_shape, self.map_resolution);
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
        let saved_at = std::time::SystemTime::now();
        let map_shape = map.shape();
        let map_resolution = map.resolution() as isize;
        let characters = info.characters.clone_without_distances();
        let manual_markers = crate::rle::encode(&info.marked_manually);
        let new_safe = SavedState {
            name: self.new_name.clone(),
            saved_at,
            map_shape,
            map_resolution,
            characters,
            manual_markers,
        };
        self.saves.push(new_safe);
    }

    pub fn update(&mut self, ui: &mut Ui, map: &mut map::Map, info: &mut info::Info) {
        ui.menu_button(" 🖴 laden / speichern", |ui| {
            let text_galleys = {
                let screen_width = ui.ctx().available_rect().width();
                self.saves
                    .iter()
                    .map(|s| {
                        let saved_at = chrono::DateTime::<chrono::Local>::from(s.saved_at);
                        let text = egui::RichText::new(format!(
                            "{}\n{} ({}), {} um {}",
                            s.name,
                            s.map_shape.emoji(),
                            s.map_resolution,
                            saved_at.date_naive(),
                            saved_at.time().round_subsecs(0)
                        ));
                        egui::WidgetText::RichText(text).into_galley(
                            ui,
                            Some(false),
                            screen_width,
                            egui::TextStyle::Body,
                        )
                    })
                    .collect_vec()
            };
            {
                let max_name_len = text_galleys.iter().map(|g| g.rect.width()).fold(0.0, f32::max);
                //assume buttons for delete / load fit in 80 units.
                let space = max_name_len + 80.0;
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
                    if let Some(del) = self.deleted.take() {
                        self.saves.push(del);
                    }
                }
            }
            egui::ScrollArea::vertical().show(ui, |ui| {
                let mut delete = None;
                let mut load = None;

                let old_wrap = ui.style().wrap;
                ui.style_mut().wrap = Some(false);
                for (i, text) in izip!(0.., text_galleys) {
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.label(text);
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button("laden").clicked() {
                                load = Some(i);
                            }
                            if ui.button(" 🗑 ").on_hover_text("löschen").clicked() {
                                delete = Some(i);
                            }
                        });
                    });
                }
                ui.style_mut().wrap = old_wrap;

                if let Some(i) = delete {
                    let del = self.saves.remove(i);
                    self.deleted = Some(del);
                }
                if let Some(i) = load {
                    self.saves[i].load(map, info);
                }
            });
        });
    }
}
