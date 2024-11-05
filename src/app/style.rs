use itertools::izip;

use egui::{Color32, Ui};

#[derive(Clone, Copy, PartialEq, Debug, serde::Deserialize, serde::Serialize)]
pub struct Style<const N: usize>
where
    [Color32; N]: for<'a> serde::Deserialize<'a> + serde::Serialize,
{
    pub colors: [Color32; N],
    pub size: f32,
}

use std::sync::atomic::{AtomicUsize, Ordering};
/// the color picker is not working out of a menu button, thus we build our own menu button with a window
/// placed at the right spot. because only up to a single such window should be open at the same time,
/// we keep track at which adress this style is placed. if no options menu is open, this address is 0.
static OPEN_MENU: AtomicUsize = AtomicUsize::new(0);

pub fn close_options_menu() {
    OPEN_MENU.store(0, Ordering::Release);
}

pub const FIFTH_ROOT_OF_TWO: f64 = 1.148_698_354_997_035;

impl<const N: usize> Style<N>
where
    [Color32; N]: for<'a> serde::Deserialize<'a> + serde::Serialize,
{
    pub const fn new(colors: &[Color32; N]) -> Self {
        Self { colors: *colors, size: 1.0 }
    }

    pub fn draw_options(&mut self, ui: &mut Ui, default_colors: &[Color32; N]) {
        {
            let curr_open = OPEN_MENU.load(Ordering::Acquire);
            let this = self as *const _ as usize;
            let this_is_open = curr_open == this;
            if ui.button(" ⛭ ").clicked() {
                let new_val = if this_is_open { 0 } else { this };
                OPEN_MENU.store(new_val, Ordering::Release);
                return;
            }
            if !this_is_open {
                return;
            }
        }
        let window_id = egui::Id::from("Style Options");
        egui::Area::new(window_id)
            .fixed_pos(ui.cursor().left_top())
            .show(ui.ctx(), |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    for (color, default) in izip!(&mut self.colors, default_colors) {
                        ui.horizontal(|ui| {
                            ui.label("Farbe: ");
                            ui.color_edit_button_srgba(color);
                            if ui.button("Reset").clicked() {
                                *color = *default;
                            }
                        });
                    }
                    ui.horizontal(|ui| {
                        ui.label("Größe: ");
                        super::add_drag_value(
                            ui,
                            &mut self.size,
                            "",
                            (0.125, 8.0),
                            FIFTH_ROOT_OF_TWO,
                        );
                    });
                })
            });

        let window_layer_id = egui::LayerId::new(egui::Order::Middle, window_id);
        ui.ctx().move_to_top(window_layer_id);

        //TODO: somehow figure out when to close popup,
        //because an interaction happened somewhere else
    }
}
