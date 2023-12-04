use egui::*;


mod dim2;
mod dim3;
mod cop_robber_algos;


const GREY: Color32 = Color32::from_rgb(130, 130, 150);
const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
const RED: Color32 = Color32::from_rgb(230, 50, 50);

const COP_BLUE: Color32 = Color32::from_rgb(10, 50, 170);
const ROBBER_RED: Color32 = Color32::from_rgb(170, 40, 40);
const BLUE_GLOW: Color32 = Color32::from_rgb(60, 120, 235);
const RED_GLOW: Color32 = Color32::from_rgb(235, 120, 120);

/// returns if val was changed
fn add_drag_value(ui: &mut Ui, val: &mut usize, name: &str, min: usize, max: usize) -> bool {    
    ui.horizontal(|ui| { 
        let prev = *val;  
        ui.label(name);                     
        if ui.button(" - ").clicked() && prev > min {
            *val -= 1;
        }                     
        ui.add(DragValue::new(val).clamp_range(min..=max));   
        if ui.button(" + ").clicked() && prev < max {
            *val += 1;
        }
        prev != *val
    }).inner
}

pub struct State {
    state_2d: dim2::State,
    state_3d: dim3::State,
    show_2d: bool, //else show 3d
}

impl State {
    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self { 
            state_2d: dim2::State::new(_cc), 
            state_3d: dim3::State::new(_cc), 
            show_2d: true,
        }
    }
}

impl eframe::App for State {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {

        SidePanel::left("left_panel").show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.heading("Optionen");
                let button_switch_text = if self.show_2d { "Zeige 3D" } else { "Zeige 2D" };
                if ui.button(button_switch_text).clicked() {
                    self.show_2d = !self.show_2d;
                }

                if self.show_2d {
                    self.state_2d.draw_menu(ui);
                }
                else {
                    self.state_3d.draw_menu(ui);
                }
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            if self.show_2d {
                self.state_2d.draw_graph(ui);
            }
            else {
                self.state_3d.draw_graph(ui);
            }
        });
    }
}