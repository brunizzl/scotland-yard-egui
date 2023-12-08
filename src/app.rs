use egui::*;


mod dim2;
mod dim3;
mod cop_robber_algos;


pub const GREY: Color32 = Color32::from_rgb(130, 130, 150);
pub const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
pub const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
pub const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
pub const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
pub const RED: Color32 = Color32::from_rgb(230, 50, 50);
 
pub const COP_BLUE: Color32 = Color32::from_rgb(10, 50, 170);
pub const ROBBER_RED: Color32 = Color32::from_rgb(170, 40, 40);
pub const BLUE_GLOW: Color32 = Color32::from_rgb(60, 120, 235);
pub const RED_GLOW: Color32 = Color32::from_rgb(235, 120, 120);

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

#[derive(Clone, Copy)]
struct Camera2D {
    /// == 1.0 -> no change
    /// < 1.0  -> zoomed out
    /// > 1.0  -> zoomed in
    zoom: f32, 
    /// offset of center independent of zoom
    offset: Vec2,
}

impl Camera2D {
    fn new() -> Self {
        Self { 
            zoom: 1.0, 
            offset: Vec2::new(0.0, 0.0), 
        }
    }

    fn update(&mut self, ui: &mut Ui, response: Option<&Response>) {        
        ui.input(|info| {
            if info.pointer.button_down(PointerButton::Secondary) {
                self.offset += info.pointer.delta();
            }
            self.offset += info.scroll_delta;

            let zoom_delta = info.zoom_delta();
            self.zoom *= zoom_delta;
            if zoom_delta != 1.0 {
                if let (Some(ptr_pos), Some(resp)) = (info.pointer.latest_pos(), response) {
                    //keep fixed point of zoom at mouse pointer
                    let draw_mid = resp.rect.center();
                    let mid_to_ptr = ptr_pos - draw_mid;
                    let mut zoom_center = self.offset - mid_to_ptr;
                    zoom_center *= zoom_delta;
                    self.offset = zoom_center + mid_to_ptr;
                }
            }
            if let Some(drag) = info.multi_touch() {
                self.offset += drag.translation_delta;
            }
        });
    }

    /// zoom changes happen with the cursor position as fixed point, thus 
    /// with zooming we also change the offset
    fn update_cursor_centered(&mut self, ui: &mut Ui, response: &Response) {
        self.update(ui, Some(response));
    }

    /// zoom changes don't change the offset at all
    fn update_screen_centered(&mut self, ui: &mut Ui) {
        self.update(ui, None);
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
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
                widgets::global_dark_light_mode_buttons(ui);
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