use std::{collections::HashMap, fs::File, io::Write};

use egui::{
    emath::RectTransform,
    epaint::{ColorMode, TextShape},
    *,
};

use super::tikz::to_unique_str;
use crate::geo;

struct CetzPicture {
    file_name: std::path::PathBuf,
    header: String,
    content: String,
    color_names: HashMap<[u8; 4], String>,
    to_cetz: RectTransform,
    curr_stroke: egui::Stroke,
    curr_fill: Color32,
    border: geo::BoundedRect,
}

impl CetzPicture {
    fn color_name(&mut self, color: Color32) -> String {
        let color_array = color.to_array();
        if let Some(name) = self.color_names.get(&color_array) {
            return name.clone();
        }
        let identifier = to_unique_str(self.color_names.len() + 1);
        let new_name = format!("c{identifier}");
        self.color_names.insert(color_array, new_name.clone());
        new_name
    }

    fn add_command(&mut self, line: &str) {
        self.content.push_str("    ");
        self.content.push_str(line);
        self.content.push('\n');
    }

    fn coord_scale(&self) -> f32 {
        let scale = self.to_cetz.scale();
        debug_assert!((scale.x - scale.y).abs() / scale.x < 1e-4);
        scale.x
    }

    fn width_scale(&self) -> f32 {
        let coord_scale = self.coord_scale();
        coord_scale * 30.0
    }

    /// `clip_rect` is assumed to be in egui's screen coordinates.
    /// egui has the origin in the left upper corner with the y-axis pointing down.
    /// tikz has the origin in the left lower corner with the y-axis pointing up.
    /// we use `clip_rect` to find the transformation to get us from egui to tikz coordinates.
    fn new(file_name: std::path::PathBuf, header: String, clip_rect: Rect) -> Self {
        let is_comment = |line: &str| {
            let trimmed = line.trim();
            trimmed.starts_with("//") || trimmed.is_empty()
        };
        assert!(header.split('\n').all(is_comment));
        let tikz_rect = {
            let x_range = Rangef::new(0.0, 8.0);
            let y_range = Rangef::new(0.0, 8.0 / clip_rect.aspect_ratio());
            Rect::from_x_y_ranges(x_range, y_range)
        };
        let border = geo::BoundedRect::from_rect(clip_rect);
        Self {
            file_name,
            header,
            content: String::new(),
            color_names: HashMap::new(),
            to_cetz: RectTransform::from_to(clip_rect, tikz_rect),
            curr_stroke: Stroke::NONE,
            curr_fill: Color32::TRANSPARENT,
            border,
        }
    }

    fn update_stroke(&mut self, width: f32, color: Color32) {
        let new = egui::Stroke::new(width, color);
        if new == self.curr_stroke {
            return;
        }
        self.curr_stroke = new;
        if width == 0.0 || color.a() == 0 {
            self.add_command("stroke(none);");
        } else {
            let color = self.color_name(color);
            let width = width * self.width_scale();
            self.add_command(&format!("stroke({width:.6}pt + {color});"));
        }
    }

    fn update_fill(&mut self, fill: Color32, next_shape: Option<&Shape>, command: &mut String) {
        let fill_of = |s: &Shape| -> Option<Color32> {
            match s {
                Shape::Circle(c) => Some(c.fill),
                Shape::Path(p) => Some(p.fill),
                Shape::Text(t) => Some(t.fallback_color),
                _ => None,
            }
        };
        if fill == self.curr_fill {
            return;
        }
        let fill_name = self.color_name(fill);
        if next_shape.is_none_or(|s2| fill_of(s2) != Some(fill)) {
            command.push_str(&format!(", fill: {fill_name}"));
        } else {
            self.curr_fill = fill;
            self.add_command(&format!("fill({fill_name});"));
        }
    }

    fn add_shapes(&mut self, shapes: &[Shape], text_shift: egui::Vec2) {
        let coord_scale = self.coord_scale();
        let text_scale = |t: &TextShape| {
            let font_size = t.galley.job.sections[0].format.font_id.size;
            font_size * coord_scale * 22.0
        };

        for (i, shape) in shapes.iter().enumerate() {
            match shape {
                Shape::Circle(c) => {
                    self.update_stroke(c.stroke.width, c.stroke.color);
                    let mut command = {
                        let Pos2 { x, y } = self.to_cetz.transform_pos(c.center);
                        let r = c.radius * coord_scale;
                        format!("circle(({x:+.6}, {y:+.6}), radius: {r:.6}")
                    };
                    self.update_fill(c.fill, shapes.get(i + 1), &mut command);
                    command += ");";
                    self.add_command(&command);
                },
                Shape::LineSegment { points, stroke } => {
                    let points = geo::line_from_to(points[0], points[1]);
                    if let Some(points) = self.border.trim(points) {
                        let points: [_; 2] = points.into();
                        let Pos2 { x: ax, y: ay } = self.to_cetz.transform_pos(points[0]);
                        let Pos2 { x: bx, y: by } = self.to_cetz.transform_pos(points[1]);

                        self.update_stroke(stroke.width, stroke.color);
                        self.add_command(&format!(
                            "line(({ax:+.6},{ay:+.6}), ({bx:+.6},{by:+.6}));",
                        ));
                    }
                },
                Shape::Text(t) => {
                    let Pos2 { x, y } = self.to_cetz.transform_pos(t.pos - text_shift);
                    let scale = text_scale(t);
                    let mut command =
                        format!("content(({x:+.6}, {y:+.6}), text(size: {scale:.6}pt");
                    self.update_fill(t.fallback_color, shapes.get(i + 1), &mut command);
                    command += &format!(")[{}]);", t.galley.job.text);
                    self.add_command(&command);
                },
                Shape::Path(ps) => {
                    let mut command = String::from("line(");

                    let mut sep = "";
                    for &point in &ps.points {
                        let Pos2 { x, y } = self.to_cetz.transform_pos(point);
                        command += &format!("{sep}({x:+.6}, {y:+.6})");
                        sep = ", ";
                    }

                    if let ColorMode::Solid(color) = ps.stroke.color {
                        self.update_stroke(ps.stroke.width, color);
                    } else if ps.closed {
                        self.update_stroke(0.0, Color32::TRANSPARENT);
                    }

                    self.update_fill(ps.fill, shapes.get(i + 1), &mut command);
                    if ps.closed {
                        command += ", closed: true";
                    }
                    command += ");";
                    self.add_command(&command);
                },
                _ => {},
            }
        }
    }
}

impl Drop for CetzPicture {
    fn drop(&mut self) {
        let mut file = match File::create(&self.file_name) {
            Ok(f) => f,
            Err(e) => {
                println!("{e}");
                return;
            },
        };
        let mut write = |data: &str| file.write_all(data.as_bytes()).ok();
        write(&self.header);
        // note: this requires installation of the correct font on your system.
        // the font is found within the egui / eframe source code on github.
        write("#set text(font: \"Noto Emoji\");\n");
        write("\n#import \"@preview/cetz:0.3.2\";\n");
        write("#cetz.canvas({\n");
        write("    import cetz.draw: *;\n");
        for (&[r, g, b, a], name) in &self.color_names {
            write(&format!(
                "    let {name} = color.rgb({r}, {g}, {b}, {a});\n"
            ));
        }
        write("    scale(y: -100%);\n");
        write(&self.content);
        write("})");
    }
}

/// order sub-slices of same color by shape.
/// this can decrease the line count of the resulting picture,
/// as otherwise stroke commands need to be interspersed more often.
/// at the same time, the result is not visually altered,
/// as shapes of same color can be drawn in any order.
/// (this is somewhat false, as only stroke and not fill is considered)
fn sort_color_slices(shapes: &mut [Shape]) {
    // note: as sort is stable, no two shapes of different colors are mixed up,
    // as long as this function and the key function below cover the same cases.
    // note further, how only stroke color matters,
    // as only change in stroke generates an extra stroke command.
    let color_of = |s: &Shape| -> Color32 {
        match s {
            Shape::Circle(c) => c.stroke.color,
            Shape::LineSegment { points: _, stroke } => stroke.color,
            Shape::Text(t) => t.fallback_color,
            Shape::Path(p) => {
                if p.stroke.is_empty() {
                    p.fill
                } else if let ColorMode::Solid(color) = p.stroke.color {
                    color
                } else {
                    Color32::TRANSPARENT
                }
            },
            _ => Color32::TRANSPARENT,
        }
    };
    let mut i = 0;
    while i < shapes.len() {
        let i_color = color_of(&shapes[i]);
        let mut j = i + 1;
        while j < shapes.len() && color_of(&shapes[j]) == i_color {
            j += 1;
        }
        let same_color_slice = &mut shapes[i..j];
        // make sure, to cover the same cases, as the color_of function above!
        same_color_slice.sort_by_key(|s: &Shape| match s {
            Shape::Circle(_) => 0,
            Shape::LineSegment { points: _, stroke: _ } => 1,
            Shape::Text(_) => 2,
            Shape::Path(_) => 3,
            _ => 4,
        });
        i = j;
    }
}

pub fn draw_to_file(
    file_name: std::path::PathBuf,
    header: String,
    content: &egui::Painter,
    clip: Rect,
    text_shift: egui::Vec2,
) {
    let mut pic = CetzPicture::new(file_name, header, clip);
    let mut all_visible_shapes = Vec::new();
    content.for_each_shape(|clipped| {
        let bounding_box = clipped.shape.visual_bounding_rect();
        if clip.intersects(bounding_box) {
            all_visible_shapes.push(clipped.shape.clone());
        }
    });
    sort_color_slices(&mut all_visible_shapes);
    pic.add_shapes(&all_visible_shapes, text_shift);
}
