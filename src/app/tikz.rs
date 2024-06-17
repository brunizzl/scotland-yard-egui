use std::{collections::HashMap, fs::File, io::Write};

use egui::{emath::RectTransform, epaint::TextShape, *};

use crate::geo;

fn to_unique_str(mut nr: usize) -> String {
    const ALPHABET_LEN: usize = 26;
    const ALPHABET: [char; ALPHABET_LEN] = [
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R',
        'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    ];
    let mut res = String::new();
    while nr != 0 {
        let next_digit = nr % ALPHABET_LEN;
        nr /= ALPHABET_LEN;
        res.push(ALPHABET[next_digit]);
    }
    res
}

type StrMap = HashMap<&'static str, &'static str>;

struct TikzPicture {
    file_name: std::path::PathBuf,
    content: String,
    color_names: HashMap<[u8; 4], String>,
    text_replacements: StrMap,
    to_tikz: RectTransform,
    border: geo::BoundedRect,
}

impl TikzPicture {
    fn color_name(&mut self, color: Color32) -> String {
        let color_array = color.to_array();
        if let Some(name) = self.color_names.get(&color_array) {
            return name.clone();
        }
        let identifier = to_unique_str(self.color_names.len() + 1);
        let new_name = format!("c{}", identifier);
        self.color_names.insert(color_array, new_name.clone());
        new_name
    }

    fn add_command(&mut self, line: &str) {
        self.content.push_str("    ");
        self.content.push_str(line);
        self.content.push('\n');
    }

    fn coord_scale(&self) -> f32 {
        let scale = self.to_tikz.scale();
        debug_assert!((scale.x - scale.y).abs() / scale.x < 1e-4);
        scale.x
    }

    fn width_scale(&self) -> f32 {
        let coord_scale = self.coord_scale();
        coord_scale * 20.0
    }

    /// `clip_rect` is assumed to be in egui's screen coordinates.
    /// egui has the origin in the left upper corner with the y-axis pointing down.
    /// tikz has the origin in the left lower corner with the y-axis pointing up.
    /// we use `clip_rect` to find the transformation to get us from egui to tikz coordinates.
    fn new(file_name: std::path::PathBuf, clip_rect: Rect, text_replacements: StrMap) -> Self {
        let tikz_rect = {
            let x_range = Rangef::new(0.0, 8.0);
            let y_range = Rangef::new(0.0, 8.0 / clip_rect.aspect_ratio());
            Rect::from_x_y_ranges(x_range, y_range)
        };
        let border = geo::BoundedRect::from_rect(clip_rect);
        Self {
            file_name,
            content: String::new(),
            color_names: HashMap::new(),
            to_tikz: RectTransform::from_to(clip_rect, tikz_rect),
            text_replacements,
            border,
        }
    }

    fn add_shapes(&mut self, shapes: &[Shape]) {
        let coord_scale = self.coord_scale();
        let text_scale = |t: &TextShape| {
            let font_size = t.galley.job.sections[0].format.font_id.size;
            font_size * coord_scale * 2.0
        };
        let mut iter = shapes.iter().peekable();
        while let Some(shape) = iter.next() {
            match shape {
                Shape::Circle(c) => {
                    let mid = self.to_tikz.transform_pos(c.center);
                    let r = c.radius * coord_scale;
                    let thickness = c.stroke.width * self.width_scale();
                    //terrible special case treatment: we know how characters where added and draw character circles
                    //as node with the character symbol in the center.
                    let node_text = 'character_symbol: {
                        if let Some(Shape::Text(t)) = iter.peek() {
                            if t.pos.x == c.center.x {
                                let original_text: &str = &t.galley.job.text;
                                let color = self.color_name(t.fallback_color);
                                let scale = text_scale(t);
                                if let Some(text) = self.text_replacements.get(original_text) {
                                    let _ = iter.next(); //destroy peeked value as we already use it here
                                    break 'character_symbol format!(
                                        " node[color={color}, scale={scale}] {{{text}}}"
                                    );
                                }
                            }
                        }
                        String::new()
                    };

                    let has_fill = c.fill.a() != 0;
                    let has_stroke = c.stroke.color.a() != 0 && c.stroke.width != 0.0;
                    if has_fill && has_stroke {
                        let stroke_color = self.color_name(c.stroke.color);
                        let fill_color = self.color_name(c.fill);
                        self.add_command(&format!(
                            "\\filldraw[color={stroke_color}, fill={fill_color}, line width={}] ({},{}) circle ({}){};",
                            thickness, mid.x, mid.y, r, node_text
                        ));
                    } else if has_fill {
                        let fill_color = self.color_name(c.fill);
                        self.add_command(&format!(
                            "\\fill[{fill_color}] ({},{}) circle ({}){};",
                            mid.x, mid.y, r, node_text
                        ));
                    } else if has_stroke {
                        let stroke_color = self.color_name(c.stroke.color);
                        self.add_command(&format!(
                            "\\draw[{stroke_color}, line width={}] ({},{}) circle ({}){};",
                            thickness, mid.x, mid.y, r, node_text
                        ));
                    }
                },
                Shape::LineSegment { points, stroke } => {
                    let points = geo::line_from_to(points[0], points[1]);
                    if let Some(points) = self.border.trim(points) {
                        let points: [_; 2] = points.into();
                        let a = self.to_tikz.transform_pos(points[0]);
                        let b = self.to_tikz.transform_pos(points[1]);

                        let color = self.color_name(stroke.color);
                        self.add_command(&format!(
                            "\\draw[color={color}, line width={}] ({},{}) -- ({},{});",
                            stroke.width * self.width_scale(),
                            a.x,
                            a.y,
                            b.x,
                            b.y
                        ));
                    }
                },
                Shape::Text(t) => {
                    let original_text: &str = &t.galley.job.text;
                    let content = if let Some(tt) = self.text_replacements.get(original_text) {
                        tt
                    } else {
                        original_text
                    };
                    let Pos2 { x, y } = self.to_tikz.transform_pos(t.pos);
                    let scale = text_scale(t);
                    let color = self.color_name(t.fallback_color);
                    self.add_command(&format!(
                        "\\node[color={color}, scale={scale}, anchor=north] at ({x},{y}) {{{content}}};"
                    ));
                },
                _ => {},
            }
        }
    }
}

impl Drop for TikzPicture {
    fn drop(&mut self) {
        let mut file = match File::create(&self.file_name) {
            Ok(f) => f,
            Err(e) => {
                println!("{}", e);
                return;
            },
        };
        let mut write = |data: &str| file.write_all(data.as_bytes()).ok();
        write("\\begin{tikzpicture}\n");
        for (&[r, g, b, _], name) in &self.color_names {
            write(&format!(
                "  \\definecolor{{{name}}}{{RGB}}{{{r},{g},{b}}}\n"
            ));
        }
        write(&format!(
            "  \\begin{{scope}}[yscale=-1,yshift={}]\n",
            self.to_tikz.from().height() * self.coord_scale()
        ));
        write(&self.content);
        write("  \\end{scope}\n");
        write("\\end{tikzpicture}\n");
    }
}

pub fn draw_to_file(
    file_name: std::path::PathBuf,
    content: &egui::Painter,
    clip: Rect,
    replace: StrMap,
) {
    let mut pic = TikzPicture::new(file_name, clip, replace);
    let mut all_visible_shapes = Vec::new();
    content.for_each_shape(|clipped| {
        let bounding_box = clipped.shape.visual_bounding_rect();
        if clip.intersects(bounding_box) {
            all_visible_shapes.push(clipped.shape.clone());
        }
    });
    pic.add_shapes(&all_visible_shapes);
}
