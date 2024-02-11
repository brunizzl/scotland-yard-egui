use std::{collections::BTreeMap, fs::File, io::Write};

use egui::{emath::RectTransform, *};

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

struct TikzPicture {
    file_name: std::path::PathBuf,
    content: String,
    color_names: BTreeMap<[u8; 4], String>,
    to_tikz: RectTransform,
    indentation: String,
}

impl TikzPicture {
    fn color_name(&mut self, color: Color32) -> String {
        let color_array = color.to_array();
        if let Some(name) = self.color_names.get(&color_array) {
            return name.clone();
        }
        let identifier = to_unique_str(self.color_names.len() + 1);
        let new_name = format!("color{}", identifier);
        self.color_names.insert(color_array, new_name.clone());
        new_name
    }

    fn add_indentation(&mut self) {
        self.indentation.push(' ');
        self.indentation.push(' ');
    }

    fn remove_indentation(&mut self) {
        self.indentation.pop();
        self.indentation.pop();
    }

    fn add_command(&mut self, line: &str) {
        self.content.push_str(&self.indentation);
        self.content.push_str(line);
        self.content.push('\n');
    }

    #[allow(dead_code)]
    fn new_scope<'a>(&'a mut self, options: &str) -> Scope<'a> {
        self.add_command(&format!("\\begin{{scope}}[{options}]\n"));
        self.add_indentation();
        Scope { pic: self }
    }

    fn coord_scale(&self) -> f32 {
        let scale = self.to_tikz.scale();
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
    fn new(file_name: std::path::PathBuf, clip_rect: Rect) -> Self {
        let tikz_rect = {
            let x_range = Rangef::new(0.0, 10.0 * clip_rect.aspect_ratio());
            let y_range = Rangef::new(0.0, 10.0);
            Rect::from_x_y_ranges(x_range, y_range)
        };
        let mut res = Self {
            file_name,
            content: String::new(),
            color_names: BTreeMap::new(),
            to_tikz: RectTransform::from_to(clip_rect, tikz_rect),
            indentation: String::new(),
        };
        res.add_indentation();
        res.add_indentation();
        res
    }

    fn add_shape(&mut self, shape: &Shape) {
        let bounding_box = shape.visual_bounding_rect();
        if !self.to_tikz.from().intersects(bounding_box) {
            return;
        }
        match shape {
            Shape::Circle(c) => {
                let mid = self.to_tikz.transform_pos(c.center);
                let r = c.radius * self.coord_scale();
                let thickness = c.stroke.width * self.width_scale();

                let has_fill = c.fill.a() != 0;
                let has_stroke = c.stroke.color.a() != 0 && c.stroke.width != 0.0;
                if has_fill && has_stroke {
                    let stroke_color = self.color_name(c.stroke.color);
                    let fill_color = self.color_name(c.fill);
                    self.add_command(&format!(
                        "\\filldraw[color={stroke_color}, fill={fill_color}, line width={}] ({},{}) circle ({});",
                        thickness, mid.x, mid.y, r
                    ));
                } else if has_fill {
                    let fill_color = self.color_name(c.fill);
                    self.add_command(&format!(
                        "\\fill[{fill_color}] ({},{}) circle ({});",
                        mid.x, mid.y, r
                    ));
                } else if has_stroke {
                    let stroke_color = self.color_name(c.stroke.color);
                    self.add_command(&format!(
                        "\\draw[{stroke_color}, line width={}] ({},{}) circle ({});",
                        thickness, mid.x, mid.y, r
                    ));
                }
            },
            Shape::LineSegment { points, stroke } => {
                let a = self.to_tikz.transform_pos(points[0]);
                let b = self.to_tikz.transform_pos(points[1]);

                let color = self.color_name(stroke.color);
                self.add_command(&format!(
                    "\\draw[color={color}, line width={}] ({},{}) -- ({},{});",
                    stroke.width * self.width_scale(), a.x, a.y, b.x, b.y
                ))
            },
            Shape::Text(_) => {
                //TODO
            },
            _ => {},
        }
    }
}

impl Drop for TikzPicture {
    fn drop(&mut self) {
        let Ok(mut file) = File::create(&self.file_name) else {
            return;
        };
        let mut write = |data: &str| file.write_all(data.as_bytes()).ok();
        write("\\begin{tikzpicture}\n");
        for (&[r, g, b, _], name) in self.color_names.iter() {
            write(&format!("  \\definecolor{{{name}}}{{RGB}}{{{r},{g},{b}}}\n"));
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

struct Scope<'a> {
    pic: &'a mut TikzPicture,
}

impl<'a> Drop for Scope<'a> {
    fn drop(&mut self) {
        self.remove_indentation();
        self.add_command("\\end{scope}\n");
    }
}

impl<'a> std::ops::Deref for Scope<'a> {
    type Target = TikzPicture;
    fn deref(&self) -> &Self::Target {
        self.pic
    }
}

impl<'a> std::ops::DerefMut for Scope<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.pic
    }
}

pub fn draw_to_file(file_name: std::path::PathBuf, content: &egui::Painter, clip: Rect) {
    let mut pic = TikzPicture::new(file_name, clip);
    content.for_each_shape(|clipped| pic.add_shape(&clipped.shape));
}
