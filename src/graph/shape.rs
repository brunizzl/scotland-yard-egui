use std::io::Read;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub enum BuildStep {
    /// distinct vertives of up to given distance become neighbors
    NeighNeighs(usize),
    /// every edge becomes a path with given number of new interior vertices
    SubdivEdges(usize),
    /// vertex at given coordinates.
    /// `.0` is the index. indices cannot have gaps and must be in order, starting from the number of vertices of the base shape.
    /// `.1` are the coordinates in better printable / parseable form.
    /// each coordinate is divided by `1000.0` to get to the corresponding
    /// float value in the graph coordinate system.
    Vertex(usize, [i32; 3]),
    /// delete vertex with this index.
    DeleteVertex(usize),
    /// change the position of the vertex with this index by the given delta
    MoveVertex(usize, [i32; 3]),
    /// connect vertices with these indices
    Edge(usize, usize),
    /// delete edge between vertices with these indices
    DeleteEdge(usize, usize),
    /// adds every Edge between a vertex in `.0` and `.1`.
    CompleteBetween(Box<[usize]>, Box<[usize]>),
    /// adds every edge between consecutive elements
    Path(Box<[usize]>),
    /// if the graph so far has n vertices and m edges, this adds another n + m + 1 vertices and many edges in between.
    /// the idea is that iff the graph before applying this build step is connected,
    /// the graph after applying this build step should be clearable from speed-1 fog by a single visibility-1 cleaner.
    /// at the time of writing this comment, we don't know whether this construction actually works.
    FogTestIsGonnected,
    /// turns the graph build so far into a graph that is (hopefully) clearable from speed-1 fog by a single
    /// visibility-1 cleaner iff the original graph has a hamilton path between the optonally given vertices.
    /// leaving the first argument empty builds everything except the final two edges, thereby leaving this part configurable.
    /// the second argument is the visibility of the cleaner. note that 0 is forbidden, so we bump 0 to 1.
    FogTestHamPath(Option<[usize; 2]>, #[serde(default)] usize),
}

/// operator that separates first and last element of a sequence of consecutive integers.
const SEQUENCE_SEP: &str = "..=";
/// name of [`BuildStep::FogTestIsGonnected`]
const FOG_TEST_IS_CONNECTED_NAME: &str = "ZsgTest";
/// name of [`BuildStep::FogTestHamPath`]
const FOG_TEST_HAM_PATH_NAME: &str = "HamTest";

impl BuildStep {
    pub const EXPLAINER: &str = "\
        N<dist>: <dist>-distance and closer vertices become neighbors\n\
        D<divide>: every edge becomes path with <divide> many inner vertices\n\
        V<index>(<x>,<y>): vertex at coordinates (<x>, <y>) / 1000\n    \
            note that indices must be in order and no index skipped.\n\
        E<u>,<v>: edge between vertices with indices <u> and <v>.\n\
        K(<X>)(<Y>): all edges between sequences <X> and <Y>\n\
        HamTest<u>,<v>: (new is 1-cleanable) <=> (<u>-<v> hamilton path in old)\n\n\
        a sequence has the form <a1>, ..., <an> and is made up of indices.\n\
        ALSO CONSIDER MOUSE TOOLS [±v] AND [±e]\
        ";

    pub const DEFAULT_Z: i32 = (crate::graph::Z_OFFSET_2D * 1000.0) as i32;

    pub fn is_vertex(&self) -> bool {
        matches!(self, Self::Vertex(_, _))
    }
    pub fn is_move(&self) -> bool {
        matches!(self, Self::MoveVertex(_, _))
    }
}

impl std::fmt::Display for BuildStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // writes a sequence, except long subsequences of consecutive entries are written with SEQUENCE_SEP
        let write_sequence = |f: &mut std::fmt::Formatter<'_>, vs: &[usize]| {
            write!(f, "(")?;
            let mut i = 0;
            let mut sep = "";
            while i < vs.len() {
                let x = vs[i];
                let mut y = x;
                let mut streak = 1;
                while i + streak < vs.len() && vs[i + streak] == vs[i] + streak {
                    y += 1;
                    streak += 1;
                }
                // arbitrary decision to not rewrite streaks of just two or three elements
                if streak > 3 {
                    write!(f, "{sep}{x}{SEQUENCE_SEP}{y}")?;
                    i += streak;
                } else {
                    write!(f, "{sep}{x}")?;
                    i += 1;
                }
                sep = ", ";
            }
            write!(f, ")")
        };
        match self {
            Self::NeighNeighs(n) => write!(f, "N{n}"),
            Self::SubdivEdges(n) => write!(f, "D{n}"),
            Self::Vertex(v, [x, y, Self::DEFAULT_Z]) => write!(f, "V{v}({x},{y})"),
            Self::Vertex(v, [x, y, z]) => write!(f, "V{v}({x},{y},{z})"),
            Self::DeleteVertex(v) => write!(f, "DelV{v}"),
            Self::MoveVertex(v, [dx, dy, 0]) => write!(f, "MovV{v}({dx},{dy})"),
            Self::MoveVertex(v, [dx, dy, dz]) => write!(f, "MovV{v}({dx},{dy},{dz})"),
            Self::Edge(v1, v2) => write!(f, "E{v1},{v2}"),
            Self::DeleteEdge(v1, v2) => write!(f, "DelE{v1},{v2}"),
            Self::CompleteBetween(xs, ys) => {
                write!(f, "K")?;
                if xs != ys {
                    write_sequence(f, xs)?;
                }
                write_sequence(f, ys)
            },
            Self::Path(xs) => {
                write!(f, "P")?;
                write_sequence(f, xs)
            },
            Self::FogTestIsGonnected => write!(f, "{FOG_TEST_IS_CONNECTED_NAME}"),
            Self::FogTestHamPath(None, 0) => write!(f, "{FOG_TEST_HAM_PATH_NAME}"),
            Self::FogTestHamPath(None, vis) => write!(f, "{FOG_TEST_HAM_PATH_NAME}({vis})"),
            Self::FogTestHamPath(Some([a, b]), 0) => write!(f, "{FOG_TEST_HAM_PATH_NAME}{a},{b}"),
            Self::FogTestHamPath(Some([a, b]), vis) => {
                write!(f, "{FOG_TEST_HAM_PATH_NAME}{a},{b}({vis})")
            },
        }
    }
}

fn print_steps(steps: &[BuildStep], single_line: bool) -> String {
    use std::fmt::Write;
    let mut res = String::new();
    for step in steps {
        write!(res, "{step}").ok();
        if !single_line {
            writeln!(res).ok();
        }
    }
    res
}

fn parse_steps(mut input: String, size_hint: usize) -> Vec<BuildStep> {
    input.retain(|c| !c.is_ascii_whitespace());
    let mut data: &str = &input;

    fn parse_usize(data: &mut &str) -> Option<usize> {
        let int_end = data.find(|c: char| !c.is_ascii_digit()).unwrap_or(data.len());
        let (int_part, rest) = data.split_at(int_end);
        *data = rest;
        int_part.parse::<usize>().ok()
    }
    fn parse_i32(data: &mut &str) -> Option<i32> {
        let sign = if data.starts_with("-") {
            *data = &data[1..];
            -1
        } else if data.starts_with("+") {
            *data = &data[1..];
            1
        } else {
            1
        };
        let val = parse_usize(data)?;
        Some(sign * i32::try_from(val).ok()?)
    }
    fn remove_single(data: &mut &str, pattern: impl Fn(char) -> bool) {
        if data.chars().next().is_some_and(pattern) {
            let mut first_match = true;
            let is_first = |_| std::mem::replace(&mut first_match, false);
            *data = data.trim_start_matches(is_first);
        }
    }
    fn remove_exact(data: &mut &str, ch: char) {
        remove_single(data, |c| c == ch);
    }
    fn remove_comma(data: &mut &str) {
        remove_exact(data, ',');
    }
    fn parse_sequence(data: &mut &str, sort: bool) -> Box<[usize]> {
        remove_exact(data, '(');
        let mut sequence = Vec::new();
        while let Some(x) = parse_usize(data) {
            if data.starts_with(SEQUENCE_SEP) {
                *data = &data[(SEQUENCE_SEP.len())..];
                if let Some(y) = parse_usize(data)
                    && y >= x
                {
                    sequence.extend(x..=y);
                }
            } else {
                sequence.push(x);
            }
            remove_comma(data);
        }
        remove_exact(data, ')');
        if sort {
            sequence.sort();
            sequence.dedup();
        }
        Box::from(sequence)
    }

    let mut result = Vec::with_capacity(size_hint);
    while !data.is_empty() {
        if data.starts_with(FOG_TEST_HAM_PATH_NAME) {
            data = &data[(FOG_TEST_HAM_PATH_NAME.len())..];
            let ends = parse_usize(&mut data).and_then(|a| {
                remove_comma(&mut data);
                parse_usize(&mut data).map(|b| [a, b])
            });
            remove_exact(&mut data, '(');
            let vis = parse_usize(&mut data).unwrap_or_default();
            remove_exact(&mut data, ')');
            result.push(BuildStep::FogTestHamPath(ends, vis));
        } else if data.starts_with(FOG_TEST_IS_CONNECTED_NAME) {
            data = &data[(FOG_TEST_IS_CONNECTED_NAME.len())..];
            result.push(BuildStep::FogTestIsGonnected);
        } else if data.starts_with("DelV") {
            data = &data["DelV".len()..];
            if let Some(v) = parse_usize(&mut data) {
                result.push(BuildStep::DeleteVertex(v));
            }
        } else if data.starts_with("DelE") {
            data = &data["DelE".len()..];
            let v1 = parse_usize(&mut data).unwrap_or(0);
            remove_comma(&mut data);
            let v2 = parse_usize(&mut data).unwrap_or(0);
            result.push(BuildStep::DeleteEdge(v1, v2));
        } else if data.starts_with("N") {
            data = &data[1..];
            let n = parse_usize(&mut data).unwrap_or(1);
            result.push(BuildStep::NeighNeighs(n));
        } else if data.starts_with("D") {
            data = &data[1..];
            let n = parse_usize(&mut data).unwrap_or(1);
            result.push(BuildStep::SubdivEdges(n));
        } else if data.starts_with("MovV") {
            data = &data["MovV".len()..];
            if let Some(v) = parse_usize(&mut data) {
                remove_exact(&mut data, '(');
                let dx = parse_i32(&mut data).unwrap_or(0);
                remove_comma(&mut data);
                let dy = parse_i32(&mut data).unwrap_or(0);
                remove_comma(&mut data);
                let dz = parse_i32(&mut data).unwrap_or(0);
                remove_exact(&mut data, ')');
                result.push(BuildStep::MoveVertex(v, [dx, dy, dz]));
            }
        } else if data.starts_with("V") {
            data = &data[1..];
            if let Some(v) = parse_usize(&mut data) {
                remove_exact(&mut data, '(');
                let x = parse_i32(&mut data).unwrap_or(0);
                remove_comma(&mut data);
                let y = parse_i32(&mut data).unwrap_or(0);
                remove_comma(&mut data);
                let z = parse_i32(&mut data).unwrap_or(BuildStep::DEFAULT_Z);
                remove_exact(&mut data, ')');
                result.push(BuildStep::Vertex(v, [x, y, z]));
            }
        } else if data.starts_with("E") {
            data = &data[1..];
            let v1 = parse_usize(&mut data).unwrap_or(0);
            remove_comma(&mut data);
            let v2 = parse_usize(&mut data).unwrap_or(0);
            result.push(BuildStep::Edge(v1, v2));
        } else if data.starts_with("K") {
            data = &data[1..];
            let xs = parse_sequence(&mut data, true);
            let mut ys = parse_sequence(&mut data, true);
            if ys.is_empty() {
                ys = xs.clone();
            }
            result.push(BuildStep::CompleteBetween(xs, ys));
        } else if data.starts_with("P") {
            data = &data[1..];
            let xs = parse_sequence(&mut data, false);
            result.push(BuildStep::Path(xs));
        } else {
            // remove the leading character and try again
            remove_single(&mut data, |_| true);
        }
    }
    result
}

/// test if the last two operations are moves of the same vertex. if so, combine them.
pub fn combine_last_move_operations(steps: &mut Vec<BuildStep>) {
    use BuildStep::{MoveVertex, Vertex};
    // snd last may also be the vertex itself, not just a move of an existing vertex.
    while let [.., Vertex(u, du) | MoveVertex(u, du), MoveVertex(v, dv)] = &mut steps[..]
        && u == v
    {
        for i in 0..3 {
            du[i] += dv[i];
        }
        _ = steps.pop();
    }
}

/// tries to combine move operations at the end of the input.
/// note: this makes most sense for [`CustomBuild::build_steps`],
/// but does'nt play nice with the undo and redo operations, hence is currently unused.
#[allow(dead_code)]
fn combine_and_sort_last_move_operations(steps: &mut Vec<BuildStep>) {
    use BuildStep::{MoveVertex, Vertex};

    // the slice at the very end of build_steps containing only move operations
    let mut last_moves = {
        let len = steps.len();
        let nr_last_moves = steps.iter().rev().position(|s| !s.is_move()).unwrap_or(len);
        let last_moves = &mut steps[(len - nr_last_moves)..];
        debug_assert_eq!(last_moves.len(), nr_last_moves);
        debug_assert!(last_moves.iter().all(BuildStep::is_move));
        // sorting allows us to only look at adjacent moves in the loop below
        last_moves.sort();
        last_moves
    };

    // idea: if two move operations belong to the same vertex,
    // combine them in one and replace the other with a do-nothing-move.
    // then remove all do-nothing-moves at once for linear runtime.
    const ZERO_MOVE: BuildStep = MoveVertex(0, [0, 0, 0]);
    while let [MoveVertex(u, du), MoveVertex(v, dv), ..] = last_moves {
        if u == v {
            for i in 0..3 {
                dv[i] += du[i];
            }
            last_moves[0] = ZERO_MOVE;
        }
        last_moves = &mut last_moves[1..];
    }
    steps.retain(|s| s != &ZERO_MOVE);

    // final touch up: if a just-placed vertex is moved, we combine both operations.
    if let [.., Vertex(u, pos), MoveVertex(v, dv)] = &mut steps[..]
        && u == v
    {
        for i in 0..3 {
            pos[i] += dv[i];
        }
        _ = steps.pop();
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CustomBuild {
    pub basis: Shape,
    pub build_steps_string: String,
    pub build_steps: Vec<BuildStep>,
    /// if user currently wields a building tool:
    /// extended whenever user hits [ctrl] + [z],
    /// shrunk whenever user hits [ctrl] + [y].
    #[serde(default)]
    pub future_build_steps: Vec<BuildStep>,
    /// identifier chosen by user.
    #[serde(default)]
    pub name: String,
}

/// the vertex position is irrelevant for the graph theoretic application.
/// this filters out the move operations.
/// note: we don't filter out the [`BuildStep::Vertex`] and [`BuildStep::DeleteVertex`],
/// because they actually matter. not the vertex position, but when it was created / deleted.
fn filter_ord_deciding(steps: &[BuildStep]) -> impl Iterator<Item = &BuildStep> {
    steps.iter().filter(|s| !s.is_move())
}

impl PartialEq for CustomBuild {
    fn eq(&self, other: &Self) -> bool {
        if self.basis != other.basis {
            return false;
        }
        // we don't compare the build_steps_string, because it may have unapplied
        // changes and if not, build_steps should contain the same information.
        let mut self_iter = filter_ord_deciding(&self.build_steps);
        let mut other_iter = filter_ord_deciding(&other.build_steps);
        loop {
            match (self_iter.next(), other_iter.next()) {
                (Some(s1), Some(s2)) => {
                    // the vertex position is irrelevant for the graph theoretic application,
                    // which is what we are interested in. thus: only compare if two build steps are vertices,
                    // not which exact vertices in particular.
                    let steps_match = (s1.is_vertex() && s2.is_vertex()) || s1 == s2;
                    if !steps_match {
                        return false;
                    }
                },
                (Some(_), None) | (None, Some(_)) => return false,
                (None, None) => return true,
            }
        }
    }
}
impl Eq for CustomBuild {}

impl Ord for CustomBuild {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        let basis = self.basis.cmp(&other.basis);

        let mut self_iter = filter_ord_deciding(&self.build_steps);
        let mut other_iter = filter_ord_deciding(&other.build_steps);
        let steps = loop {
            match (self_iter.next(), other_iter.next()) {
                (Some(s1), Some(s2)) => {
                    // see comment in eq implementation above.
                    if s1.is_vertex() && s2.is_vertex() {
                        continue;
                    }
                    let cmp_s12 = s1.cmp(s2);
                    if cmp_s12.is_ne() {
                        break cmp_s12;
                    }
                },
                (Some(_), None) => break std::cmp::Ordering::Greater,
                (None, Some(_)) => break std::cmp::Ordering::Less,
                (None, None) => break std::cmp::Ordering::Equal,
            }
        };

        basis.then(steps)
    }
}
impl PartialOrd for CustomBuild {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl CustomBuild {
    const DEFAULT_NAME: &str = "unnamed";

    pub fn create_new_name() -> String {
        if crate::app::NATIVE {
            use chrono::{DateTime, Local, Timelike};
            let now = DateTime::<Local>::from(std::time::SystemTime::now());
            let date = now.date_naive();
            let secs = now.time().num_seconds_from_midnight();
            format!("{}-{date}-{secs}", Self::DEFAULT_NAME)
        } else {
            Self::DEFAULT_NAME.to_string()
        }
    }

    pub fn new(basis: Shape) -> Self {
        Self {
            basis,
            build_steps_string: String::new(),
            build_steps: Vec::new(),
            future_build_steps: Vec::new(),
            name: Self::create_new_name(),
        }
    }

    pub fn print_build_steps(&self, single_line: bool) -> String {
        print_steps(&self.build_steps, single_line)
    }

    /// returns an identifier. the user must ensure that this is unique.
    pub fn print_name(&self) -> String {
        if self.name != Self::DEFAULT_NAME {
            return self.name.clone();
        }

        let nr_vertices = self.build_steps.iter().filter(|s| s.is_vertex()).count();

        let is_edge = |s: &&_| matches!(s, BuildStep::Edge(_, _));
        let nr_edges = self.build_steps.iter().filter(is_edge).count();

        format!("{}-{nr_vertices}-{nr_edges}", Self::DEFAULT_NAME)
    }

    /// the removal build steps cannot be parsed and should only
    /// be creatable by clicking with the right tool + shift.
    pub fn parse_build_steps(&mut self) {
        let data_string = std::mem::take(&mut self.build_steps_string);
        let size_hint = self.build_steps.len() + 3;
        self.build_steps = parse_steps(data_string, size_hint);
        combine_last_move_operations(&mut self.build_steps);
        self.build_steps_string = self.print_build_steps(false);
    }
}

/// in folder "custom-graphs", one can eighter store a single [`FromFile::class_name`].txt
/// or a folder with name [`FromFile::class_name`] with entries `0.txt`, `1.txt`, `2.txt`, ...
/// the respective files must contain the text representation of [`BuildStep`]'s.
/// if the folder is chosen, the resolution decides which file is parsed.
///
/// note: this struct makes it clear, that the resolution should really just have been a
/// value held by each shape variant where a resolution makes sense.
/// before we change this however, we need to think about how saves
/// keep their file format. we would loose old saves otherwise.
/// note to note: one could in princible manually edit the files. this would be at bit tedious though.
#[derive(Debug, Default, Clone, Deserialize, Serialize)]
pub struct FromFile {
    /// the only thing deciding the _logical entitiy_. e.g. [`Eq`] and [`Ord`] are only decided on this.
    class_name: String,
    /// once this is set after creation, this will never be [`None`].
    #[serde(skip)]
    resulution: Option<usize>,
    #[serde(skip)]
    data: Option<(Vec<BuildStep>, String)>,
    #[serde(skip)]
    build_error: Option<String>,
}

impl PartialEq for FromFile {
    fn eq(&self, other: &Self) -> bool {
        self.class_name.eq(&other.class_name)
    }
}
impl Eq for FromFile {}
impl Ord for FromFile {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.class_name.cmp(&other.class_name)
    }
}
impl PartialOrd for FromFile {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl FromFile {
    pub const FOLDER_NAME: &str = "custom-graphs";

    fn reload(&mut self, res: usize) {
        self.resulution = Some(res);

        let mut file = 'open_file: {
            let folder = Self::FOLDER_NAME;
            let name = self.class_name.as_str();

            let res_path = format!("{folder}/{name}/{res}.txt");
            let res_err = match std::fs::File::open(&res_path) {
                Ok(file) => break 'open_file file,
                Err(err) => err,
            };
            let single_path = format!("{folder}/{name}.txt");
            let single_err = match std::fs::File::open(&single_path) {
                Ok(file) => break 'open_file file,
                Err(err) => err,
            };
            self.build_error = Some(format!(
                "cannot open graph as single file \"{single_path}\":\n{single_err}\n\n\
                    and not with resolution \"{res_path}\":\n{res_err}"
            ));
            return;
        };

        let mut file_content = String::new();
        if let Err(err) = file.read_to_string(&mut file_content) {
            self.build_error = Some(format!("cannot read file:\n{err}"));
            return;
        }
        let size_hint = file_content.len() / 10 + 10;
        let steps = parse_steps(file_content, size_hint);
        let steps_string = print_steps(&steps, false);
        self.data = Some((steps, steps_string));
        self.build_error = None;
    }

    pub fn update(&mut self, ui: &mut egui::Ui, res: usize) -> bool {
        let name_edit = ui.horizontal(|ui| {
            ui.label("Name:");
            use egui::Widget;
            let text_response = egui::TextEdit::singleline(&mut self.class_name)
                .desired_width(ui.available_width() * 0.8)
                .ui(ui);

            if let Some(err) = &self.build_error {
                let boom = egui::RichText::from("💥").color(egui::Color32::RED);
                crate::app::menu_button_closing_outside(ui, boom, |ui| {
                    ui.label(err);
                });
            } else if let Some((_, steps_str)) = &self.data {
                crate::app::menu_button_closing_outside(ui, "☺", |ui| {
                    ui.label("parsed data:");
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        ui.label(steps_str);
                    });
                });
            }
            text_response
        });
        let change = name_edit.inner.changed() || self.resulution.is_none_or(|r| r != res);
        if change {
            self.reload(res);
        }
        change
    }

    pub fn build(mut this: Box<Self>, res: usize) -> super::Embedding3D {
        if this.resulution.is_none_or(|r| r != res) {
            this.reload(res);
        }
        let fallback = |this| {
            let mut single_vertex = super::Embedding3D::new_single_vertex();
            *single_vertex.shape_mut() = Shape::FromFile(this);
            single_vertex
        };

        if let Some((steps, _)) = &this.data {
            let empty = super::Embedding2D::empty();
            let mut result = super::Embedding3D::from_2d(empty, Shape::SingleVertex);
            result.extend_custom(steps);
            if result.nr_vertices() == 0 {
                this.build_error = Some("graph must have at least one vertex".to_string());
                return fallback(this);
            }
            *result.shape_mut() = Shape::FromFile(this);
            return result;
        }
        assert!(this.build_error.is_some());
        fallback(this)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub enum Shape {
    /// only (intendet to be) used as [`CustomBuild::basis`].
    SingleVertex,
    Tetrahedron,
    Octahedron,
    Icosahedron,
    DividedIcosahedron(isize),
    Cube,
    Football,
    FabianHamann,
    Dodecahedron,
    TriangTorus,
    TriangTorusSkewed(isize),
    SquareTorus,
    TriangGrid,
    SquareGrid,
    RegularPolygon2D(isize),
    Random2D(u32),
    /// really plays two roles:
    /// either with basis shape [`Shape::SingleVertex`] as completely custom graph
    /// or with a different shape as extension of this (then non-trivial) base graph.
    /// (the second role is -currently- only available if compiled natively)
    Custom(Box<CustomBuild>),
    /// for obvious reasons only available if compiled natively.
    FromFile(Box<FromFile>),
}

impl Shape {
    /// returns `true` iff variant [`Self::Custom`] is held and
    /// the basis shape is [`Self::SingleVertex`].
    pub fn is_pure_custom(&self) -> bool {
        matches!(self, Self::Custom(c) if c.basis == Self::SingleVertex)
    }

    pub fn name_str(&self) -> &'static str {
        match self {
            Self::SingleVertex => "SingleVertex",
            Self::Tetrahedron => "Tetrahedron",
            Self::Octahedron => "Octahedron",
            Self::Icosahedron => "Icosahedron",
            Self::DividedIcosahedron(_) => "Inflated Icosahedron",
            Self::Cube => "Cube",
            Self::Football => "Football",
            Self::FabianHamann => "Fabian Hamanns Graph",
            Self::Dodecahedron => "Dodecahedron",
            Self::TriangTorus => "Torus (Triangles)",
            Self::TriangTorusSkewed(_) => "Skewed Torus (Triangles)",
            Self::SquareTorus => "Torus (Squares)",
            Self::TriangGrid => "Grid (Triangles)",
            Self::SquareGrid => "Grid (Squares)",
            Self::RegularPolygon2D(_) => "2D Polygon triangulated",
            Self::Random2D(_) => "2D Disk triangulated",
            Self::Custom(_) if self.is_pure_custom() => "Custom",
            Self::Custom(_) => "Extend Current Graph",
            Self::FromFile(_) => "From File",
        }
    }

    /// these are the names used for file names etc. we thus want to keep them stable.
    pub fn to_sting(&self) -> String {
        match self {
            Self::SingleVertex => "Einzelner-Knoten".to_string(),
            Self::Cube => "Wuerfel".to_string(),
            Self::DividedIcosahedron(pressure) => format!("Ikosaeder-{pressure}x-aufgepustet"),
            Self::Dodecahedron => "Dodekaeder".to_string(),
            Self::FabianHamann => "Fabian-Hamann".to_string(),
            Self::Football => "Fussball".to_string(),
            Self::Octahedron => "Oktaeder".to_string(),
            Self::Random2D(seed) => format!("Zufaellig-{seed}"),
            Self::TriangTorus => "Torus-Dreiecke".to_string(),
            Self::TriangTorusSkewed(dy) => format!("Torus-Dreiecke-Schief-{dy}"),
            Self::SquareTorus => "Torus-Vierecke".to_string(),
            Self::TriangGrid => "Gitter-Dreiecke".to_string(),
            Self::SquareGrid => "Gitter-Vierecke".to_string(),
            Self::RegularPolygon2D(nr_sides) => format!("2d-Polygon-{nr_sides}-seitig"),
            Self::Tetrahedron => "Tetraeder".to_string(),
            Self::Icosahedron => "Ikosaeder".to_string(),
            Self::Custom(c) if self.is_pure_custom() => {
                format!("Custom-{}", c.print_name())
            },
            Self::Custom(c) => {
                let basis = c.basis.to_sting();
                let steps = c.print_name();
                format!("Extended-{basis}-{steps}")
            },
            Self::FromFile(ff) => format!("FromFile-{}", ff.class_name),
        }
    }

    pub fn emoji(&self) -> &'static str {
        match self {
            Self::SingleVertex => "(1)",
            Self::Tetrahedron => "🌐Tet",
            Self::Octahedron => "🌐Oct",
            Self::Icosahedron => "🌐Ico",
            Self::DividedIcosahedron(_) => "🌐Ico💨",
            Self::Cube => "🎲",
            Self::Football => "⚽",
            Self::FabianHamann => "⚽F.H.",
            Self::Dodecahedron => "🌐Dod",
            Self::TriangTorus => "🍩6",
            Self::TriangTorusSkewed(_) => "🍩6S",
            Self::SquareTorus => "🍩4",
            Self::TriangGrid => "✂🍩6",
            Self::SquareGrid => "✂🍩4",
            Self::RegularPolygon2D(_) => "⬣",
            Self::Random2D(_) => "⏺",
            Self::Custom(_) if self.is_pure_custom() => "🔨",
            Self::Custom(_) => "+🔨",
            Self::FromFile(_) => "📁",
        }
    }

    pub fn min_res(&self) -> isize {
        match self {
            Self::SingleVertex
            | Self::Tetrahedron
            | Self::Octahedron
            | Self::Icosahedron
            | Self::DividedIcosahedron(_)
            | Self::Cube
            | Self::Football
            | Self::FabianHamann
            | Self::Dodecahedron
            | Self::RegularPolygon2D(_)
            | Self::Random2D(_) => 0,

            Self::TriangTorus | Self::SquareTorus | Self::TriangGrid | Self::SquareGrid => 2,
            Self::TriangTorusSkewed(_) => 3,

            Self::Custom(c) => c.basis.min_res(),
            Self::FromFile(_) => 0,
        }
    }

    pub fn max_res(&self) -> isize {
        match self {
            Self::SingleVertex => 0,
            Self::Tetrahedron
            | Self::Octahedron
            | Self::Icosahedron
            | Self::DividedIcosahedron(_)
            | Self::Cube
            | Self::Football
            | Self::FabianHamann
            | Self::Dodecahedron
            | Self::RegularPolygon2D(_)
            | Self::Random2D(_) => 200,

            Self::TriangTorus
            | Self::TriangTorusSkewed(_)
            | Self::SquareTorus
            | Self::TriangGrid
            | Self::SquareGrid => 800,

            Self::Custom(c) => c.basis.max_res(),
            Self::FromFile(_) => 10000,
        }
    }

    pub fn is_3d(&self) -> bool {
        match self {
            Self::Tetrahedron
            | Self::Octahedron
            | Self::Icosahedron
            | Self::DividedIcosahedron(_)
            | Self::Cube
            | Self::Football
            | Self::FabianHamann
            | Self::Dodecahedron => true,

            Self::SingleVertex
            | Self::TriangTorus
            | Self::TriangTorusSkewed(_)
            | Self::SquareTorus
            | Self::TriangGrid
            | Self::SquareGrid
            | Self::RegularPolygon2D(_)
            | Self::Random2D(_) => false,

            Self::Custom(c) => c.basis.is_3d(),
            Self::FromFile(_) => false,
        }
    }
}
