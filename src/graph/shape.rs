use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub enum BuildStep {
    /// neighbors of neighbors become neighbors, procedure is iterated given number of times
    NeighNeighs(usize),
    /// every edge becomes a path with given number of new interior vertices
    SubdivEdges(usize),
    /// vertex at given coordinates.
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
        N<zahl>: <zahl>-distanz und nähere Knoten werden Nachbarn\n\
        D<zahl>: Jede Kante wird Weg mit <zahl> vielen inneren Knoten\n\
        V<x>,<y>,<z>: Knoten mit Koordinaten (<x>, <y>, <z>) / 1000\n\
        E<u>,<v>: Kante zwischen Knoten mit Indices <u> und <v>.\n\
        K(<X>)(<Y>): alle Kanten zwischen Folgen <X> und <Y>\n\
        ZsgTest: entnebeln neu <=> zusammenhängend alt\n\
        HamTest<u>,<v>: entnebeln neu <=> <u>-<v> hamilton pfad alt\n\n\
        eine Folge hat die Form <a1>, ..., <an>.\n\
        BITTE BEACHTE WERKZEUGE [±v] UND [±e]\
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

// also: the vertex position is irrelevant for the graph theoretic application
//   -> no need to keep move operations.
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
    const DEFAULT_NAME: &str = "unbenannt";

    pub fn create_new_name() -> String {
        if crate::app::NATIVE {
            use chrono::{DateTime, Local, Timelike};
            let now = DateTime::<Local>::from(std::time::SystemTime::now());
            let date = now.date_naive();
            let secs = now.time().num_seconds_from_midnight();
            format!("{date}-{secs}")
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

    /// test if the last two operations are moves of the same vertex. if so, combine them.
    pub fn combine_move_operations_naive(&mut self) {
        use BuildStep::MoveVertex;
        while let [.., MoveVertex(u, du), MoveVertex(v, dv)] = &self.build_steps[..]
            && u == v
        {
            let dcombined = [du[0] + dv[0], du[1] + dv[1], du[2] + dv[2]];
            let new_step = MoveVertex(*u, dcombined);
            _ = self.build_steps.pop();
            _ = self.build_steps.pop();
            self.build_steps.push(new_step);
        }
    }

    pub fn print_build_steps(&self, single_line: bool) -> String {
        use std::fmt::Write;
        let mut res = String::new();
        for step in &self.build_steps {
            write!(res, "{step}").ok();
            if !single_line {
                writeln!(res).ok();
            }
        }
        res
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
        let mut data_string = std::mem::take(&mut self.build_steps_string);
        data_string.retain(|c| !c.is_ascii_whitespace());
        let mut data: &str = &data_string;

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
            Some(sign * val as i32)
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
            while data.starts_with(|c: char| c.is_ascii_digit()) {
                let x = parse_usize(data).unwrap();
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

        self.build_steps.clear();
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
                self.build_steps.push(BuildStep::FogTestHamPath(ends, vis));
            } else if data.starts_with(FOG_TEST_IS_CONNECTED_NAME) {
                data = &data[(FOG_TEST_IS_CONNECTED_NAME.len())..];
                self.build_steps.push(BuildStep::FogTestIsGonnected);
            } else if data.starts_with("DelV") {
                data = &data["DelV".len()..];
                if let Some(v) = parse_usize(&mut data) {
                    self.build_steps.push(BuildStep::DeleteVertex(v));
                }
            } else if data.starts_with("DelE") {
                data = &data["DelE".len()..];
                let v1 = parse_usize(&mut data).unwrap_or(0);
                remove_comma(&mut data);
                let v2 = parse_usize(&mut data).unwrap_or(0);
                self.build_steps.push(BuildStep::DeleteEdge(v1, v2));
            } else if data.starts_with("N") {
                data = &data[1..];
                let n = parse_usize(&mut data).unwrap_or(1);
                self.build_steps.push(BuildStep::NeighNeighs(n));
            } else if data.starts_with("D") {
                data = &data[1..];
                let n = parse_usize(&mut data).unwrap_or(1);
                self.build_steps.push(BuildStep::SubdivEdges(n));
            } else if data.starts_with("MovV") {
                data = &data["MovV".len()..];
                if let Some(v) = parse_usize(&mut data) {
                    remove_exact(&mut data, '(');
                    let x = parse_i32(&mut data).unwrap_or(0);
                    remove_comma(&mut data);
                    let y = parse_i32(&mut data).unwrap_or(0);
                    remove_comma(&mut data);
                    let z = parse_i32(&mut data).unwrap_or(BuildStep::DEFAULT_Z);
                    remove_exact(&mut data, ')');
                    self.build_steps.push(BuildStep::MoveVertex(v, [x, y, z]));
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
                    self.build_steps.push(BuildStep::Vertex(v, [x, y, z]));
                }
            } else if data.starts_with("E") {
                data = &data[1..];
                let v1 = parse_usize(&mut data).unwrap_or(0);
                remove_comma(&mut data);
                let v2 = parse_usize(&mut data).unwrap_or(0);
                self.build_steps.push(BuildStep::Edge(v1, v2));
            } else if data.starts_with("K") {
                data = &data[1..];
                let xs = parse_sequence(&mut data, true);
                let mut ys = parse_sequence(&mut data, true);
                if ys.is_empty() {
                    ys = xs.clone();
                }
                self.build_steps.push(BuildStep::CompleteBetween(xs, ys));
            } else if data.starts_with("P") {
                data = &data[1..];
                let xs = parse_sequence(&mut data, false);
                self.build_steps.push(BuildStep::Path(xs));
            } else {
                // remove the leading character and try again
                remove_single(&mut data, |_| true);
            }
        }
        self.combine_move_operations_naive();
        self.build_steps_string = self.print_build_steps(false);
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
            Self::Tetrahedron => "Tetraeder",
            Self::Octahedron => "Oktaeder",
            Self::Icosahedron => "Ikosaeder",
            Self::DividedIcosahedron(_) => "aufgepusteter Ikosaeder",
            Self::Cube => "Würfel",
            Self::Football => "Fußball",
            Self::FabianHamann => "Fabian Hamanns Graph",
            Self::Dodecahedron => "Dodekaeder",
            Self::TriangTorus => "Torus (Dreiecke)",
            Self::TriangTorusSkewed(_) => "Schiefer Torus (Dreiecke)",
            Self::SquareTorus => "Torus (Vierecke)",
            Self::TriangGrid => "Gitter (Dreiecke)",
            Self::SquareGrid => "Gitter (Vierecke)",
            Self::RegularPolygon2D(_) => "2D Polygon trianguliert",
            Self::Random2D(_) => "2D Kreisscheibe trianguliert",
            Self::Custom(_) if self.is_pure_custom() => "Custom",
            Self::Custom(_) => "erweitere aktuellen Graph",
        }
    }

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
        }
    }
}
