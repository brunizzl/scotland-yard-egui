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
    /// connect vertices with these indices
    Edge(usize, usize),
    /// delete edge between vertices with these indices
    DeleteEdge(usize, usize),
    /// adds every Edge between a vertex in `.0` and `.1`.
    CompleteBetween(Box<[usize]>, Box<[usize]>),
    /// adds every edge between consecutive elements
    Path(Box<[usize]>),
}

/// operator that separates first and last element of a sequence of consecutive integers.
const SEQUENCE_SEP: &str = "..=";

impl BuildStep {
    pub const EXPLAINER: &str = "\
        N<zahl>: <zahl>-distanz und nähere Knoten werden Nachbarn\n\
        D<zahl>: Jede Kante wird Weg mit <zahl> vielen inneren Knoten\n\
        V<x>,<y>,<z>: Knoten mit Koordinaten (<x>, <y>, <z>) / 1000\n\
        E<u>,<v>: Kante zwischen Knoten mit Indices <u> und <v>.\n\
        K(<X>)(<Y>): alle Kanten zwischen Folgen <X> und <Y>\n\
        P(<X>): Kantenzug entlang Folge <X>\n\n\
        eine Folge hat die Form <a1>, ..., <an>.\n\
        BITTE BEACHTE WERKZEUGE [±v] UND [±e]\
        ";

    pub const DEFAULT_Z: i32 = (crate::graph::Z_OFFSET_2D * 1000.0) as i32;
}

impl std::fmt::Display for BuildStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // writes a sequence, except long subsequences of consecutive entries are written with ..
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
                // arbitrary decision to no rewrite streaks of just two or three elements
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
            Self::NeighNeighs(1) => write!(f, "N"),
            Self::SubdivEdges(1) => write!(f, "D"),
            Self::NeighNeighs(n) => write!(f, "N{n}"),
            Self::SubdivEdges(n) => write!(f, "D{n}"),
            Self::Vertex(v, [x, y, Self::DEFAULT_Z]) => write!(f, "V{v}({x},{y})"),
            Self::Vertex(v, [x, y, z]) => write!(f, "V{v}({x},{y},{z})"),
            Self::DeleteVertex(v) => write!(f, "DelV{v}"),
            Self::Edge(v1, v2) => write!(f, "E{v1},{v2}"),
            Self::DeleteEdge(v1, v2) => write!(f, "DelE{v1},{v2}"),
            Self::CompleteBetween(xs, ys) => {
                write!(f, "K")?;
                write_sequence(f, xs)?;
                write_sequence(f, ys)
            },
            Self::Path(xs) => {
                write!(f, "P")?;
                write_sequence(f, xs)
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub struct CustomBuild {
    pub basis: Shape,
    pub build_steps_string: String,
    pub build_steps: Vec<BuildStep>,
    /// identifier chosen by user.
    #[serde(default)]
    pub name: String,
}

impl CustomBuild {
    pub fn new(basis: Shape) -> Self {
        let name = if crate::app::NATIVE {
            use chrono::{DateTime, Local, Timelike};
            let now = DateTime::<Local>::from(std::time::SystemTime::now());
            let date = now.date_naive();
            let secs = now.time().num_seconds_from_midnight();
            format!("{date}-{secs}")
        } else {
            "unbenannt".to_string()
        };
        Self {
            basis,
            build_steps_string: String::new(),
            build_steps: Vec::new(),
            name,
        }
    }

    /// must be run after adding [`BuildStep::DeleteVertex`] or [`BuildStep::DeleteEdge`],
    /// as these two are applied and then removed as steps in this function.
    pub fn normalize_build_steps(&mut self) {
        for step in &mut self.build_steps {
            if let BuildStep::Edge(v1, v2) = step
                && v1 > v2
            {
                *step = BuildStep::Edge(*v2, *v1);
            }
            if let BuildStep::DeleteEdge(v1, v2) = step
                && v1 > v2
            {
                *step = BuildStep::DeleteEdge(*v2, *v1);
            }
        }
        let mut i = 0;
        // note: there should be at most a single deletion command each time, because these are
        // invoked by clicking. therefore the O(nr_vertices^2) runtime here is ok.
        while i < self.build_steps.len() {
            let curr_step = &self.build_steps[i];
            i += 1;
            if let &BuildStep::DeleteEdge(v1, v2) = curr_step {
                let del_cmd = curr_step.clone();
                let to_del = BuildStep::Edge(v1, v2);
                let remove_edge = |s: &mut _| *s == del_cmd || *s == to_del;
                let nr_removed = self.build_steps.extract_if(.., remove_edge).count();
                i = i.saturating_sub(nr_removed);
            } else if let &BuildStep::DeleteVertex(v) = curr_step {
                let del_cmd = curr_step.clone();
                let remove_vertex =
                    |s: &mut _| *s == del_cmd || matches!(s, BuildStep::Vertex(u, _) if *u == v);
                let nr_vertices_removed = self.build_steps.extract_if(.., remove_vertex).count();
                debug_assert!(nr_vertices_removed > 0); // at least the removeal command should be removed.
                // vertices with larger index than the removed one must be adjusted.
                let decr_larger_vertex = |u: &mut usize| {
                    // only decrement if removal was successfull (more than the removal command are removed)
                    if nr_vertices_removed > 1 && *u > v {
                        *u -= 1;
                    }
                };
                // all edges containing the removed vertex are also removed.
                let remove_edge = |s: &mut BuildStep| {
                    if let BuildStep::Edge(u1, u2) | BuildStep::DeleteEdge(u1, u2) = s {
                        if *u1 == v || *u2 == v {
                            return true;
                        }
                        decr_larger_vertex(u1);
                        decr_larger_vertex(u2);
                    }
                    false
                };
                let nr_edges_removed = self.build_steps.extract_if(.., remove_edge).count();
                // adjust the kept vertices:
                for s in &mut self.build_steps {
                    let adjust_box = |us: &mut Box<[usize]>| {
                        let mut as_vec = std::mem::take(us).into_vec();
                        as_vec.extract_if(.., |u| *u == v).count();
                        for u in &mut as_vec {
                            decr_larger_vertex(u);
                        }
                        *us = Box::from(as_vec);
                    };
                    if let BuildStep::CompleteBetween(xs, ys) = s {
                        adjust_box(xs);
                        adjust_box(ys);
                    }
                    if let BuildStep::Vertex(u, _) = s {
                        decr_larger_vertex(u);
                    }
                    if let BuildStep::Path(xs) = s {
                        adjust_box(xs);
                    }
                }
                // ensure to not skip a case
                i = i.saturating_sub(nr_vertices_removed + nr_edges_removed);
            }
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

    /// returns an identifier. this is the name if that is available, else the degree sequence.
    pub fn print_fingerprint(&self) -> String {
        if !self.name.is_empty() {
            return self.name.clone();
        }

        let nr_vertices = self
            .build_steps
            .iter()
            .filter(|s| matches!(s, BuildStep::Vertex(_, _)))
            .count();
        let mut degrees = vec![0; nr_vertices];
        for step in &self.build_steps {
            if let BuildStep::Edge(u, v) = *step {
                if u < nr_vertices {
                    degrees[u] += 1;
                }
                if v < nr_vertices {
                    degrees[v] += 1;
                }
            }
        }

        use std::fmt::Write;
        let mut sep = "";
        let mut result = String::new();
        write!(result, "(").ok();
        for deg in degrees {
            write!(result, "{}{deg}", std::mem::replace(&mut sep, "-")).ok();
        }
        write!(result, ")").ok();
        result
    }

    /// the removal build steps cannot be parsed and should only
    /// be creatable by clicking with the right tool + shift.
    pub fn parse_build_steps(&mut self) {
        let mut data_string = std::mem::take(&mut self.build_steps_string);
        data_string.retain(|c| !c.is_ascii_whitespace());
        let mut data: &str = &data_string;
        let parse_usize = |data: &mut &str| -> Option<usize> {
            let int_end = data.find(|c: char| !c.is_ascii_digit()).unwrap_or(data.len());
            let (int_part, rest) = data.split_at(int_end);
            *data = rest;
            int_part.parse::<usize>().ok()
        };
        let parse_i32 = |data: &mut &str| -> Option<i32> {
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
        };
        fn remove_single(data: &mut &str, pattern: impl Fn(char) -> bool) {
            if data.chars().next().is_some_and(pattern) {
                let mut first_match = true;
                let is_first = |_| std::mem::replace(&mut first_match, false);
                *data = data.trim_start_matches(is_first);
            }
        }
        let remove_exact = |data: &mut &str, ch: char| {
            remove_single(data, |c| c == ch);
        };
        let remove_comma = |data: &mut &str| {
            remove_exact(data, ',');
        };
        let parse_sequence = |data: &mut &str, sort: bool| {
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
        };
        self.build_steps.clear();
        while !data.is_empty() {
            if data.starts_with("N") {
                data = &data[1..];
                let n = parse_usize(&mut data).unwrap_or(1);
                self.build_steps.push(BuildStep::NeighNeighs(n));
            } else if data.starts_with("D") {
                data = &data[1..];
                let n = parse_usize(&mut data).unwrap_or(1);
                self.build_steps.push(BuildStep::SubdivEdges(n));
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
                let ys = parse_sequence(&mut data, true);
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
        self.normalize_build_steps();
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
                format!("Custom-{}", c.print_fingerprint())
            },
            Self::Custom(c) => {
                let basis = c.basis.to_sting();
                let steps = c.print_build_steps(true);
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
