use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
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
}

impl BuildStep {
    pub const EXPLAINER: &str = "\
        N<zahl>: <zahl>-distanz und nähere Knoten werden Nachbarn\n\
        D<zahl>: Jede Kante wird Weg mit <zahl> vielen inneren Knoten\n\
        V<x>,<y>,<z>: Knoten mit Koordinaten (<x>, <y>, <z>) / 1000\n\
        E<u>,<v>: Kante zwischen Knoten mit Indices <u> und <v>.\n\n\
        BITTE BEACHTE WERKZEUGE [±v] UND [±e]\
        ";

    pub const DEFAULT_Z: i32 = (crate::graph::Z_OFFSET_2D * 1000.0) as i32;
}

impl std::fmt::Display for BuildStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub struct CustomBuild {
    pub basis: Shape,
    pub build_steps_string: String,
    pub build_steps: Vec<BuildStep>,
}

impl CustomBuild {
    pub fn new(basis: Shape) -> Self {
        Self {
            basis,
            build_steps_string: String::new(),
            build_steps: Vec::new(),
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
            let curr_step = self.build_steps[i];
            i += 1;
            if let del_cmd @ BuildStep::DeleteEdge(v1, v2) = curr_step {
                let to_del = BuildStep::Edge(v1, v2);
                let remove_edge = |s: &mut _| *s == del_cmd || *s == to_del;
                let nr_removed = self.build_steps.extract_if(.., remove_edge).count();
                i = i.saturating_sub(nr_removed);
            }
            if let del_cmd @ BuildStep::DeleteVertex(v) = curr_step {
                let remove_vertex =
                    |s: &mut _| *s == del_cmd || matches!(s, BuildStep::Vertex(u, _) if *u == v);
                let nr_vertices_removed = self.build_steps.extract_if(.., remove_vertex).count();
                debug_assert!(nr_vertices_removed > 0); // at least the removeal command should be removed.
                // all edges containing the removed vertex are also removed.
                let remove_edge = |s: &mut BuildStep| {
                    // vertices with larger index than the removed one must be adjusted.
                    let decr_larger_vertex = |u: &mut usize| {
                        // only decrement if removal was successfull (more than the removal command are removed)
                        if nr_vertices_removed > 1 && *u > v {
                            *u -= 1;
                        }
                    };
                    if let BuildStep::Edge(u1, u2) | BuildStep::DeleteEdge(u1, u2) = s {
                        if *u1 == v || *u2 == v {
                            return true;
                        }
                        decr_larger_vertex(u1);
                        decr_larger_vertex(u2);
                    }
                    if let BuildStep::Vertex(u, _) = s {
                        decr_larger_vertex(u);
                    }
                    false
                };
                let nr_edges_removed = self.build_steps.extract_if(.., remove_edge).count();
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
        let remove_space_or_comma = |data: &mut &str| {
            remove_single(data, |c| c == ' ' || c == ',');
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
                    remove_single(&mut data, |c| c == '(');
                    let x = parse_i32(&mut data).unwrap_or(0);
                    remove_space_or_comma(&mut data);
                    let y = parse_i32(&mut data).unwrap_or(0);
                    remove_space_or_comma(&mut data);
                    let z = parse_i32(&mut data).unwrap_or(BuildStep::DEFAULT_Z);
                    remove_single(&mut data, |c| c == ')');
                    self.build_steps.push(BuildStep::Vertex(v, [x, y, z]));
                }
            } else if data.starts_with("E") {
                data = &data[1..];
                let v1 = parse_usize(&mut data).unwrap_or(0);
                remove_space_or_comma(&mut data);
                let v2 = parse_usize(&mut data).unwrap_or(0);
                self.build_steps.push(BuildStep::Edge(v1, v2));
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
    pub fn pure_custom(&self) -> bool {
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
            Self::Custom(_) if self.pure_custom() => "Custom",
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
            Self::Custom(c) if self.pure_custom() => {
                format!("Custom-{}", c.print_build_steps(true))
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
            Self::Custom(_) if self.pure_custom() => "🔨",
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
