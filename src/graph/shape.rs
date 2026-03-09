use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub enum BuildStep {
    /// neighbors of neighbors become neighbors, procedure is iterated given number of times
    NeighNeihs(usize),
    /// every edge becomes a path with given number of new interior vertices
    SubdivEdges(usize),
    /// vertex at given coordinates.
    /// x, y and z coordinates in better printable / parseable form.
    /// the x and y coordinates here are divided by `1000` to get to the corresponding float values.
    Vertex(i32, i32, i32),
    /// connect vertices with these indices
    Edge(usize, usize),
}

impl BuildStep {
    pub const EXPLAINER: &str = "\
        N<zahl>: <zahl>-distanz und nähere Knoten werden Nachbarn\n\
        D<zahl>: Jede Kante wird Weg mit <zahl> vielen inneren Knoten\n\
        V<x>,<y>,<z>: Knoten mit Koordinaten (<x>, <y>, <z>) / 1000\n\
        E<u>,<v>: Kante zwischen Knoten mit Indices <u> und <v>.\
        ";

    pub const DEFAULT_Z: i32 = (crate::graph::Z_OFFSET_2D * 1000.0) as i32;
}

impl std::fmt::Display for BuildStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NeighNeihs(1) => write!(f, "N"),
            Self::SubdivEdges(1) => write!(f, "D"),
            Self::NeighNeihs(n) => write!(f, "N{n}"),
            Self::SubdivEdges(n) => write!(f, "D{n}"),
            Self::Vertex(x, y, Self::DEFAULT_Z) => write!(f, "V{x},{y}"),
            Self::Vertex(x, y, z) => write!(f, "V{x},{y},{z}"),
            Self::Edge(v1, v2) => write!(f, "E{v1},{v2}"),
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
        let space_or_comma = |data: &mut &str| {
            if data.starts_with(" ") || data.starts_with(",") {
                *data = &data[1..];
            }
        };
        self.build_steps.clear();
        while !data.is_empty() {
            if data.starts_with("N") {
                data = &data[1..];
                let n = parse_usize(&mut data).unwrap_or(1);
                self.build_steps.push(BuildStep::NeighNeihs(n));
            } else if data.starts_with("D") {
                data = &data[1..];
                let n = parse_usize(&mut data).unwrap_or(1);
                self.build_steps.push(BuildStep::SubdivEdges(n));
            } else if data.starts_with("V") {
                data = &data[1..];
                let x = parse_i32(&mut data).unwrap_or(0);
                space_or_comma(&mut data);
                let y = parse_i32(&mut data).unwrap_or(0);
                space_or_comma(&mut data);
                let z = parse_i32(&mut data).unwrap_or(BuildStep::DEFAULT_Z);
                self.build_steps.push(BuildStep::Vertex(x, y, z));
            } else if data.starts_with("E") {
                data = &data[1..];
                let v1 = parse_usize(&mut data).unwrap_or(0);
                space_or_comma(&mut data);
                let v2 = parse_usize(&mut data).unwrap_or(0);
                self.build_steps.push(BuildStep::Edge(v1, v2));
            } else {
                // remove the leading character and try again
                let mut once_true = true;
                data = data.trim_start_matches(|_| std::mem::replace(&mut once_true, false));
            }
        }
        self.build_steps_string = self.print_build_steps(false);
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub enum Shape {
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
    Custom(Box<CustomBuild>),
}

impl Shape {
    pub const fn name_str(&self) -> &'static str {
        match self {
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
            Self::Custom(_) => "Custom",
        }
    }

    pub fn to_sting(&self) -> String {
        match self {
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
            Self::Custom(c) => {
                format!(
                    "Custom-{}-{}",
                    c.basis.to_sting(),
                    c.print_build_steps(true)
                )
            },
        }
    }

    pub fn emoji(&self) -> &'static str {
        match self {
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
            Self::Custom(_) => "🔨",
        }
    }

    pub fn min_res(&self) -> isize {
        match self {
            Self::Tetrahedron
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

            Self::TriangTorus
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
