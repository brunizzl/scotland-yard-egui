
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub enum GraphShape {
    Tetrahedron,
    Octahedron,
    Icosahedron,
    DividedIcosahedron(isize),
    Cube,
    Football,
    FabianHamann,
    Dodecahedron,
    TriangTorus,
    SquareTorus,
    RegularPolygon2D(isize),
    Random2D(u32),
}

impl GraphShape {
    pub const fn name_str(self) -> &'static str {
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
            Self::SquareTorus => "Torus (Vierecke)",
            Self::RegularPolygon2D(_) => "2D Polygon trianguliert",
            Self::Random2D(_) => "2D Kreisscheibe trianguliert",
        }
    }

    pub fn to_sting(self) -> String {
        match self {
            Self::Cube => "Wuerfel".to_string(),
            Self::DividedIcosahedron(pressure) => format!("Ikosaeder-{pressure}x-aufgepustet"),
            Self::Dodecahedron => "Dodekaeder".to_string(),
            Self::FabianHamann => "Fabian-Hamann".to_string(),
            Self::Football => "Fussball".to_string(),
            Self::Octahedron => "Oktaeder".to_string(),
            Self::Random2D(seed) => format!("Zufaellig-{seed}"),
            Self::TriangTorus => "Torus-Dreiecke".to_string(),
            Self::SquareTorus => "Torus-Vierecke".to_string(),
            Self::RegularPolygon2D(nr_sides) => format!("2d-Polygon-{nr_sides}-seitig"),
            Self::Tetrahedron => "Tetraeder".to_string(),
            Self::Icosahedron => "Ikosaeder".to_string(),
        }
    }

    pub fn emoji(self) -> &'static str {
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
            Self::SquareTorus => "🍩4",
            Self::RegularPolygon2D(_) => "⬣",
            Self::Random2D(_) => "⏺",
        }
    }

    pub fn min_res(self) -> isize {
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
            Self::TriangTorus => 2,
            Self::SquareTorus => 2,
        }
    }
}
