use itertools::izip;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use self::bool_csr::BoolCSR;
use crate::geo::*;

use super::*;

pub mod explicit;
pub use explicit::*;
pub mod torus;

/// represents a graph automorphism
/// where vertices are named [`0..self.nr_vertices()`]
///
/// -> iff vertices `u` and `v` share an edge, then vertices `self.forward[u]` and `self.forward[v]` share an edge.
///
/// -> for any vertex `v` holds that `v == self.forward[self.backward[v]]`
pub trait Automorphism {
    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone;
    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone;

    fn nr_vertices(&self) -> usize;

    fn apply_forward(&self, v: usize) -> usize {
        debug_assert!(v < self.nr_vertices());
        self.forward().nth(v).unwrap()
    }

    fn apply_backward(&self, v: usize) -> usize {
        debug_assert!(v < self.nr_vertices());
        self.backward().nth(v).unwrap()
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct Identity {
    nr_vertices: usize,
}

impl Identity {
    pub fn new(nr_vertices: usize) -> Self {
        Identity { nr_vertices }
    }
}

impl Automorphism for Identity {
    fn forward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        0..self.nr_vertices()
    }

    fn backward(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        0..self.nr_vertices()
    }

    fn nr_vertices(&self) -> usize {
        self.nr_vertices
    }
}

/// representation of all [`Automorphism`]'s of a given graph
pub trait SymmetryGroup {
    type Auto: Automorphism;
    type AutoIter<'a>: IntoIterator<Item = &'a Self::Auto> + Clone
    where
        Self: 'a;

    /// applies an authomorphism to vertices in place, returns all authomorphisms which
    /// map original vertices to result.
    /// order of vertices may be altered.
    ///
    /// this function must guarantee to find a unique representative of vertices and
    /// ALL autos bringing them there.
    fn to_representative<'a>(&'a self, vertices: &mut [usize]) -> Self::AutoIter<'a>;

    /// enumerates one vertex of each vertex class. this vertex will be result of [`Self::to_representative`]
    /// if a vertex of it's class is passed as only vertex.
    fn class_representatives(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone;

    /// assumes identity at beginning
    fn all_automorphisms(&self) -> &[Self::Auto];

    const HAS_SYMMETRY: bool = true;

    fn into_enum(self) -> SymGroup;
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct NoSymmetry {
    identity: [Identity; 1],
}

impl NoSymmetry {
    pub fn new(nr_vertices: usize) -> Self {
        Self {
            identity: [Identity::new(nr_vertices)],
        }
    }
}

impl SymmetryGroup for NoSymmetry {
    type Auto = Identity;
    type AutoIter<'a> = std::iter::Once<&'a Identity>;

    fn to_representative<'a>(&'a self, vertices: &mut [usize]) -> Self::AutoIter<'a> {
        vertices.sort();
        std::iter::once(&self.identity[0])
    }

    fn class_representatives(&self) -> impl ExactSizeIterator<Item = usize> + '_ + Clone {
        self.identity[0].forward()
    }

    fn all_automorphisms(&self) -> &[Self::Auto] {
        &self.identity
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::None(self)
    }

    const HAS_SYMMETRY: bool = false;
}

pub trait DynAutomorphism {
    fn dyn_apply_forward(&self, v: usize) -> usize;
    fn dyn_apply_backward(&self, v: usize) -> usize;
}

impl<T: Automorphism> DynAutomorphism for T {
    fn dyn_apply_forward(&self, v: usize) -> usize {
        <Self as Automorphism>::apply_forward(self, v)
    }

    fn dyn_apply_backward(&self, v: usize) -> usize {
        <Self as Automorphism>::apply_backward(self, v)
    }
}

pub trait DynSymmetryGroup {
    fn dyn_to_representative<'a>(
        &'a self,
        vertices: &mut [usize],
    ) -> SmallVec<[&'a dyn DynAutomorphism; 4]>;

    fn for_each_transform(&self, f: &mut dyn FnMut(&dyn DynAutomorphism));
}

impl<T: SymmetryGroup> DynSymmetryGroup for T {
    fn dyn_to_representative<'a>(
        &'a self,
        vertices: &mut [usize],
    ) -> SmallVec<[&'a dyn DynAutomorphism; 4]> {
        let mut res = SmallVec::new();
        let iter = <Self as SymmetryGroup>::to_representative(self, vertices);
        for auto in iter {
            res.push(auto as &dyn DynAutomorphism);
        }
        res
    }

    fn for_each_transform(&self, f: &mut dyn FnMut(&dyn DynAutomorphism)) {
        for auto in self.all_automorphisms() {
            f(auto as &dyn DynAutomorphism);
        }
    }
}

#[derive(Serialize, Deserialize)]
pub enum SymGroup {
    Explicit(ExplicitClasses),
    Torus6(torus::TorusSymmetry6),
    Torus4(torus::TorusSymmetry4),
    None(NoSymmetry),
}

impl SymGroup {
    pub fn to_dyn(&self) -> &dyn DynSymmetryGroup {
        match self {
            Self::Explicit(e) => e as &dyn DynSymmetryGroup,
            Self::Torus6(t) => t as &dyn DynSymmetryGroup,
            Self::Torus4(t) => t as &dyn DynSymmetryGroup,
            Self::None(n) => n as &dyn DynSymmetryGroup,
        }
    }
}

#[cfg(test)]
pub mod test {
    use super::*;
    use std::cmp::Ordering;

    pub fn compare<I, J>(f: I, g: J) -> Ordering
    where
        I: ExactSizeIterator<Item = usize>,
        J: ExactSizeIterator<Item = usize>,
    {
        assert_eq!(f.len(), g.len());
        for (fx, gx) in izip!(f, g) {
            match fx.cmp(&gx) {
                Ordering::Equal => {},
                c => return c,
            };
        }
        Ordering::Equal
    }

    /// also returns false if any duplicates exist, but that would be weird to have anyway.
    fn is_sorted(autos: &[impl Automorphism]) -> bool {
        autos
            .iter()
            .tuple_windows()
            .all(|(f, g)| compare(f.forward(), g.forward()) == Ordering::Less)
    }

    fn contains_forward(
        autos: &[impl Automorphism],
        f: impl ExactSizeIterator<Item = usize> + Clone,
    ) -> bool {
        autos.binary_search_by(|a| compare(a.forward(), f.clone())).is_ok()
    }

    pub fn closed_under_inverse(autos: &[impl Automorphism]) {
        assert!(is_sorted(autos)); //not required for correctness, but for sane test times.
        for auto in autos {
            assert!(contains_forward(autos, auto.backward()));
        }
    }

    /// computes "x -> f[g[x]]"
    fn compose(f: &[usize], g: impl ExactSizeIterator<Item = usize>, res: &mut [usize]) {
        assert_eq!(f.len(), res.len());
        assert_eq!(g.len(), res.len());
        for (i, gi) in izip!(0.., g) {
            res[i] = f[gi];
        }
    }

    /// caution: this function runs in O(n^2)
    pub fn closed_under_composition(autos: &[impl Automorphism]) {
        assert!(is_sorted(autos)); //not required for correctness, but for sane test times.
        assert!(autos.iter().map(|a| a.nr_vertices()).all_equal());
        let n = autos[0].nr_vertices();
        let mut f = vec![0; n];
        let mut composed = vec![0; n];
        for a1 in autos {
            f.clear();
            f.extend(a1.forward());
            for a2 in autos {
                compose(&f, a2.forward(), &mut composed);
                assert!(contains_forward(autos, composed.iter().copied()));
            }
        }
    }

    pub fn is_group(s: &SymGroup) {
        fn is_group_impl<S>(autos: &S)
        where
            S: SymmetryGroup,
            S::Auto: Clone,
        {
            let mut autos = Vec::from(autos.all_automorphisms());
            //identity is not required to be stored first for this to be a group, but still.
            let n = autos[0].nr_vertices();
            assert!(compare(autos[0].forward(), 0..n) == Ordering::Equal);

            autos.sort_by(|f, g| compare(f.forward(), g.forward()));
            closed_under_inverse(&autos);
            closed_under_composition(&autos);

            //function application is always associative, so this needs no testing here.
        }

        match s {
            SymGroup::Explicit(e) => is_group_impl(e),
            SymGroup::None(n) => is_group_impl(n),
            SymGroup::Torus4(t4) => is_group_impl(t4),
            SymGroup::Torus6(t6) => is_group_impl(t6),
        }
    }

    #[test]
    fn tetrahedron_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_tetrahedron(2);
        is_group(g2.sym_group());

        let g10 = Embedding3D::new_subdivided_tetrahedron(10);
        is_group(g10.sym_group());
    }

    #[test]
    fn octahedron_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_octahedron(2);
        is_group(g2.sym_group());

        let g10 = Embedding3D::new_subdivided_octahedron(10);
        is_group(g10.sym_group());
    }

    #[test]
    fn icosahedron_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_icosahedron(2);
        is_group(g2.sym_group());

        let g10 = Embedding3D::new_subdivided_icosahedron(10);
        is_group(g10.sym_group());
    }

    #[test]
    fn torus6_automorphisms_are_group() {
        let g3 = Embedding3D::new_subdivided_triangle_torus(3);
        is_group(g3.sym_group());

        let g5 = Embedding3D::new_subdivided_triangle_torus(6);
        is_group(g5.sym_group());
    }

    #[test]
    fn torus4_automorphisms_are_group() {
        let g3 = Embedding3D::new_subdivided_squares_torus(3);
        is_group(g3.sym_group());

        let g5 = Embedding3D::new_subdivided_squares_torus(6);
        is_group(g5.sym_group());
    }
}
