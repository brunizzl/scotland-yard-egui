use itertools::izip;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use self::bool_csr::BoolCSR;
use crate::geo::*;

use super::*;
use bruteforce::RawCops;

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

    /// returns all automorphisms that map the given vertex to the
    /// representative of his equivalency class
    fn repr_automorphisms(&self, v: usize) -> impl Iterator<Item = &Self::Auto> + '_ + Clone;

    /// returns the representative of the vertices' equivalency class under the automorphism group.
    /// the class representative MUST ALWAYS be the smallest vertex of the class.
    /// thus the invariant `self.class_representatives(v) <= v` must hold for all vertices `v`.
    fn repr_of(&self, v: usize) -> usize;

    /// enumerates one vertex of each vertex class. this vertex will be result of [`Self::to_representative`]
    /// if a vertex of it's class is passed as only vertex.
    fn class_representatives(&self) -> impl Iterator<Item = usize> + '_ + Clone;

    /// assumes identity at beginning
    fn all_automorphisms(&self) -> &[Self::Auto];

    fn into_enum(self) -> SymGroup;

    fn nr_vertices(&self) -> usize;

    /// with `cops` beeing a vertex of the unordered graph power,
    /// e.g. the graph with multisets of fixed size `cops.nr_cops`,
    /// this function returns the representative vertex of `cops` under the group of
    /// automorphisms of the original graph, but applied to every element of the multiset.
    /// all such automorphisms mapping `cops` to it's representative are also returned.
    ///
    /// the input may be reordered, but nothing else.
    /// `cops` must be nonempty.
    /// the returned representative ist guaranteed to be sorted.
    fn power_repr<'a>(&'a self, cops: &mut RawCops) -> (SmallVec<[&'a Self::Auto; 4]>, RawCops) {
        debug_assert!(!cops.is_empty());

        cops.sort_by_key(|&c| self.repr_of(c));
        let rotate_class = self.repr_of(cops[0]);
        let all_autos = cops
            .iter()
            .take_while(|&&c| self.repr_of(c) == rotate_class)
            .flat_map(|&c| self.repr_automorphisms(c));

        let mut best_autos = SmallVec::<[&Self::Auto; 4]>::new();
        let mut best_val = RawCops::uninit(cops.nr_cops);
        let mut new_val = RawCops::uninit(cops.nr_cops);

        for auto in all_autos {
            for (nv, &c) in izip!(&mut new_val[..], &cops[..]) {
                *nv = auto.apply_forward(c);
            }
            new_val.sort_unstable();
            //this relies on the classes beeing sorted by what the smallest vertex appearing in one is.
            debug_assert_eq!(self.repr_of(new_val[0]), rotate_class);

            match new_val.cmp(&best_val) {
                std::cmp::Ordering::Equal => {
                    best_autos.push(auto);
                },
                std::cmp::Ordering::Less => {
                    best_val = new_val;
                    best_autos.clear();
                    best_autos.push(auto);
                },
                std::cmp::Ordering::Greater => {
                    //discard this automorphism
                },
            }
        }
        debug_assert!(!best_autos.is_empty());
        (best_autos, best_val)
    }
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

    fn repr_automorphisms(&self, _: usize) -> impl Iterator<Item = &Self::Auto> + '_ + Clone {
        std::iter::once(&self.identity[0])
    }

    fn repr_of(&self, v: usize) -> usize {
        v
    }

    fn power_repr<'a>(&'a self, cops: &mut RawCops) -> (SmallVec<[&'a Self::Auto; 4]>, RawCops) {
        cops.sort();
        (smallvec::smallvec![&self.identity[0]], *cops)
    }

    fn class_representatives(&self) -> impl Iterator<Item = usize> + '_ + Clone {
        self.identity[0].forward()
    }

    fn all_automorphisms(&self) -> &[Self::Auto] {
        &self.identity
    }

    fn into_enum(self) -> SymGroup {
        SymGroup::None(self)
    }

    fn nr_vertices(&self) -> usize {
        self.identity[0].nr_vertices
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
    pub fn to_representative(&self, cops: &mut RawCops) -> RawCops {
        match self {
            Self::Explicit(e) => e.power_repr(cops).1,
            Self::Torus6(t) => t.power_repr(cops).1,
            Self::Torus4(t) => t.power_repr(cops).1,
            Self::None(n) => n.power_repr(cops).1,
        }
    }

    #[allow(dead_code)]
    pub fn to_explicit(&self) -> ExplicitClasses {
        match self {
            Self::Explicit(e) => e.clone(),
            Self::Torus6(t) => ExplicitClasses::from(t),
            Self::Torus4(t) => ExplicitClasses::from(t),
            Self::None(n) => ExplicitClasses::from(n),
        }
    }
}

pub fn compare<I, J>(f: I, g: J) -> std::cmp::Ordering
where
    I: ExactSizeIterator<Item = usize>,
    J: ExactSizeIterator<Item = usize>,
{
    assert_eq!(f.len(), g.len());
    for (fx, gx) in izip!(f, g) {
        match fx.cmp(&gx) {
            std::cmp::Ordering::Equal => {},
            c => return c,
        };
    }
    std::cmp::Ordering::Equal
}

#[cfg(test)]
pub mod test {
    use super::*;

    /// also returns false if any duplicates exist, but that would be weird to have anyway.
    fn is_sorted(autos: &[impl Automorphism]) -> bool {
        autos
            .iter()
            .tuple_windows()
            .all(|(f, g)| compare(f.forward(), g.forward()).is_lt())
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
            assert!(compare(autos[0].forward(), 0..n).is_eq());

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

    fn automorphisms_respect_edges(graph: &planar3d::Embedding3D) {
        fn test_impl<A: Automorphism>(autos: &[A], edges: &EdgeList) {
            let mut mapped_neighs = Vec::new();
            let mut neighs_mapped = Vec::new();
            for auto in autos {
                let fw = |v| auto.apply_forward(v);
                let bw = |v| auto.apply_backward(v);
                for v in 0..edges.nr_vertices() {
                    let mapped = fw(v);
                    mapped_neighs.clear();
                    mapped_neighs.extend(edges.neighbors_of(mapped));
                    mapped_neighs.sort();

                    neighs_mapped.clear();
                    neighs_mapped.extend(edges.neighbors_of(v).map(fw));
                    neighs_mapped.sort();

                    assert_eq!(mapped_neighs, neighs_mapped);
                    assert_eq!(bw(mapped), v);
                }
            }
        }

        let edges = graph.edges();
        match graph.sym_group() {
            SymGroup::Explicit(e) => test_impl(e.all_automorphisms(), edges),
            SymGroup::Torus4(t) => test_impl(t.all_automorphisms(), edges),
            SymGroup::Torus6(t) => test_impl(t.all_automorphisms(), edges),
            SymGroup::None(_) => panic!("why test autmorphisms without symmetry?"),
        }
    }

    #[test]
    fn tetrahedron_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_tetrahedron(2);
        is_group(g2.sym_group());
        automorphisms_respect_edges(&g2);

        let g10 = Embedding3D::new_subdivided_tetrahedron(10);
        is_group(g10.sym_group());
        automorphisms_respect_edges(&g10);
    }

    #[test]
    fn cube_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_cube(2);
        is_group(g2.sym_group());
        automorphisms_respect_edges(&g2);

        let g10 = Embedding3D::new_subdivided_cube(10);
        is_group(g10.sym_group());
        automorphisms_respect_edges(&g10);
    }

    #[test]
    fn octahedron_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_octahedron(2);
        is_group(g2.sym_group());
        automorphisms_respect_edges(&g2);

        let g10 = Embedding3D::new_subdivided_octahedron(10);
        is_group(g10.sym_group());
        automorphisms_respect_edges(&g10);
    }

    #[test]
    fn icosahedron_automorphisms_are_group() {
        let g2 = Embedding3D::new_subdivided_icosahedron(2);
        is_group(g2.sym_group());
        automorphisms_respect_edges(&g2);

        let g10 = Embedding3D::new_subdivided_icosahedron(10);
        is_group(g10.sym_group());
        automorphisms_respect_edges(&g10);
    }

    #[test]
    fn torus6_automorphisms_are_group() {
        let g3 = Embedding3D::new_subdivided_triangle_grid(3, true);
        is_group(g3.sym_group());
        automorphisms_respect_edges(&g3);

        let g5 = Embedding3D::new_subdivided_triangle_grid(6, true);
        is_group(g5.sym_group());
        automorphisms_respect_edges(&g5);
    }

    #[test]
    fn torus4_automorphisms_are_group() {
        let g3 = Embedding3D::new_subdivided_squares_grid(3, true);
        is_group(g3.sym_group());
        automorphisms_respect_edges(&g3);

        let g5 = Embedding3D::new_subdivided_squares_grid(6, true);
        is_group(g5.sym_group());
        automorphisms_respect_edges(&g5);
    }
}
