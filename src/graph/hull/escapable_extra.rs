use super::*;

use itertools::Itertools;

/// map from `iproduct!(0..32, 0..32)` to `0..32`
/// where the pair stands for values overlapping in `EscapableNodes::escapable` and `i` beeing the bit set
/// in `self.escapable` to represent vertices reaching that overlap.
struct LazySafeToDilemma {
    /// this is sorted by the pair. it is a vec, because fast access is more important than fast insertion.
    /// (only up to 32 elements can be inserted anyway...)
    from: Vec<(u8, u8)>,
    /// `self.from[i]` maps to `self.to[i]`
    to: Vec<u16>,
    /// `self.from[i]` was encountered `self.number[i]` times
    number: Vec<usize>,
    /// `self.from[i]` was first encountered at vertex `self.first[i]`
    first: Vec<usize>,
}

impl LazySafeToDilemma {
    fn new() -> Self {
        Self {
            from: Vec::new(),
            to: Vec::new(),
            number: Vec::new(),
            first: Vec::new(),
        }
    }

    /// adds new configuration if none for that pair exists, else returns existing one.
    fn make_val(&mut self, overlap: (u8, u8), vertex: usize) -> u16 {
        let pos = self.from.binary_search(&overlap);
        match pos {
            Ok(index) => {
                self.number[index] += 1;
                self.to[index]
            },
            Err(index) => {
                let res = self.from.len() as u16;
                if res >= 32 {
                    return 31;
                }
                self.from.insert(index, overlap);
                self.number.insert(index, 1);
                self.first.insert(index, vertex);
                self.to.insert(index, res);
                res
            },
        }
    }

    fn clear(&mut self) {
        self.from.clear();
    }

    /// contains (from, to, number, first) in that order, where from is itself a pair.
    fn iter(&self) -> impl Iterator<Item = ((u8, u8), u16, usize, usize)> + '_ {
        izip!(
            self.from.iter().copied(),
            self.to.iter().copied(),
            self.number.iter().copied(),
            self.first.iter().copied()
        )
    }
}

/// when two sets of escapable vertices overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
pub struct DilemmaNodes {
    to_this: LazySafeToDilemma,

    /// marks vertices, that neighbor (or are contained in) at least two regions of
    /// `EscapableNodes::escapable` at the same time. -> one entry per vertex.
    /// if bit `i` in entry `i` is set, then the bits `(ei, ej)` mapping in `self.to_this` to `i`
    /// represent the regions in `EscapableNodes::escapable`, that overlap here.
    overlapping: Vec<u32>,

    /// thing we are actually interested in. one entry per vertex, set bit's as in
    /// `Self::overlapping`, just now the regions are extended.
    dilemma: Vec<u32>,

    /// temporary data, only kept permanently to save on allocations.
    values: Vec<usize>,
}

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            to_this: LazySafeToDilemma::new(),
            overlapping: Vec::new(),
            dilemma: Vec::new(),
            values: Vec::new(),
        }
    }

    pub fn dilemma(&self) -> &[u32] {
        &self.dilemma
    }

    fn update_overlapping(&mut self, lazy_safe: &[u32], edges: &EdgeList) {
        self.to_this.clear();
        self.overlapping.clear();
        self.overlapping.resize(lazy_safe.len(), 0);
        let mut bits = smallvec::SmallVec::<[u8; 16]>::new();
        for (v, over, neighs, &ls) in
            izip!(0.., &mut self.overlapping, edges.neighbors(), lazy_safe)
        {
            let union_with_neighs: u32 =
                neighs.map(|n| lazy_safe[n]).fold(ls, std::ops::BitOr::bitor);
            bits.clear();
            bits.extend((0..32).filter(|&i| union_with_neighs & (1 << i) != 0));
            for (index, &i) in izip!(0.., &bits) {
                for &j in &bits[(index + 1)..] {
                    let this_i = self.to_this.make_val((i, j), v);
                    *over |= 1 << this_i;
                }
            }
        }
    }

    fn update_dilemma(&mut self, edges: &EdgeList, hull: &[InSet], queue: &mut VecDeque<usize>) {
        debug_assert_eq!(self.overlapping.len(), edges.nr_vertices());
        self.dilemma.clear();
        self.dilemma.resize(self.overlapping.len(), 0);
        self.values.resize(self.overlapping.len(), usize::MAX);
        let mut neigh_vals = smallvec::SmallVec::<[usize; 8]>::new();
        for ((ei, ej), i, number, first) in self.to_this.iter() {
            self.values.fill(usize::MAX);
            queue.clear();
            queue.push_back(first);
            let marker = 1 << i;
            let mut nr_found = 1;
            while let Some(v) = queue.pop_front() {
                if hull[v].outside() {
                    continue;
                }
                if self.overlapping[v] & marker != 0 {
                    if self.dilemma[v] & marker == 0 {
                        self.dilemma[v] |= marker;
                        self.values[v] = 0;
                        queue.extend(edges.neighbors_of(v));
                        nr_found += 1;
                    }
                } else {
                    neigh_vals.clear();
                    neigh_vals.extend(edges.neighbors_of(v).filter_map(|n| {
                        let val = self.values[n];
                        (val != usize::MAX).then_some(val)
                    }));
                    neigh_vals.sort();
                    if let Some((_, &val)) = neigh_vals
                        .iter()
                        .dedup_by_with_count(|a, b| a == b)
                        .find(|(count, _)| count > &1)
                    {
                        if self.values[v] > val + 1 {
                            self.dilemma[v] |= marker;
                            self.values[v] = val + 1;
                            queue.extend(edges.neighbors_of(v));
                        }
                    }
                }
            }
            if nr_found < number {
                println!(
                    "upsi! nur {nr_found} von {number} gefunden in Überlappung von {ei} und {ej}"
                );
            }
        }
    }

    /// values is just some memory with length `edges.nr_vertices()`
    /// to be used as temporary space
    pub fn update(
        &mut self,
        edges: &EdgeList,
        lazy_safe: &[u32],
        hull: &[InSet],
        queue: &mut VecDeque<usize>,
    ) {
        self.update_overlapping(lazy_safe, edges);
        self.update_dilemma(edges, hull, queue);
    }
}
