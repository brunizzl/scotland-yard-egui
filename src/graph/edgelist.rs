
use std::collections::VecDeque;
use std::iter::{Map, ExactSizeIterator};
use std::slice::{Chunks, Iter};

use itertools::Itertools;

#[derive(Clone, Copy, PartialEq, PartialOrd, Eq, Ord)]
pub struct Index {
    val: u32,
}

impl Index {
    pub const fn new(x: usize) -> Self {
        Self { val: x as u32 }
    }

    pub const NONE: Index = Self::new(u32::MAX as usize);

    pub const fn is_none(&self) -> bool { self.val == Self::NONE.val }

    pub const fn get(&self) -> Option<usize> {
        if self.is_none() { None } else { Some(self.val as usize) }
    }
}

impl Default for Index {
    fn default() -> Self {
        Index::NONE
    }
}

impl From<usize> for Index {
    fn from(value: usize) -> Self {
        Self::new(value)
    }
}



fn find(xs: &mut [Index], y: Index) -> Option<&mut Index> {
    for x in xs {
        if *x == y {
            return Some(x);
        }
    }
    None
}

fn next_unused(xs: &mut [Index]) -> Option<&mut Index> {
    find(xs, Index::NONE)
}

fn take_active(xs: &[Index]) -> Map<Iter<'_, Index>, fn(&Index) -> usize> {
    let end = xs.iter().position(Index::is_none).unwrap_or(xs.len());
    debug_assert!(xs[end..].iter().all(Index::is_none));
    xs[..end].iter().map(|n| n.val as usize)
}

/// works great if most vertices have degree close to the maximum degree, as every vertex
/// allocates the same space for potential neighbors
/// CAUTION: if not most vertices have close to the maximum degree (e.g. with one central vertex),
/// this structure is way worse than using a vector for each vertices' neighbors.
#[derive(Clone)]
pub struct EdgeList {
    next_shrink_check_len: usize,
    max_neighbors: usize,
    length: usize,
    entries: Vec<Index>,
}

impl EdgeList {
    pub fn nr_vertices(&self) -> usize {
        self.length
    }

    /// upper bound for edge indices
    pub fn used_space(&self) -> usize {
        self.entries.len()
    }

    #[allow(dead_code)]
    pub fn max_neighbors(&self) -> usize {
        self.max_neighbors
    }

    pub fn max_degree(&self) -> usize {
        self.neighbors().fold(0, |acc, neighs| usize::max(acc, neighs.len()))
    }

    #[allow(dead_code)]
    pub fn min_degree(&self) -> usize {
        self.neighbors().fold(usize::MAX, |acc, neighs| usize::min(acc, neighs.len()))
    }

    pub fn new(max_neighbors: usize, length: usize) -> Self {
        let vec_len = max_neighbors * length;
        let entries = vec![Index::NONE; vec_len];
        let next_shrink_check_len = (length * 3) / 2 + 10;
        Self { next_shrink_check_len, max_neighbors, length, entries }
    }

    pub fn from_iter(iter: impl ExactSizeIterator<Item = impl Iterator<Item = usize> + Clone>, 
        max_neighbors: usize) -> Self 
    {
        let length = iter.len();
        let mut res = Self::new(max_neighbors, length);
        for (slots, neighs) in res.potential_neighbors_mut().zip(iter) {
            debug_assert!(neighs.clone().count() <= max_neighbors);
            for (slot, neigh) in slots.iter_mut().zip(neighs) {
                *slot = Index::new(neigh);
            }
        }
        res
    }

    pub fn empty() -> Self {
        Self::new(6, 0)
    }

    pub fn increase_capacity(&mut self, new_max_neighbors: usize) {
        debug_assert!(self.max_neighbors < new_max_neighbors);
        let new_vec_len = self.length * new_max_neighbors;
        let mut new_entries = vec![Index::NONE; new_vec_len];
        for v in 0..self.length {
            for i in 0..self.max_neighbors {
                let old_index = v * self.max_neighbors + i;
                let new_index = v * new_max_neighbors + i;
                new_entries[new_index] = self.entries[old_index];
            }
        }
        self.max_neighbors = new_max_neighbors;
        self.entries = new_entries;
    }

    pub fn maybe_shrink_capacity(&mut self, allowed_excess: usize) {
        let new_max_neighs = self.max_degree() + allowed_excess;
        if new_max_neighs < self.max_neighbors {            
            let mut old_start = 0;
            let mut old_end = self.max_neighbors;
            let mut new_start = 0;
            let mut new_end = new_max_neighs;
            for _v in 0..self.length {
                for (old_i, new_i) in (old_start..old_end).zip(new_start..new_end) {
                    self.entries[new_i] = self.entries[old_i];
                }
                old_start = old_end;
                old_end += self.max_neighbors;
                new_start = new_end;
                new_end += new_max_neighs;
            }
            self.max_neighbors = new_max_neighs;
            self.entries.truncate(new_max_neighs * self.length);
        }
    }

    pub fn push(&mut self) -> usize {
        let new_index = self.length;
        let new_len = self.length + 1;
        if self.next_shrink_check_len <= new_len {
            self.maybe_shrink_capacity(2);
            self.next_shrink_check_len = (new_len * 3) / 2;
        }
        self.entries.resize(self.entries.len() + self.max_neighbors, Index::NONE);
        self.length = new_len;
        new_index
    }

    fn potential_neighbors(&self) -> Chunks<'_, Index> {
        self.entries.chunks(self.max_neighbors)
    }

    pub fn neighbors(&self) -> impl ExactSizeIterator<Item = impl ExactSizeIterator<Item = usize> + '_ + Clone> + '_ + Clone {
        self.potential_neighbors().map(take_active)
    }

    #[allow(dead_code)]
    pub fn count_entries(&self) -> usize {
        self.neighbors().fold(0, |acc, neigh| acc + neigh.len())
    }

    pub fn potential_neighbors_mut(&mut self) -> std::slice::ChunksMut<'_, Index> {
        self.entries.chunks_mut(self.max_neighbors)
    }

    #[allow(dead_code)]
    pub fn neighbors_mut(&mut self) -> 
        impl ExactSizeIterator<Item = &mut [Index]> + '_ 
    {
        self.potential_neighbors_mut().map(|chunk| {            
            let end = chunk.iter().position(Index::is_none).unwrap_or(chunk.len());
            debug_assert!(chunk[end..].iter().all(Index::is_none));
            &mut chunk[..end]
        })
    }

    pub fn neighbors_of(&self, v: usize) -> Map<Iter<'_, Index>, fn(&Index) -> usize> {

        debug_assert!(v < self.length);
        let start = self.max_neighbors * v;
        let storage_end = self.max_neighbors * (v + 1);
        take_active(&self.entries[start..storage_end])
    }

    pub fn neighbors_mut_of(&mut self, v: usize) -> &mut [Index] {
        debug_assert!(v < self.length);
        let start = self.max_neighbors * v;
        let end = self.max_neighbors * (v + 1);
        &mut self.entries[start..end]
    }

    pub fn has_directed_edge(&self, v1: usize, v2: usize) -> bool {
        self.neighbors_of(v1).contains(&v2.into())
    }

    pub fn has_edge(&self, v1: usize, v2: usize) -> bool {
        let res = self.has_directed_edge(v1, v2);
        debug_assert!(res == self.has_directed_edge(v2, v1));
        res
    }

    /// depends on self.max_neighbors
    pub fn directed_index(&self, v1: usize, v2: usize) -> usize {
        self.neighbors_of(v1).position(|v| v == v2).map(|v2_pos| {
            let v1_neighs_range_start = v1 * self.max_neighbors;
            v1_neighs_range_start + v2_pos
        })
        .unwrap()
    }

    #[allow(dead_code)]
    pub fn all_valid_edge_indices(&self) -> Vec<bool> {
        self.entries.iter().map(|e| !e.is_none()).collect()
    }

    pub fn edge_at_index(&self, index: usize) -> (usize, usize) {
        let v1 = index / self.max_neighbors;
        let v2_pos = index % self.max_neighbors;
        let v2 = self.neighbors_of(v1).nth(v2_pos).unwrap();
        (v1, v2)
    }

    pub fn add_directed_edge(&mut self, v1: usize, v2: usize) {
        debug_assert!(!self.has_directed_edge(v1, v2));
        let i2 = Index::new(v2);
        loop {
            if let Some(e1) = next_unused(self.neighbors_mut_of(v1)) {
                *e1 = i2;
                break;
            }
            //guarantees next iteration to exit
            self.increase_capacity(self.max_neighbors + 2);
        }
    }

    pub fn add_edge(&mut self, v1: usize, v2: usize) {
        self.add_directed_edge(v1, v2);
        self.add_directed_edge(v2, v1);
    }

    pub fn remove_edge(&mut self, v1: usize, v2: usize) {
        debug_assert!(self.has_edge(v1, v2));
        let i1 = Index::new(v1);
        let i2 = Index::new(v2);
        if let Some(n1) = find(self.neighbors_mut_of(v1), i2) {
            *n1 = Index::NONE;
        }
        if let Some(n2) = find(self.neighbors_mut_of(v2), i1) {
            *n2 = Index::NONE;
        }
        //keep NONE at end
        self.neighbors_mut_of(v1).sort();
        self.neighbors_mut_of(v2).sort();
    }    

    pub fn add_path_edges_ref<'a>(&mut self, path: impl Iterator<Item = &'a usize>) {
        for (&v1, &v2) in path.tuple_windows() {
            self.add_edge(v1, v2);
        }
    }    

    pub fn add_path_edges(&mut self, path: impl Iterator<Item= usize>) {
        for (v1, v2) in path.tuple_windows() {
            self.add_edge(v1, v2);
        }
    }

    /// assumes undirected edges, iterates over every edge only in one direction,
    /// e.g. one can assume f is called with ordered parameters (fst <= snd)
    pub fn for_each_edge(&self, mut f: impl FnMut(usize, usize)) {
        for (v1, neighs) in self.potential_neighbors().enumerate() {
            for &n in neighs {
                let v2 = n.val as usize;
                //no edge is iterated twice and v2 is not Index::NONE
                if v2 < v1 {
                    f(v1, v2)
                }
            }
        }
    }

    /// everything in queue is starting point and expected to already have the correct distance
    pub fn calc_distances_to_with<F>(&self, distances: &mut [isize], mut select: F, queue: &mut VecDeque<usize>) 
    where F: FnMut(usize, &[isize]) -> bool
    {
        debug_assert_eq!(distances.len(), self.nr_vertices());
        let mut local_queue = std::mem::take(queue);
        while let Some(v) = local_queue.pop_front() {
            let dist = distances[v];
            for n in self.neighbors_of(v) {
                if select(n, distances) && distances[n] > dist + 1 {
                    distances[n] = dist + 1;
                    local_queue.push_back(n);
                }
            }
        }
        *queue = local_queue;
    }

    /// everything in queue is starting point and expected to already have the correct distance
    pub fn calc_distances_to(&self, queue: &mut VecDeque<usize>, distances: &mut [isize]) {
        self.calc_distances_to_with(distances, |_, _| true, queue)
    }

    /// paintbucket tool, all in queue are starting vertices, is_old decides if a color is changed to new
    /// and can therefore also access colors as snd parameter
    pub fn recolor_region_with<Color, F>(&self, new: Color, colors: &mut [Color], 
        mut is_old: F, queue: &mut VecDeque<usize>) -> usize 
    where Color: Eq + Clone + std::fmt::Debug,
          F: FnMut(usize, &[Color]) -> bool
    {        
        debug_assert_eq!(self.nr_vertices(), colors.len());
        let mut nr_colored = 0;
        let mut local_queue = std::mem::take(queue);
        while let Some(v) = local_queue.pop_front() {
            debug_assert_eq!(colors[v], new);
            for n in self.neighbors_of(v) {
                //seems redundant at first for the simple recolor_region case, 
                //but prevents getting stuck in the case of old == new
                if colors[n] == new { 
                    continue;
                }
                if is_old(n, colors) {
                    nr_colored += 1;
                    colors[n] = new.clone();
                    local_queue.push_back(n);
                }
            }
        }
        *queue = local_queue;
        nr_colored
    }

    /// paintbucket tool, all in queue are starting vertices
    pub fn recolor_region<Color>(&self, (old, new): (Color, Color), 
        colors: &mut [Color], queue: &mut VecDeque<usize>) -> usize 
    where Color: Eq + Clone + std::fmt::Debug
    {
        debug_assert_ne!(old, new); //technically not needed, but why would anyone want that?
        self.recolor_region_with(new, colors, |v, cs| cs[v] == old, queue)
    }

    pub fn find_local_minimum(&self, mut potential: impl FnMut(usize) -> f32, node_hint: usize) -> (usize, f32) {
        let mut nearest = node_hint;
        let mut smallest_pot = potential(node_hint);
        let mut maybe_neighbor_better = true;
        while maybe_neighbor_better {
            maybe_neighbor_better = false;
            for neigh in self.neighbors_of(nearest) {
                let neigh_pot = potential(neigh);
                if neigh_pot < smallest_pot {
                    nearest = neigh;
                    smallest_pot = neigh_pot;
                    maybe_neighbor_better = true;
                }
            }
        }
        (nearest, smallest_pot)
    } 
} //impl EdgeList



