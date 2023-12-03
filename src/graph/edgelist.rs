
use std::collections::VecDeque;
use std::iter::Map;
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
    debug_assert!(!xs[..end].iter().any(Index::is_none));
    xs[..end].iter().map(|n| n.val as usize)
}

/// works great if most vertices have degree close to the maximum degree, as every vertex
/// allocates the same space for potential neighbors
/// CAUTION: if not most vertices have close to the maximum degree (e.g. with one central vertex),
/// this structre is way worse than using a vector for each vertices' neighbors.
pub struct EdgeList {
    next_shrink_check_len: usize,
    max_neighbors: usize,
    length: usize,
    entries: Vec<Index>,
}

impl EdgeList {
    pub fn len(&self) -> usize {
        self.length
    }

    #[allow(dead_code)]
    pub fn max_neighbors(&self) -> usize {
        self.max_neighbors
    }

    pub fn max_degree(&self) -> usize {
        self.neighbors().fold(0, |a, b| usize::max(a, b.len()))
    }

    pub fn new(max_neighbors: usize, length: usize) -> Self {
        let vec_len = max_neighbors * length;
        let entries = vec![Index::NONE; vec_len];
        let next_shrink_check_len = (length * 3) / 2 + 10;
        Self { next_shrink_check_len, max_neighbors, length, entries }
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

    pub fn push(&mut self) {
        let new_len = self.length + 1;
        if self.next_shrink_check_len <= new_len {
            self.maybe_shrink_capacity(2);
            self.next_shrink_check_len = (new_len * 3) / 2;
        }
        self.entries.resize(self.entries.len() + self.max_neighbors, Index::NONE);
        self.length = new_len;
    }

    fn potential_neighbors(&self) -> Chunks<'_, Index> {
        self.entries.chunks(self.max_neighbors)
    }

    pub fn neighbors(&self) -> Map<Chunks<'_, Index>, fn(&[Index]) -> Map<Iter<'_, Index>, fn(&Index) -> usize>> {
        self.potential_neighbors().map(take_active)
    }

    pub fn neighbors_mut(&mut self) -> std::slice::ChunksMut<'_, Index> {
        self.entries.chunks_mut(self.max_neighbors)
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

    pub fn has_edge(&self, v1: usize, v2: usize) -> bool {
        let res = self.neighbors_of(v1).contains(&v2.into());
        debug_assert!(res == self.neighbors_of(v2).contains(&v1.into()));
        res
    }

    pub fn add_edge(&mut self, v1: usize, v2: usize) {
        debug_assert!(!self.has_edge(v1, v2));
        debug_assert_ne!(v1, v2);
        let i1 = Index::new(v1);
        let i2 = Index::new(v2);
        loop {
            if let Some(e1) = next_unused(self.neighbors_mut_of(v1)) {
                *e1 = i2;
                break;
            }
            //guarantees next iteration to exit
            self.increase_capacity(self.max_neighbors + 2);
        }
        loop {
            if let Some(e2) = next_unused(self.neighbors_mut_of(v2)) {
                *e2 = i1;
                break;
            }
            //guarantees next iteration to exit
            self.increase_capacity(self.max_neighbors + 2);
        }
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

    pub fn add_path_edges<'a>(&mut self, path: impl Iterator<Item = &'a usize>) {
        for (&v1, &v2) in path.tuple_windows() {
            self.add_edge(v1, v2);
        }
    }

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
    pub fn calc_distances_to(&self, queue: &mut VecDeque<usize>, distances: &mut Vec<isize>) {
        while let Some(v) = queue.pop_front() {
            let dist = distances[v];
            for n in self.neighbors_of(v) {
                if distances[n] > dist + 1 {
                    distances[n] = dist + 1;
                    queue.push_back(n);
                }
            }
        }
    }

    /// paintbucket tool, all in queue are starting vertices
    pub fn recolor_region<Color: Eq + Clone>(&self, (old, new): (Color, Color), 
        colors: &mut [Color], queue: &mut VecDeque<usize>) {
        
        debug_assert_eq!(self.len(), colors.len());
        while let Some(node) = queue.pop_front() {
            for n in self.neighbors_of(node) {
                if colors[n] == old {
                    colors[n] = new.clone();
                    queue.push_back(n);
                }
            }
        }
    }    
} //impl EdgeList



