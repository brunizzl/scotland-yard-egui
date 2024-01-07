
use std::collections::VecDeque;

use itertools::izip;
use bitvec::prelude as bv;

use super::EdgeList;





pub struct CartesianGraphProduct<'a> {
    original_edges: &'a EdgeList,
    nr_cops: usize, //stored in usize, but this can in praxis only be veeeery small
}

impl<'a> CartesianGraphProduct<'a> {

    pub fn nr_product_vertices(&self) -> usize {
        self.original_edges.nr_vertices().pow(self.nr_cops as u32)
    }

    pub fn new(original_edges: &'a EdgeList, nr_cops: usize) -> Option<Self> {
        //only start computation when all vertices' indices can be converted to usize
        let _size = (original_edges.nr_vertices() as usize).checked_pow(nr_cops as u32)?;
        Some(Self { original_edges, nr_cops })
    }

    pub fn unpack(&self, mut positions: u64) -> impl ExactSizeIterator<Item = usize> + Clone {
        let nr_vertices = self.original_edges.nr_vertices() as u64;
        (0..self.nr_cops).map(move |_| {
            let val = positions % nr_vertices;
            positions /= nr_vertices;
            val as usize
        })
    }

    pub fn pack(&self, cops: impl Iterator<Item = usize>) -> u64 {
        let nr_vertices = self.original_edges.nr_vertices() as u64;
        let mut positions: u64 = 0;
        let mut i = 0;
        for cop in cops {
            positions *= nr_vertices;
            debug_assert!((cop as u64) < nr_vertices);
            positions += cop as u64;
            i += 1;
        }
        assert_eq!(i, self.nr_cops);
        positions
    }

    /// returns all cop positions reachable in a single lazy-cop move from positions, except the do-nothing move
    pub fn lazy_cop_moves_from(&self, positions: u64) -> impl Iterator<Item = u64> + '_ + Clone {
        assert!(self.nr_cops <= 8);
        let mut unpacked = [0usize; 8];
        for (storage, cop_pos) in izip!(&mut unpacked, self.unpack(positions)) {
            *storage = cop_pos;
        }

        let iter = (0..self.nr_cops).flat_map(move |i| {
            let cop_i_pos = unpacked[i];
            self.original_edges.neighbors_of(cop_i_pos).map(move |n| 
                self.pack((0..self.nr_cops).map(|j| 
                    if i == j { n } else { unpacked[j] }
                ))
            )
        });
        
        iter
    }

}

pub struct SafeRobberPositions {
    safe: bv::BitVec<u32>,
    nr_map_vertices: usize,
}

impl SafeRobberPositions {
    fn new(cop_moves: &CartesianGraphProduct<'_>) -> Option<Self> {
        let nr_entries = cop_moves.nr_product_vertices().checked_mul(cop_moves.original_edges.nr_vertices())?;
        let vec_data_len = (nr_entries + 31) / 32;
        let mut bit_vec_data = Vec::new();
        if bit_vec_data.try_reserve(vec_data_len).is_err() {
            return None;
        }
        bit_vec_data.resize(vec_data_len, u32::MAX);
        let Ok(safe) = bv::BitVec::try_from_vec(bit_vec_data) else {
            return None;
        };
        let nr_map_vertices = cop_moves.original_edges.nr_vertices();

        Some(Self { safe, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self, ) -> usize {
        self.nr_map_vertices
    }

    fn robber_indices_at(&self, positions: u64) -> std::ops::Range<usize> {
        let start = positions as usize * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        start..stop
    }

    pub fn robber_safe_at(&self, positions: u64) -> impl ExactSizeIterator<Item = bool> + '_ + Clone {
        let range = self.robber_indices_at(positions);
        range.map(|i| self.safe[i])
    }
}

pub enum BruteForceResult {
    None,
    Error(String),
    /// stores for how many cops result was computed and for a graph over how many vertices
    CopsWin(usize, usize),
    /// stores for how many cops result was computed
    RobberWins(usize, SafeRobberPositions),
}

/// algorithm 2.2
pub fn compute_safe_robber_positions<'a>(nr_cops: usize, graph: &'a EdgeList) -> BruteForceResult {
    let Some(cop_moves) = CartesianGraphProduct::new(graph, nr_cops) else {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Cops passen nicht in Int)".to_owned());
    };
    let Some(mut f) = SafeRobberPositions::new(&cop_moves) else {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Räuber-Gewinnfunktion zu groß)".to_owned());
    };

    let mut queue = VecDeque::new();
    if queue.try_reserve(cop_moves.nr_product_vertices()).is_err() {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Queue zu lang)".to_owned());
    }

    for cop_positions in 0u64..(cop_moves.nr_product_vertices() as u64) {
        //line 2
        let robber_range = f.robber_indices_at(cop_positions);
        for cop_pos in cop_moves.unpack(cop_positions) {
            f.safe.set(robber_range.start + cop_pos, false);
            for n in graph.neighbors_of(cop_pos) {
                f.safe.set(robber_range.start + n, false);
            }
        }
        
        //line 3
        queue.push_back(cop_positions);
    }

    let mut safe_robber_neighbors = vec![false; graph.nr_vertices()];
    let mut f_temp = vec![false; graph.nr_vertices()];

    //lines 4 + 5
    while let Some(cop_positions) = queue.pop_back() {
        //line 6
        for (stored, safe) in izip!(&mut safe_robber_neighbors, f.robber_safe_at(cop_positions)) {
            *stored = safe;
        }
        for (v, safe) in f.robber_safe_at(cop_positions).enumerate() {
            if safe {
                for n in graph.neighbors_of(v) {
                    safe_robber_neighbors[n] = true;
                }
            }
        }

        //line 7
        for neigh_cop_positions in cop_moves.lazy_cop_moves_from(cop_positions) {
            //line 8
            let mut f_temp_changed = false;
            for (stored, b1, &b2) in izip!(&mut f_temp, f.robber_safe_at(neigh_cop_positions), &safe_robber_neighbors) {
                f_temp_changed |= b1 && !b2;
                *stored = b1 && b2;
            }

            //line 9
            if f_temp_changed {
                //line 10
                let f_range = f.robber_indices_at(neigh_cop_positions);
                for (i, &val) in izip!(f_range, &f_temp) {
                    f.safe.set(i, val);
                }
                //line 11
                if queue.try_reserve(1).is_err() {
                    return BruteForceResult::Error("Zu wenig Speicherplatz (Queue zu lang)".to_owned());
                }
                queue.push_back(neigh_cop_positions);
            }

            //lines 13 + 14 + 15
            if f.robber_safe_at(neigh_cop_positions).all(|x| !x) {
                return BruteForceResult::CopsWin(nr_cops, graph.nr_vertices());
            }
        }

    }

    BruteForceResult::RobberWins(nr_cops, f)
}





