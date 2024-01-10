
use std::collections::VecDeque;

use itertools::izip;
use bitvec::prelude as bv;

use super::{EdgeList, EquivalenceClass};


type CompactCops = usize;

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct CompactCopsIndex(usize);

pub struct SymmetricMap {
    pub edges: EdgeList,
    pub eqivalence: Option<EquivalenceClass>,
}

/// keeps track of all multisets of size `self.nr_cops` and elements in  `self.nr_original_vertices` 
pub struct CopConfigurations {
    map: SymmetricMap,
    nr_cops: usize, //stored in usize, but this can in praxis only be veeeery small
    configurations: Vec<CompactCops>,
}


fn pack(nr_cops: usize, nr_original_vertices: usize, cops: impl Iterator<Item = usize>) -> CompactCops {
    let nr_vertices = nr_original_vertices as CompactCops;
    let mut positions: CompactCops = 0;
    let mut i = 0;
    for cop in cops {
        positions *= nr_vertices;
        debug_assert!((cop as CompactCops) < nr_vertices);
        positions += cop as CompactCops;
        i += 1;
    }
    debug_assert_eq!(i, nr_cops);
    positions
}

impl CopConfigurations {

    fn nr_og_vertices(&self) -> usize {
        self.map.edges.nr_vertices()
    } 

    pub fn nr_configurations(&self) -> usize {
        self.configurations.len()
    }

    pub fn new(map: SymmetricMap, nr_cops: usize) -> Option<Self> {

        fn for_each_configuration(nr_cops: usize, nr_vertices: usize, mut f: impl FnMut(CompactCops)) {
            let mut curr_config = vec![0usize; nr_cops];
            let mut sorted_curr_config = vec![0usize; nr_cops];
            let advance = |config: &mut [_]| -> bool {
                let mut fst_change = 0;
                loop {
                    if config[fst_change] + 1 < nr_vertices {
                        config[fst_change] += 1;
                        break;
                    }
                    fst_change += 1;
                    if fst_change == nr_cops {
                        return false;
                    }
                };
                for i in 0..fst_change {
                    config[i] = config[fst_change];
                }
                true
            };
            loop {
                sorted_curr_config.clone_from(&curr_config);
                sorted_curr_config.sort();
                let packed = pack(nr_cops, nr_vertices, sorted_curr_config.iter().map(|&c| c));
                f(packed);
                if !advance(&mut curr_config) {
                    return;
                }
            }
        }
        
        let nr_vertices = map.edges.nr_vertices();
        let size = {
            let mut i = 0usize;
            for_each_configuration(nr_cops, nr_vertices, |_| i += 1);
            i
        };
        let mut configurations = Vec::new();        
        if configurations.try_reserve_exact(size).is_err() {
            return None;
        }
        for_each_configuration(nr_cops, nr_vertices, |c| configurations.push(c));

        Some(Self { map, nr_cops, configurations })
    }

    pub fn unpack(&self, index: CompactCopsIndex) -> impl ExactSizeIterator<Item = usize> + Clone {
        let mut positions = self.configurations[index.0];
        let nr_vertices = self.nr_og_vertices() as CompactCops;
        (0..self.nr_cops).map(move |_| {
            let val = positions % nr_vertices;
            positions /= nr_vertices;
            val as usize
        })
    }

    pub fn pack(&self, cops: impl Iterator<Item = usize>) -> CompactCopsIndex {
        assert!(self.nr_cops <= 8);
        let mut unpacked = [0usize; 8];
        for (i, pos) in cops.enumerate() {
            debug_assert!((pos as CompactCops) < self.nr_og_vertices());
            debug_assert_ne!(i, self.nr_cops);
            unpacked[i] = pos;
        }
        unpacked[..self.nr_cops].sort();

        let nr_vertices = self.nr_og_vertices() as CompactCops;
        let mut positions: CompactCops = 0;
        for &pos in &unpacked[..self.nr_cops] {
            positions *= nr_vertices;
            positions += pos as CompactCops;
        }
        CompactCopsIndex(self.configurations.binary_search(&positions).unwrap())
    }

    /// returns all cop positions reachable in a single lazy-cop move from positions, except the do-nothing move
    pub fn lazy_cop_moves_from<'a>(&'a self, positions: CompactCopsIndex) 
    -> impl Iterator<Item = CompactCopsIndex> + 'a + Clone 
    {        assert!(self.nr_cops <= 8);
        let mut unpacked = [0usize; 8];
        for (storage, cop_pos) in izip!(&mut unpacked, self.unpack(positions)) {
            *storage = cop_pos;
        }

        let iter = (0..self.nr_cops).flat_map(move |i| {
            let cop_i_pos = unpacked[i];
            self.map.edges.neighbors_of(cop_i_pos).map(move |n| 
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
    fn new(cop_moves: &CopConfigurations) -> Option<Self> {
        let nr_entries = cop_moves.nr_configurations().checked_mul(cop_moves.nr_og_vertices())?;
        let vec_data_len = (nr_entries + 31) / 32;
        let mut bit_vec_data = Vec::new();
        if bit_vec_data.try_reserve(vec_data_len).is_err() {
            return None;
        }
        bit_vec_data.resize(vec_data_len, u32::MAX);
        let Ok(safe) = bv::BitVec::try_from_vec(bit_vec_data) else {
            return None;
        };
        let nr_map_vertices = cop_moves.nr_og_vertices();

        Some(Self { safe, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self, ) -> usize {
        self.nr_map_vertices
    }

    fn robber_indices_at(&self, index: CompactCopsIndex) -> std::ops::Range<usize> {
        let start = index.0 * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        start..stop
    }

    pub fn robber_safe_at(&self, index: CompactCopsIndex) -> impl ExactSizeIterator<Item = bool> + '_ + Clone {
        let range = self.robber_indices_at(index);
        range.map(|i| self.safe[i])
    }
}

pub enum BruteForceResult {
    None,
    Error(String),
    /// stores for how many cops result was computed and for a graph over how many vertices
    CopsWin(usize, usize),
    /// stores for how many cops result was computed
    RobberWins(usize, SafeRobberPositions, CopConfigurations),
}

/// algorithm 2.2
pub fn compute_safe_robber_positions<'a>(nr_cops: usize, map: SymmetricMap) -> BruteForceResult {
    let Some(cop_moves) = CopConfigurations::new(map, nr_cops) else {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Cops passen nicht in Int)".to_owned());
    };
    let Some(mut f) = SafeRobberPositions::new(&cop_moves) else {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Räuber-Gewinnfunktion zu groß)".to_owned());
    };

    let mut queue = VecDeque::new();
    if queue.try_reserve(cop_moves.nr_configurations()).is_err() {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Queue zu lang)".to_owned());
    }

    for index in (0..cop_moves.nr_configurations()).map(|i| CompactCopsIndex(i)) {
        //line 2
        let robber_range = f.robber_indices_at(index);
        for cop_pos in cop_moves.unpack(index) {
            f.safe.set(robber_range.start + cop_pos, false);
            for n in cop_moves.map.edges.neighbors_of(cop_pos) {
                f.safe.set(robber_range.start + n, false);
            }
        }
        
        //line 3
        queue.push_back(index);
    }

    let mut safe_robber_neighbors = vec![false; cop_moves.map.edges.nr_vertices()];
    let mut f_temp = vec![false; cop_moves.map.edges.nr_vertices()];

    //lines 4 + 5
    while let Some(cop_positions) = queue.pop_back() {
        //line 6
        for (stored, safe) in izip!(&mut safe_robber_neighbors, f.robber_safe_at(cop_positions)) {
            *stored = safe;
        }
        for (v, safe) in f.robber_safe_at(cop_positions).enumerate() {
            if safe {
                for n in cop_moves.map.edges.neighbors_of(v) {
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
                return BruteForceResult::CopsWin(nr_cops, cop_moves.map.edges.nr_vertices());
            }
        }

    }

    BruteForceResult::RobberWins(nr_cops, f, cop_moves)
}





