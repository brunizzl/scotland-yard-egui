
use std::collections::VecDeque;

use itertools::{izip, Itertools};
use bitvec::prelude as bv;

use super::{EdgeList, EquivalenceClasses};
use crate::geo::Matrix3x3;


#[derive(Clone, Copy, PartialEq, Eq)]
struct CompactCops {
    fst_cop: usize,
    rest_cops: usize,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CompactCopsIndex {
    fst_index: usize,
    rest_index: usize,
 }

pub struct SymmetricMap {
    pub edges: EdgeList,
    pub eqivalence: Option<EquivalenceClasses>,
}

/// keeps track of all multisets of size `self.nr_cops` and elements in  `self.nr_original_vertices` 
pub struct CopConfigurations {
    map: SymmetricMap,
    nr_cops: usize, //stored in usize, but this can in praxis only be veeeery small
    configurations: std::collections::BTreeMap<usize, Vec<usize>>,
}


fn pack_rest(nr_vertices: usize, other_cops: &[usize]) -> usize {
    let mut positions: usize = 0;
    for &cop in other_cops.iter() {
        positions *= nr_vertices;
        debug_assert!(cop < nr_vertices);
        positions += cop;
    }    
    positions
}

impl CopConfigurations {

    fn nr_og_vertices(&self) -> usize {
        self.map.edges.nr_vertices()
    } 

    fn nr_configurations(&self) -> usize {
        self.configurations.iter().map(|(_, c)| c.len()).sum()
    }

    pub fn new(map: SymmetricMap, nr_cops: usize) -> Option<Self> {
        let nr_vertices = map.edges.nr_vertices();
        let curr_rest_config_size = 0.max(nr_cops as isize - 1) as usize;
        let mut curr_rest_config = vec![0usize; curr_rest_config_size];
        let advance = |rest_config: &mut [_]| -> bool {
            let mut fst_change = rest_config.len() - 1;
            loop {
                if rest_config[fst_change] + 1 < nr_vertices {
                    rest_config[fst_change] += 1;
                    break;
                }
                if fst_change == 0 {
                    return false;
                }
                fst_change -= 1;
            };
            for i in fst_change..rest_config.len() {
                rest_config[i] = rest_config[fst_change];
            }
            true
        };

        let mut configurations = std::collections::BTreeMap::new();  

        let mut add_configs_for_first = |fst_cop: usize| {
            curr_rest_config.iter_mut().for_each(|c| *c = fst_cop);
            let mut new_configuration = Vec::new();
            if nr_cops > 1 {
                loop {
                    debug_assert!(curr_rest_config.iter().tuple_windows().all(|(a, b)| a <= b));
                    let packed = pack_rest(map.edges.nr_vertices(), &curr_rest_config);
                    if let Err(_) = new_configuration.try_reserve(1) {
                        return false;
                    }
                    new_configuration.push(packed);
                    if !advance(&mut curr_rest_config) {
                        break;
                    }
                }
            } else {
                new_configuration.push(0);
            }
            let old = configurations.insert(fst_cop, new_configuration);
            debug_assert!(old.is_none());
            true
        };
        if let Some(equiv) = &map.eqivalence {
            for &fst_cop in equiv.class_representatives() {
                if !add_configs_for_first(fst_cop) {
                    return None;
                }
            }
        }
        else {
            for fst_cop in 0..map.edges.nr_vertices() {
                if !add_configs_for_first(fst_cop) {
                    return None;
                }
            }
        }

        Some(Self { map, nr_cops, configurations })
    }

    pub fn rest_positions_at(&self, index: CompactCopsIndex) -> usize {
        let configs_part = self.configurations.get(&index.fst_index).unwrap();
        configs_part[index.rest_index]
    }

    pub fn unpack(&self, index: CompactCopsIndex) -> impl Iterator<Item = usize> + Clone {
        let mut positions = self.rest_positions_at(index);
        let nr_vertices = self.nr_og_vertices();
        
        std::iter::once(index.fst_index)
        .chain((1..self.nr_cops).map(move |_| {
            let val = positions % nr_vertices;
            positions /= nr_vertices;
            val as usize
        }))
    }

    pub fn pack(&self, cops: impl Iterator<Item = usize>) -> (Matrix3x3, CompactCopsIndex) {
        assert!(self.nr_cops <= 8);
        let mut unpacked = [0usize; 8];
        for (i, pos) in cops.enumerate() {
            debug_assert!(pos < self.nr_og_vertices());
            debug_assert_ne!(i, self.nr_cops);
            unpacked[i] = pos;
        }
        let unpacked = &mut unpacked[..self.nr_cops];
        let rotation = if let Some(e) = &self.map.eqivalence {
            e.transform_all(&self.map.edges, unpacked)
        } else {
            unpacked.sort();
            Matrix3x3::IDENTITY
        };
        debug_assert!(unpacked.iter().tuple_windows().all(|(a, b)| a <= b));

        let fst_cop = unpacked[0];
        let rest_cops = &unpacked[1..];

        let packed = pack_rest(self.map.edges.nr_vertices(), rest_cops);
        let configs_part = self.configurations.get(&fst_cop).unwrap();
        let rest_pos = configs_part.binary_search(&packed);
        (rotation, CompactCopsIndex {
            fst_index: fst_cop,
            rest_index: rest_pos.unwrap(),
        })        
    }

    /// returns all cop positions reachable in a single lazy-cop move from positions, except the do-nothing move
    pub fn lazy_cop_moves_from<'a>(&'a self, positions: CompactCopsIndex) 
    -> impl Iterator<Item = (Matrix3x3, CompactCopsIndex)> + 'a + Clone 
    {
        assert!(self.nr_cops <= 8);
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
    safe: std::collections::BTreeMap<usize, bv::BitVec<u32>>,
    nr_map_vertices: usize,
}

struct RobberPosRange {
    fst_index: usize,
    range: std::ops::Range<usize>,
}

struct RobberPosIndex {
    fst_index: usize,
    rest_index: usize,
}

impl RobberPosRange {
    fn at(&self, i: usize) -> RobberPosIndex {
        RobberPosIndex { 
            fst_index: self.fst_index,
            rest_index: self.range.start + i,
        }
    }
}

impl SafeRobberPositions {
    fn new(cop_moves: &CopConfigurations) -> Option<Self> {
        let mut safe = std::collections::BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len().checked_mul(cop_moves.nr_og_vertices())?;
            let vec_data_len = (nr_entries + 31) / 32;
            let mut bit_vec_data = Vec::<u32>::new();
            if bit_vec_data.try_reserve(vec_data_len).is_err() {
                return None;
            }
            bit_vec_data.resize(vec_data_len, u32::MAX);
            let Ok(alloced_safe_data) = bv::BitVec::try_from_vec(bit_vec_data) else {
                return None;
            };
            let old = safe.insert(fst_index, alloced_safe_data);
            debug_assert!(old.is_none());
        }

        let nr_map_vertices = cop_moves.nr_og_vertices();

        Some(Self { safe, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self, ) -> usize {
        self.nr_map_vertices
    }

    fn robber_indices_at(&self, index: CompactCopsIndex) -> RobberPosRange {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        RobberPosRange {
            fst_index: index.fst_index,
            range: start..stop,
        }
    }

    pub fn robber_safe_when(&self, index: CompactCopsIndex) -> impl ExactSizeIterator<Item = bool> + '_ + Clone {
        let RobberPosRange{ fst_index, range } = self.robber_indices_at(index);
        let safe_part = self.safe.get(&fst_index).unwrap();
        range.map(move |i| safe_part[i])
    }

    fn mark_robber_at(&mut self, index: RobberPosIndex, value: bool) {
        let safe_part = self.safe.get_mut(&index.fst_index).unwrap();
        safe_part.set(index.rest_index, value);
    }

    fn robber_is_safe_at(&self, index: RobberPosIndex) -> bool {
        let safe_part = self.safe.get(&index.fst_index).unwrap();
        safe_part[index.rest_index]
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

    for (&fst_index, sub_configs) in &cop_moves.configurations {
        for (rest_index, _packed_rest_cops) in izip!(0.., sub_configs) {
            let index = CompactCopsIndex { fst_index, rest_index };
            
            debug_assert!({
                let unpacked = cop_moves.unpack(index);
                let (_, packed) = cop_moves.pack(unpacked);
                debug_assert_eq!(index, packed);
                true
            });

            //line 2
            let robber_range = f.robber_indices_at(index);
            for cop_pos in cop_moves.unpack(index) {
                f.mark_robber_at(robber_range.at(cop_pos), false);
                for n in cop_moves.map.edges.neighbors_of(cop_pos) {
                    f.mark_robber_at(robber_range.at(n), false);
                }
            }
            
            //line 3
            queue.push_back(index);            
        } 
    }

    //return BruteForceResult::RobberWins(nr_cops, f, cop_moves);

    let nr_map_vertices = cop_moves.map.edges.nr_vertices();
    let mut safe_robber_neighbors = vec![false; nr_map_vertices];
    let mut f_temp = vec![false; nr_map_vertices];

    //lines 4 + 5
    while let Some(cop_positions) = queue.pop_back() {
        //line 6
        for (stored, is_safe) in izip!(&mut safe_robber_neighbors, f.robber_safe_when(cop_positions)) {
            *stored = is_safe;
        }
        for (v, is_safe) in f.robber_safe_when(cop_positions).enumerate() {
            if is_safe {
                for n in cop_moves.map.edges.neighbors_of(v) {
                    safe_robber_neighbors[n] = true;
                }
            }
        }

        //line 7
        for (neigh_rotate, rotated_neigh_cop_positions) in cop_moves.lazy_cop_moves_from(cop_positions) {
            //line 8
            let mut f_neighbor_changed = false;
            //rotate back brings the maybe rotated cop placement back s.t. it is actually obtainable by 
            //only moving one cop.
            let rotate_back = neigh_rotate.transposed();
            if let Some(e) = &cop_moves.map.eqivalence {
                //guarantee that rotations do the right thing
                debug_assert!({
                    let mut unpacked_curr = [0usize; 8];
                    for (storage, pos) in izip!(&mut unpacked_curr, cop_moves.unpack(cop_positions)) {
                        *storage = pos;
                    }
                    let unpacked_curr = &mut unpacked_curr[..cop_moves.nr_cops];
                    let mut found_step = false;
                    for rotated_neigh_pos in cop_moves.unpack(rotated_neigh_cop_positions) {
                        let unrotated = e.apply_transform(&cop_moves.map.edges, &rotate_back, rotated_neigh_pos);
                        let rerotated = e.apply_transform(&cop_moves.map.edges, &neigh_rotate, unrotated);
                        debug_assert_eq!(rerotated, rotated_neigh_pos);
                        if let Some(i) = unpacked_curr.iter().position(|&c| c == unrotated) {
                            unpacked_curr[i] = usize::MAX;
                        }
                        else {
                            debug_assert!(!found_step);
                            debug_assert!(
                                cop_moves.map.edges
                                .neighbors_of(unrotated)
                                .any(|n| unpacked_curr.contains(&n))
                            );
                            found_step = true;
                        }
                    }
                    found_step
                }, "rotate_back is wrong");

                let f_indices = f.robber_indices_at(rotated_neigh_cop_positions);
                for v in 0..nr_map_vertices {
                    let rotated_v = e.apply_transform(&cop_moves.map.edges, &rotate_back, v);
                    let robber_safe = f.robber_is_safe_at(f_indices.at(rotated_v));
                    f_neighbor_changed |= robber_safe && !safe_robber_neighbors[rotated_v];
                    f_temp[rotated_v] = robber_safe && safe_robber_neighbors[rotated_v];
                }
            }
            else {
                debug_assert!(neigh_rotate == Matrix3x3::IDENTITY, 
                    "we only rotate when there are equivalent vertices");
                let robber_safe = f.robber_safe_when(rotated_neigh_cop_positions);
                for (stored, b1, &b2) in izip!(&mut f_temp, robber_safe, &safe_robber_neighbors) {
                    f_neighbor_changed |= b1 && !b2;
                    *stored = b1 && b2;
                }
            }

            //line 9
            if f_neighbor_changed {
                //line 10
                let range = f.robber_indices_at(rotated_neigh_cop_positions);
                for (v, &val) in izip!(0.., &f_temp) {
                    let rotated = if let Some(e) = &cop_moves.map.eqivalence {
                        e.apply_transform(&cop_moves.map.edges, &neigh_rotate, v)
                    } else {
                        v
                    };
                    f.mark_robber_at(range.at(rotated), val);
                }
                //line 11
                if queue.try_reserve(1).is_err() {
                    return BruteForceResult::Error("Zu wenig Speicherplatz (Queue zu lang)".to_owned());
                }
                queue.push_back(rotated_neigh_cop_positions);
            }

            //lines 13 + 14 + 15
            if f.robber_safe_when(rotated_neigh_cop_positions).all(|x| !x) {
                return BruteForceResult::CopsWin(nr_cops, nr_map_vertices);
            }
        }

    }

    BruteForceResult::RobberWins(nr_cops, f, cop_moves)
}





