
use std::collections::VecDeque;

use itertools::{izip, Itertools};
use bitvec::prelude as bv;

use crate::app::map;
use super::*;

pub const MAX_COPS: usize = 8;

/// this corresponds to one entry in CopConfigurations:
/// [`self.fst_cop`] is expected to be a vertex symmetry class representative and the (rotated / mirrored) position of one of the cops.
/// [`self.rest_cops`] represents the compacted sorted tuple of the other cops positions (obv. rotated the same as fst_cop).
/// thus `(0..nr_symmetry_classes).contains(&self.fst_cop)` and `self.nr_rest_cops < nr_map_vertices * nr_cops`
#[derive(Clone, Copy, PartialEq, Eq)]
struct CompactCops {
    fst_cop: usize,
    rest_cops: usize,
}

/// same structure as [`CompactCops`], only now [`self.rest_index`] is an index into the [`CopConfigurations::configurations`] entry
/// at key [`self.fst_index`]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CompactCopsIndex {
    fst_index: usize,
    rest_index: usize,
 }

pub struct SymmetricMap<S: SymmetryGroup> {
    pub shape: map::Shape,
    pub edges: EdgeList,
    pub symmetry: S,
}

/// keeps track of all multisets of size `self.nr_cops` and elements in  `self.nr_original_vertices` 
pub struct CopConfigurations<S: SymmetryGroup = ExplicitClasses> {
    map: SymmetricMap<S>,
    nr_cops: usize, //stored in usize, but this can in praxis only be veeeery small
    configurations: std::collections::BTreeMap<usize, Vec<usize>>,
}

/// fits whole tuple `other_cops` in integer, assuming all entries are always smaller than `nr_vertices`
fn pack_rest(nr_vertices: usize, other_cops: &[usize]) -> usize {
    let mut positions: usize = 0;
    for &cop in other_cops.iter() {
        positions *= nr_vertices;
        debug_assert!(cop < nr_vertices);
        positions += cop;
    }    
    positions
}

fn is_stored_config(sym: &impl SymmetryGroup, fst_cop: usize, rest_cops: &[usize]) -> bool {
    let mut config_copy = [usize::MAX; MAX_COPS];
    let config_copy = &mut config_copy[..(1 + rest_cops.len())];
    config_copy[0] = fst_cop;
    for i in 0..rest_cops.len() {
        config_copy[i + 1] = rest_cops[i];
    }
    sym.to_representative(config_copy);
    config_copy[0] == fst_cop && config_copy[1..] == rest_cops[..]
}

impl<S: SymmetryGroup> CopConfigurations<S> {

    pub fn nr_og_vertices(&self) -> usize {
        self.map.edges.nr_vertices()
    } 

    fn nr_configurations(&self) -> usize {
        self.configurations.iter().map(|(_, c)| c.len()).sum()
    }

    /// this function exists mostly as an explainer of [`CompactCopsIndex`] and [`CompactCops`]
    #[allow(dead_code)]
    fn get(&self, index: CompactCopsIndex) -> CompactCops {
        let configs_part = self.configurations.get(&index.fst_index).unwrap();
        CompactCops {
            fst_cop: index.fst_index,
            rest_cops: configs_part[index.rest_index]
        }
    }

    /// returns [`Self`] if enough memory is available
    pub fn new(map: SymmetricMap<S>, nr_cops: usize) -> Option<Self> {
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
            let mut i = 0;
            if nr_cops > 1 {
                loop {
                    i += 1;
                    debug_assert!(curr_rest_config.iter().tuple_windows().all(|(a, b)| a <= b));
                    if is_stored_config(&map.symmetry, fst_cop, &curr_rest_config) {
                        let packed = pack_rest(map.edges.nr_vertices(), &curr_rest_config);
                        if let Err(_) = new_configuration.try_reserve(1) {
                            return false;
                        }
                        new_configuration.push(packed);
                    }
                    if !advance(&mut curr_rest_config) {
                        break;
                    }
                }
            } else {
                new_configuration.push(0);
            }
            println!("{} -> {} / {} = {}", fst_cop, new_configuration.len(), i, 
                new_configuration.len() as f32 / i as f32);
            let old = configurations.insert(fst_cop, new_configuration);
            debug_assert!(old.is_none());
            true
        };
        for fst_cop in map.symmetry.class_representatives() {
            if !add_configs_for_first(fst_cop) {
                return None;
            }
        }

        Some(Self { map, nr_cops, configurations })
    }

    pub fn rest_positions_at(&self, index: CompactCopsIndex) -> usize {
        let configs_part = self.configurations.get(&index.fst_index).unwrap();
        configs_part[index.rest_index]
    }

    /// returns the configuration stored at index
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

    fn eager_unpack(&self, index: CompactCopsIndex) -> [usize; MAX_COPS] {
        let mut unpacked = [usize::MAX; MAX_COPS];
        for (i, c) in self.unpack(index).enumerate() {
            unpacked[i] = c;
        }
        unpacked
    }

    /// returns the index where the (rotated / mirrored) configuration represented by cops is stored as `_.1`
    /// and returns the transformation that rotated and / or mirrored the input in order to find it in [`self.configurations`]
    pub fn pack<'a>(&'a self, cops: impl Iterator<Item = usize>) -> (S::AutoIter<'a>, CompactCopsIndex) {
        assert!(self.nr_cops <= MAX_COPS);
        let mut unpacked = [0usize; MAX_COPS];
        for (i, pos) in cops.enumerate() {
            debug_assert!(pos < self.nr_og_vertices());
            debug_assert_ne!(i, self.nr_cops);
            unpacked[i] = pos;
        }
        let unpacked = &mut unpacked[..self.nr_cops];
        let rotation = self.map.symmetry.to_representative(unpacked);
        debug_assert!(unpacked.iter().tuple_windows().all(|(a, b)| a <= b));

        let fst_cop = unpacked[0];
        let rest_cops = &unpacked[1..];
        debug_assert!(rest_cops.len() == 0 || fst_cop <= rest_cops[0]);

        let packed = pack_rest(self.map.edges.nr_vertices(), rest_cops);
        let configs_part = self.configurations.get(&fst_cop).unwrap();
        let rest_pos = configs_part.binary_search(&packed);
        (rotation, CompactCopsIndex {
            fst_index: fst_cop,
            rest_index: rest_pos.unwrap(),
        })
    }

    /// returns all cop positions reachable in a single lazy-cop move from positions, except the do-nothing move.
    /// because these positions may be stored in a different rotation and / or flipped, that rotation + flip to get from
    /// the move as rotated in the input to the output is also returned (see [`Self::pack`])
    pub fn lazy_cop_moves_from<'a>(&'a self, positions: CompactCopsIndex) 
    -> impl Iterator<Item = (S::AutoIter<'a>, CompactCopsIndex)> + 'a + Clone 
    {
        assert!(self.nr_cops <= MAX_COPS);
        let unpacked = self.eager_unpack(positions);

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

    pub fn replace_symmetry<S2: SymmetryGroup>(self, new_sym: S2) -> CopConfigurations<S2> {
        let map = SymmetricMap {
            edges: self.map.edges,
            shape: self.map.shape,
            symmetry: new_sym,
        };
        CopConfigurations { map, nr_cops: self.nr_cops, configurations: self.configurations }
    }

}

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// wether that vertex is safe for the robber to stand on or not.
pub struct SafeRobberPositions {
    safe: std::collections::BTreeMap<usize, bv::BitVec<u32>>,
    nr_map_vertices: usize,
}

/// the indices of all game map vertices for one cop configuration in [`SafeRobberPositions`]
struct RobberPosRange {
    fst_index: usize,
    range: std::ops::Range<usize>,
}

/// the index of one vertex in one cop configuration in [`SafeRobberPositions`] 
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
    /// returns [`Self`] if enough memory is available
    fn new<S: SymmetryGroup>(cop_moves: &CopConfigurations<S>) -> Option<Self> {
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
}

pub enum BruteForceResult<S: SymmetryGroup> {
    None,
    Error(String),
    /// stores for how many cops result was computed and for a graph over how many vertices of what shape
    CopsWin(usize, usize, map::Shape),
    /// stores for how many cops result was computed, + result
    RobberWins(usize, map::Shape, SafeRobberPositions, CopConfigurations<S>),
}

impl BruteForceResult<NoSymmetry> {
    pub fn to_explicit(self) -> BruteForceResult<ExplicitClasses> {
        match self {            
            BruteForceResult::None => BruteForceResult::None,
            BruteForceResult::CopsWin(x, y, z) => BruteForceResult::CopsWin(x, y, z),
            BruteForceResult::Error(e) => BruteForceResult::Error(e),
            BruteForceResult::RobberWins(nr_cops, shape, safe, configs) => {
                let new_id = ExplicitClasses::new_no_symmetry(configs.nr_og_vertices());
                let new_configs = configs.replace_symmetry(new_id);
                BruteForceResult::RobberWins(nr_cops, shape, safe, new_configs)
            }
        }
    }
}

/// algorithm 2.2 of Fabian Hamann's masters thesis.
/// 
/// idea: store for each cop configuration and each vertex of the game map, if this vertex is safe for the robber to be on.
/// (this storage is [`SafeRobberPositions::safe`]).
/// 
/// start with every vertex marked as safe, that is not a cop's position and not a neighbor of a cop's position.
/// put every cop state in the queue to update.
/// 
/// for any cop state `c`:
/// if the cops can arrive at `c` in a single move (e.g. all positions except one are identical and that one neighbors the old position),
/// the robber was only safe on a vertex with respect to a previous cop state `c'`, if he can move to a vertex marked as safe for `c`.
/// 
/// thus the main loop of the algorithm takes a cop state `c` out of the queue, 
/// marks the neighborhood of the vertices marked safe for state `c` as *"safe should the cops move to `c`"* and goes though all
/// potential previous cop states `c'`. a vertex will now lose it's safety status w.r.t. `c'`, if it is not also contained 
/// in the freshly marked neighborhood of the vertices marked safe for state `c`.
/// should at least one vertex lose it's safe status for cop configuration `c'`, we need to test cop states `c''` which could turn into `c'`
/// via a single move. hence `c'` is entered into the queue.
/// 
/// the hole thing is somewhat more complicated by the fact, that neighboring cop states `c` and `c'` may not both be stored in the same
/// rotation. therefore one needs to constantly rotate between the two.
pub fn compute_safe_robber_positions<'a, S: SymmetryGroup>(nr_cops: usize, map: SymmetricMap<S>) -> BruteForceResult<S> {
    if nr_cops == 0 {
        return BruteForceResult::Error("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return BruteForceResult::Error(format!("Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."));
    }
    let Some(cop_moves) = CopConfigurations::new(map, nr_cops) else {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Cops passen nicht in Int)".to_owned());
    };
    let Some(mut f) = SafeRobberPositions::new(&cop_moves) else {
        return BruteForceResult::Error("Zu wenig Speicherplatz (Räuber-Gewinnfunktion zu groß)".to_owned());
    };

    let mut queue = VecDeque::new();
    if queue.try_reserve(cop_moves.nr_configurations()).is_err() {
        return BruteForceResult::Error("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
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
    //if the current game state has cop configuration `curr`, this contains all robber positions that 
    //where safe last game state, given that the cops move to `curr`.
    let mut safe_should_cops_move_to_curr = vec![false; nr_map_vertices];
    //intersection of `safe_should_cops_move_to_curr` and the vertices previously marked as safe for gamestate bevor curr
    let mut f_temp = vec![false; nr_map_vertices];

    //lines 4 + 5
    while let Some(curr_cop_positions) = queue.pop_back() {
        //line 6
        for (safe_before, safe_now) in izip!(&mut safe_should_cops_move_to_curr, f.robber_safe_when(curr_cop_positions)) {
            *safe_before = safe_now;
        }
        for (v, safe_now) in f.robber_safe_when(curr_cop_positions).enumerate() {
            if safe_now {
                for n in cop_moves.map.edges.neighbors_of(v) {
                    safe_should_cops_move_to_curr[n] = true;
                }
            }
        }

        //line 7
        for (neigh_rotations, rotated_neigh_cop_positions) in cop_moves.lazy_cop_moves_from(curr_cop_positions) {
            //if it only takes a single move to go from curr_cop_positions to rotated_neigh_cop_positions, so
            //should the other direction.
            debug_assert!(cop_moves
                .lazy_cop_moves_from(rotated_neigh_cop_positions)
                .any(|(_, pos)| pos == curr_cop_positions)
            );
            debug_assert!(!neigh_rotations.clone().into_iter().count() > 0);

            for neigh_rotate in neigh_rotations {               
                //line 8
                let mut f_neighbor_changed = false;
                //guarantee that rotations do the right thing
                debug_assert!({
                    let mut unpacked_curr = [0usize; MAX_COPS];
                    for (storage, pos) in izip!(&mut unpacked_curr, cop_moves.unpack(curr_cop_positions)) {
                        *storage = pos;
                    }
                    //all positions of current cop configuration.
                    let unpacked_curr = &mut unpacked_curr[..cop_moves.nr_cops];
                    let mut moved_cop_pos = usize::MAX;
                    for rotated_neigh_pos in cop_moves.unpack(rotated_neigh_cop_positions) {
                        let unrotated = neigh_rotate.apply_backward(rotated_neigh_pos);
                        let rerotated = neigh_rotate.apply_forward(unrotated);
                        debug_assert_eq!(rerotated, rotated_neigh_pos);
                        //if the neighbor configuration's position is found in curr configuration,
                        //remove entry from curr configuration.
                        if let Some(i) = unpacked_curr.iter().position(|&c| c == unrotated) {
                            unpacked_curr[i] = usize::MAX;
                        }
                        else {
                            //position is not found -> that must have been the cop that moved.
                            //remember for later.
                            debug_assert!(moved_cop_pos == usize::MAX);
                            moved_cop_pos = unrotated;
                        }
                    }
                    //now only the curr position of the cop which just moved there should be left in unpacked_curr.
                    debug_assert!(moved_cop_pos != usize::MAX);
                    unpacked_curr.iter().all(
                        |&c| c == usize::MAX || cop_moves.map.edges.neighbors_of(c).contains(&moved_cop_pos)
                    )
                });

                for (v, marked_safe_for_neigh) in izip!(neigh_rotate.backward(), f.robber_safe_when(rotated_neigh_cop_positions)) {
                    f_neighbor_changed |= marked_safe_for_neigh && !safe_should_cops_move_to_curr[v];
                    f_temp[v] = marked_safe_for_neigh && safe_should_cops_move_to_curr[v];
                }

                //line 9
                if f_neighbor_changed {
                    //line 10
                    let range = f.robber_indices_at(rotated_neigh_cop_positions);
                    for (v, &val) in izip!(neigh_rotate.forward(), &f_temp) {
                        f.mark_robber_at(range.at(v), val);
                    }
                    //line 11
                    if queue.try_reserve(1).is_err() {
                        return BruteForceResult::Error("Zu wenig Speicherplatz (Queue zu lang)".to_owned());
                    }
                    queue.push_back(rotated_neigh_cop_positions);
                }
            }

            //lines 13 + 14 + 15
            if f.robber_safe_when(rotated_neigh_cop_positions).all(|x| !x) {
                //it is redundant to return `f` (or `cop_moves`), because no vertex would be marked anyway.
                return BruteForceResult::CopsWin(nr_cops, nr_map_vertices, cop_moves.map.shape);
            }
        }

    }

    BruteForceResult::RobberWins(nr_cops, cop_moves.map.shape, f, cop_moves)
}



