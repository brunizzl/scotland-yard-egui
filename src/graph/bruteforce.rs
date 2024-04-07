use std::collections::VecDeque;

use bitvec::prelude as bv;
use itertools::{izip, Itertools};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

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

/// same structure as [`CompactCops`], except [`self.rest_index`] is an index into the [`CopConfigurations::configurations`] entry
/// at key [`self.fst_index`]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CompactCopsIndex {
    fst_index: usize,
    rest_index: usize,
}

/// keeps track of all multisets of size `self.nr_cops` and elements in  `self.nr_original_vertices`
#[derive(Serialize, Deserialize)]
pub struct CopConfigurations {
    nr_cops: usize, //stored in usize, but this can in praxis only be veeeery small
    nr_map_vertices: usize,
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
    config_copy[1..].copy_from_slice(rest_cops);

    sym.to_representative(config_copy);
    config_copy[0] == fst_cop && config_copy[1..] == rest_cops[..]
}

impl CopConfigurations {
    fn nr_configurations(&self) -> usize {
        self.configurations.values().map(|c| c.len()).sum()
    }

    pub fn nr_map_vertices(&self) -> usize {
        self.nr_map_vertices
    }

    pub fn nr_cops(&self) -> usize {
        self.nr_cops
    }

    /// this function exists mostly as an explainer of [`CompactCopsIndex`] and [`CompactCops`]
    #[allow(dead_code)]
    fn get(&self, index: CompactCopsIndex) -> CompactCops {
        let configs_part = self.configurations.get(&index.fst_index).unwrap();
        CompactCops {
            fst_cop: index.fst_index,
            rest_cops: configs_part[index.rest_index],
        }
    }

    /// returns [`Self`] if enough memory is available
    pub fn new<S: SymmetryGroup>(
        edges: &EdgeList,
        sym: &S,
        nr_cops: usize,
    ) -> Result<Self, String> {
        let nr_map_vertices = edges.nr_vertices();
        let curr_rest_config_size = 0.max(nr_cops as isize - 1) as usize;
        if (nr_map_vertices - 1).checked_pow(curr_rest_config_size as u32).is_none() {
            return Err("Polizeipositionen passen nicht in usize".to_string());
        }
        let mut curr_rest_config = vec![0usize; curr_rest_config_size];
        let advance = |rest_config: &mut [_]| -> bool {
            let mut fst_change = rest_config.len() - 1;
            loop {
                if rest_config[fst_change] + 1 < nr_map_vertices {
                    rest_config[fst_change] += 1;
                    break;
                }
                if fst_change == 0 {
                    return false;
                }
                fst_change -= 1;
            }
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
                    if is_stored_config(sym, fst_cop, &curr_rest_config) {
                        let packed = pack_rest(nr_map_vertices, &curr_rest_config);
                        if new_configuration.try_reserve(1).is_err() {
                            return Err("Zu wenig Speicherplatz für Polizeipositionen".to_string());
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
            println!(
                "{} -> {} / {} = {}",
                fst_cop,
                new_configuration.len(),
                i,
                new_configuration.len() as f32 / i as f32
            );
            let old = configurations.insert(fst_cop, new_configuration);
            debug_assert!(old.is_none());
            Ok(())
        };
        for fst_cop in sym.class_representatives() {
            add_configs_for_first(fst_cop)?;
        }

        Ok(Self {
            nr_cops,
            nr_map_vertices,
            configurations,
        })
    }

    pub fn rest_positions_at(&self, index: CompactCopsIndex) -> usize {
        let configs_part = self.configurations.get(&index.fst_index).unwrap();
        configs_part[index.rest_index]
    }

    /// returns the configuration stored at index
    pub fn unpack(&self, index: CompactCopsIndex) -> impl Iterator<Item = usize> + Clone {
        let mut positions = self.rest_positions_at(index);
        let nr_vertices = self.nr_map_vertices();

        std::iter::once(index.fst_index).chain((1..self.nr_cops).map(move |_| {
            let val = positions % nr_vertices;
            positions /= nr_vertices;
            val
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
    fn pack_impl<'a, A, AIter>(
        &self,
        mut to_representative: impl FnMut(&mut [usize]) -> AIter + 'a,
        cops: impl Iterator<Item = usize>,
    ) -> (AIter, CompactCopsIndex)
    where
        A: ?Sized + 'a,
        AIter: IntoIterator<Item = &'a A>,
    {
        assert!(self.nr_cops <= MAX_COPS);
        let mut unpacked = [0usize; MAX_COPS];
        for (i, pos) in cops.enumerate() {
            debug_assert!(pos < self.nr_map_vertices());
            debug_assert_ne!(i, self.nr_cops);
            unpacked[i] = pos;
        }
        let unpacked = &mut unpacked[..self.nr_cops];
        let rotation = to_representative(unpacked);
        debug_assert!(unpacked.iter().tuple_windows().all(|(a, b)| a <= b));

        let fst_cop = unpacked[0];
        let rest_cops = &unpacked[1..];
        debug_assert!(rest_cops.is_empty() || fst_cop <= rest_cops[0]);

        let packed = pack_rest(self.nr_map_vertices(), rest_cops);
        let configs_part = self.configurations.get(&fst_cop).unwrap();
        let rest_pos = configs_part.binary_search(&packed);
        (
            rotation,
            CompactCopsIndex {
                fst_index: fst_cop,
                rest_index: rest_pos.unwrap(),
            },
        )
    }

    pub fn pack<'a, S: SymmetryGroup>(
        &self,
        symmetry: &'a S,
        cops: impl Iterator<Item = usize>,
    ) -> (S::AutoIter<'a>, CompactCopsIndex) {
        self.pack_impl(
            |vertices: &mut [usize]| symmetry.to_representative(vertices),
            cops,
        )
    }

    fn pack_dyn<'a>(
        &self,
        symmetry: &'a dyn DynSymmetryGroup,
        cops: impl Iterator<Item = usize>,
    ) -> (SmallVec<[&'a dyn DynAutomorphism; 4]>, CompactCopsIndex) {
        self.pack_impl(
            |vertices: &mut [usize]| symmetry.dyn_to_representative(vertices),
            cops,
        )
    }

    /// returns all cop positions reachable in a single lazy-cop move from positions, except the do-nothing move.
    /// because these positions may be stored in a different rotation and / or flipped, that rotation + flip to get from
    /// the move as rotated in the input to the output is also returned (see [`Self::pack`])
    pub fn lazy_cop_moves_from<'a, S: SymmetryGroup>(
        &'a self,
        edges: &'a EdgeList,
        sym: &'a S,
        positions: CompactCopsIndex,
    ) -> impl Iterator<Item = (S::AutoIter<'a>, CompactCopsIndex)> + 'a + Clone {
        assert!(self.nr_cops <= MAX_COPS);
        let unpacked = self.eager_unpack(positions);

        let iter = (0..self.nr_cops).flat_map(move |i| {
            let cop_i_pos = unpacked[i];
            edges.neighbors_of(cop_i_pos).map(move |n| {
                let swap_i = |j| if j == i { n } else { unpacked[j] };
                self.pack(sym, (0..self.nr_cops).map(swap_i))
            })
        });

        iter
    }

    pub fn dyn_lazy_cop_moves_from<'a>(
        &'a self,
        edges: &'a EdgeList,
        sym: &'a dyn DynSymmetryGroup,
        positions: CompactCopsIndex,
    ) -> impl Iterator<
        Item = (
            SmallVec<[&'a dyn DynAutomorphism; 4]>,
            CompactCopsIndex,
            [usize; MAX_COPS],
        ),
    > {
        let mut unpacked = self.eager_unpack(positions);
        unpacked[..self.nr_cops].sort();

        (0..self.nr_cops).flat_map(move |i| {
            let cop_i_pos = unpacked[i];
            edges.neighbors_of(cop_i_pos).map(move |n| {
                let swap_i = |j| if j == i { n } else { unpacked[j] };
                let (vec, index) = self.pack_dyn(sym, (0..self.nr_cops).map(swap_i));
                let mut unpacked_neigh = unpacked;
                unpacked_neigh[i] = n;
                (vec, index, unpacked_neigh)
            })
        })
    }

    pub fn all_positions_unpacked(&self) -> impl Iterator<Item = [usize; MAX_COPS]> + '_ {
        self.configurations.iter().flat_map(move |(&k, configs_part)| {
            (0..configs_part.len())
                .map(move |i| self.eager_unpack(CompactCopsIndex { fst_index: k, rest_index: i }))
        })
    }
}

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// wether that vertex is safe for the robber to stand on or not.
#[derive(Serialize, Deserialize)]
pub struct SafeRobberPositions {
    safe: std::collections::BTreeMap<usize, bv::BitVec<u32>>,
    nr_map_vertices: usize,
}

/// the indices of all game map vertices for one cop configuration in [`SafeRobberPositions`]
pub struct RobberPosRange {
    fst_index: usize,
    range: std::ops::Range<usize>,
}

/// the index of one vertex in one cop configuration in [`SafeRobberPositions`]
pub struct RobberPosIndex {
    fst_index: usize,
    rest_index: usize,
}

impl RobberPosRange {
    pub fn at(&self, i: usize) -> RobberPosIndex {
        RobberPosIndex {
            fst_index: self.fst_index,
            rest_index: self.range.start + i,
        }
    }
}

impl SafeRobberPositions {
    /// returns [`Self`] if enough memory is available
    fn new(nr_map_vertices: usize, cop_moves: &CopConfigurations) -> Option<Self> {
        let mut safe = std::collections::BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len().checked_mul(nr_map_vertices)?;

            let vec_data_len = (nr_entries + 31) / 32;
            let mut bit_vec_data = Vec::<u32>::new();
            bit_vec_data.try_reserve(vec_data_len).ok()?;
            bit_vec_data.resize(vec_data_len, u32::MAX);

            let alloced_safe_data = bv::BitVec::try_from_vec(bit_vec_data).ok()?;
            let old = safe.insert(fst_index, alloced_safe_data);
            debug_assert!(old.is_none());
        }

        Some(Self { safe, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self) -> usize {
        self.nr_map_vertices
    }

    pub fn robber_indices_at(&self, index: CompactCopsIndex) -> RobberPosRange {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        RobberPosRange {
            fst_index: index.fst_index,
            range: start..stop,
        }
    }

    pub fn robber_safe_when(
        &self,
        index: CompactCopsIndex,
    ) -> impl ExactSizeIterator<Item = bool> + '_ + Clone {
        let RobberPosRange { fst_index, range } = self.robber_indices_at(index);
        let safe_part = self.safe.get(&fst_index).unwrap();
        range.map(move |i| safe_part[i])
    }

    fn mark_robber_at(&mut self, index: RobberPosIndex, value: bool) {
        let safe_part = self.safe.get_mut(&index.fst_index).unwrap();
        safe_part.set(index.rest_index, value);
    }

    pub fn robber_safe_at(&self, index: RobberPosIndex) -> bool {
        let safe_part = self.safe.get(&index.fst_index).unwrap();
        safe_part[index.rest_index]
    }
}

#[derive(Serialize, Deserialize)]
pub struct RobberWinData {
    pub symmetry: SymGroup,
    pub safe: SafeRobberPositions,
    pub cop_moves: CopConfigurations,
}

impl RobberWinData {
    pub fn pack(
        &self,
        cops: impl Iterator<Item = usize>,
    ) -> (SmallVec<[&dyn DynAutomorphism; 4]>, CompactCopsIndex) {
        self.cop_moves.pack_dyn(self.symmetry.to_dyn(), cops)
    }
}

#[derive(Serialize, Deserialize)]
pub enum Outcome {
    /// stores for how many cops result was computed and for a graph over how many vertices of what shape
    CopsWin,
    RobberWins(RobberWinData),
}
struct RobberStratQueue {
    queue: VecDeque<CompactCopsIndex>,
    contained: std::collections::BTreeMap<usize, Vec<bool>>,
}

impl RobberStratQueue {
    pub fn new(cop_moves: &CopConfigurations) -> Option<Self> {
        let mut contained = std::collections::BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len();

            let mut data = Vec::new();
            data.try_reserve(nr_entries).ok()?;
            data.resize(nr_entries, false);
            let old = contained.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        let mut queue = VecDeque::new();
        queue.try_reserve(cop_moves.nr_configurations()).ok()?;
        Some(Self { queue, contained })
    }

    pub fn push(&mut self, entry: CompactCopsIndex) {
        let is_there = &mut self.contained.get_mut(&entry.fst_index).unwrap()[entry.rest_index];
        if !*is_there {
            self.queue.push_back(entry);
            *is_there = true;
        }
    }

    pub fn pop(&mut self) -> Option<CompactCopsIndex> {
        let res = self.queue.pop_front();
        if let Some(i) = res {
            self.contained.get_mut(&i.fst_index).unwrap()[i.rest_index] = false;
        }
        res
    }

    pub fn len(&self) -> usize {
        self.queue.len()
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
/// the whole thing is somewhat more complicated by the fact, that neighboring cop states `c` and `c'` may not both be stored in the same
/// rotation. therefore one needs to constantly rotate between the two.
use std::sync::mpsc;
pub fn compute_safe_robber_positions<S>(
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    log_sender: Option<mpsc::Sender<String>>,
) -> Result<Outcome, String>
where
    S: SymmetryGroup + Serialize,
{
    let log = |msg| log_sender.as_ref().map(|s| s.send(msg).ok());

    if nr_cops == 0 {
        return Err("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return Err(format!(
            "Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."
        ));
    }

    log("liste Polizeipositionen".into());
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops)?;

    log("initialisiere Räubergewinnfunktion".into());
    let Some(mut f) = SafeRobberPositions::new(edges.nr_vertices(), &cop_moves) else {
        return Err("Zu wenig Speicherplatz (Räubergewinnfunktion zu groß)".to_owned());
    };

    log("initialisiere Queue".into());
    let Some(mut queue) = RobberStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    for (&fst_index, sub_configs) in &cop_moves.configurations {
        for (rest_index, _packed_rest_cops) in izip!(0.., sub_configs) {
            let index = CompactCopsIndex { fst_index, rest_index };

            debug_assert!({
                let unpacked = cop_moves.unpack(index);
                let (_, packed) = cop_moves.pack(&sym, unpacked);
                debug_assert_eq!(index, packed);
                true
            });

            //line 2
            let robber_range = f.robber_indices_at(index);
            for cop_pos in cop_moves.unpack(index) {
                f.mark_robber_at(robber_range.at(cop_pos), false);
                for n in edges.neighbors_of(cop_pos) {
                    f.mark_robber_at(robber_range.at(n), false);
                }
            }

            //line 3
            queue.push(index);
        }
    }

    let nr_map_vertices = edges.nr_vertices();
    //if the current game state has cop configuration `curr`, this contains all robber positions that
    //where safe last game state, given that the cops move to `curr`.
    let mut safe_should_cops_move_to_curr = vec![false; nr_map_vertices];
    //intersection of `safe_should_cops_move_to_curr` and the vertices previously marked as safe for gamestate bevor curr
    let mut f_temp = vec![false; nr_map_vertices];

    //lines 4 + 5
    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            log(format!(
                "berechne Räuberstrategie:\n{:.2}% in Queue ({})",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len()
            ));
            time_until_log_refresh = 2000;
        }

        //line 6
        for (safe_before, safe_now) in izip!(
            &mut safe_should_cops_move_to_curr,
            f.robber_safe_when(curr_cop_positions)
        ) {
            *safe_before = safe_now;
        }
        for (v, safe_now) in f.robber_safe_when(curr_cop_positions).enumerate() {
            if safe_now {
                for n in edges.neighbors_of(v) {
                    safe_should_cops_move_to_curr[n] = true;
                }
            }
        }

        //line 7
        for (neigh_rotations, rotated_neigh_cop_positions) in
            cop_moves.lazy_cop_moves_from(&edges, &sym, curr_cop_positions)
        {
            //if it only takes a single move to go from curr_cop_positions to rotated_neigh_cop_positions, so
            //should the other direction.
            debug_assert!(cop_moves
                .lazy_cop_moves_from(&edges, &sym, rotated_neigh_cop_positions)
                .any(|(_, pos)| pos == curr_cop_positions));
            debug_assert!(!neigh_rotations.clone().into_iter().count() > 0);

            let mut f_neighbor_changed_some_rotation = false;
            for neigh_rotate in neigh_rotations {
                //line 8
                let mut f_neighbor_changed_this_rotation = false;
                //guarantee that rotations do the right thing
                debug_assert!({
                    let mut unpacked_curr = [0usize; MAX_COPS];
                    for (storage, pos) in
                        izip!(&mut unpacked_curr, cop_moves.unpack(curr_cop_positions))
                    {
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
                        } else {
                            //position is not found -> that must have been the cop that moved.
                            //remember for later.
                            debug_assert!(moved_cop_pos == usize::MAX);
                            moved_cop_pos = unrotated;
                        }
                    }
                    //now only the curr position of the cop which just moved there should be left in unpacked_curr.
                    debug_assert!(moved_cop_pos != usize::MAX);
                    unpacked_curr
                        .iter()
                        .all(|&c| c == usize::MAX || edges.neighbors_of(c).contains(&moved_cop_pos))
                });

                for (v, marked_safe_for_neigh) in izip!(
                    neigh_rotate.backward(),
                    f.robber_safe_when(rotated_neigh_cop_positions)
                ) {
                    f_neighbor_changed_this_rotation |=
                        marked_safe_for_neigh && !safe_should_cops_move_to_curr[v];
                    f_temp[v] = marked_safe_for_neigh && safe_should_cops_move_to_curr[v];
                }

                //line 9
                if f_neighbor_changed_this_rotation {
                    f_neighbor_changed_some_rotation = true;
                    //line 10
                    let range = f.robber_indices_at(rotated_neigh_cop_positions);
                    for (v, &val) in izip!(neigh_rotate.forward(), &f_temp) {
                        f.mark_robber_at(range.at(v), val);
                    }
                }
            }
            //if we changed the robber win function for some rotation, it is irrelevant which rotation caused the change.
            //we thus only enqueue each neighbor up to once, not up to once per our rotation.
            if f_neighbor_changed_some_rotation {
                //line 11
                queue.push(rotated_neigh_cop_positions);
            }

            //lines 13 + 14 + 15
            if f.robber_safe_when(rotated_neigh_cop_positions).all(|x| !x) {
                //it is redundant to return `f` (or `cop_moves`), because no vertex would be marked anyway.
                return Ok(Outcome::CopsWin);
            }
        }
    }

    Ok(Outcome::RobberWins(RobberWinData {
        symmetry: sym.into_enum(),
        safe: f,
        cop_moves,
    }))
}

pub type UTime = u8;

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// how many more moves the police need at most to catch the robber.
#[derive(Serialize, Deserialize)]
pub struct TimeToWin {
    time: std::collections::BTreeMap<usize, Vec<UTime>>,
    nr_map_vertices: usize,
}

impl TimeToWin {
    /// returns [`Self`] if enough memory is available
    fn new(nr_map_vertices: usize, cop_moves: &CopConfigurations) -> Option<Self> {
        let mut time = std::collections::BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len().checked_mul(nr_map_vertices)?;

            let mut data = Vec::new();
            data.try_reserve(nr_entries).ok()?;
            data.resize(nr_entries, UTime::MAX);
            let old = time.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        Some(Self { time, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self) -> usize {
        self.nr_map_vertices
    }

    /// returns current number for each vertex in graph given cops placed like `index`
    pub fn nr_moves_left(&self, index: CompactCopsIndex) -> &[UTime] {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        &self.time.get(&index.fst_index).unwrap()[start..stop]
    }

    /// returns current number for each vertex in graph given cops placed like `index`
    pub fn nr_moves_left_mut(&mut self, index: CompactCopsIndex) -> &mut [UTime] {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        &mut self.time.get_mut(&index.fst_index).unwrap()[start..stop]
    }
}

#[derive(Serialize, Deserialize)]
pub struct CopStrategy {
    pub symmetry: SymGroup,
    pub time_to_win: TimeToWin,
    pub cop_moves: CopConfigurations,
    pub max_moves: usize,
    pub cops_win: bool,
}

impl CopStrategy {
    pub fn pack(
        &self,
        cops: impl Iterator<Item = usize>,
    ) -> (SmallVec<[&dyn DynAutomorphism; 4]>, CompactCopsIndex) {
        self.cop_moves.pack_dyn(self.symmetry.to_dyn(), cops)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InQueue {
    No,
    Yes,
    NoAndAtMax,
    YesAndAtMax,
}

struct CopStratQueue {
    queue: VecDeque<CompactCopsIndex>,
    status: std::collections::BTreeMap<usize, Vec<InQueue>>,
    curr_max_nr_moves: UTime,
}

impl CopStratQueue {
    pub fn new(cop_moves: &CopConfigurations) -> Option<Self> {
        let mut contained = std::collections::BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len();

            let mut data = Vec::new();
            data.try_reserve(nr_entries).ok()?;
            data.resize(nr_entries, InQueue::No);
            let old = contained.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        let mut queue = VecDeque::new();
        queue.try_reserve(cop_moves.nr_configurations()).ok()?;
        Some(Self {
            queue,
            status: contained,
            curr_max_nr_moves: 2,
        })
    }

    fn status_mut_of(&mut self, entry: CompactCopsIndex) -> &mut InQueue {
        &mut self.status.get_mut(&entry.fst_index).unwrap()[entry.rest_index]
    }

    pub fn push(&mut self, entry: CompactCopsIndex) {
        let entry_status = self.status_mut_of(entry);

        let enqueue = match *entry_status {
            InQueue::No => {
                *entry_status = InQueue::Yes;
                true
            },
            InQueue::NoAndAtMax => {
                *entry_status = InQueue::YesAndAtMax;
                true
            },
            _ => false,
        };
        if enqueue {
            self.queue.push_back(entry);
        }
    }

    pub fn mark_as_at_max(&mut self, entry: CompactCopsIndex) {
        let entry_status = self.status_mut_of(entry);
        match *entry_status {
            InQueue::No => {
                *entry_status = InQueue::NoAndAtMax;
            },
            InQueue::Yes => {
                *entry_status = InQueue::YesAndAtMax;
            },
            _ => {},
        }
    }

    fn pop_and_mark_popped(&mut self) -> Option<CompactCopsIndex> {
        let res = self.queue.pop_front();
        if let Some(entry) = res {
            let entry_status = self.status_mut_of(entry);
            match *entry_status {
                InQueue::Yes => *entry_status = InQueue::No,
                InQueue::YesAndAtMax => *entry_status = InQueue::NoAndAtMax,
                _ => panic!(),
            }
        }
        res
    }

    pub fn pop(&mut self) -> Option<CompactCopsIndex> {
        if let Some(entry) = self.pop_and_mark_popped() {
            return Some(entry);
        }
        self.curr_max_nr_moves += 1;
        for (&fst_index, part) in &mut self.status {
            for (rest_index, entry_status) in part.iter_mut().enumerate() {
                if *entry_status == InQueue::NoAndAtMax {
                    *entry_status = InQueue::Yes;
                    self.queue.push_back(CompactCopsIndex { fst_index, rest_index });
                }
                if *entry_status == InQueue::YesAndAtMax {
                    *entry_status = InQueue::Yes;
                }
            }
        }
        self.pop_and_mark_popped()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }

    pub fn curr_max(&self) -> UTime {
        self.curr_max_nr_moves
    }
}

#[allow(dead_code)]
pub fn compute_cop_strategy<S>(
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    log_sender: Option<mpsc::Sender<String>>,
) -> Result<CopStrategy, String>
where
    S: SymmetryGroup + Serialize,
{
    let log = |msg| log_sender.as_ref().map(|s| s.send(msg).ok());

    if nr_cops == 0 {
        return Err("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return Err(format!(
            "Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."
        ));
    }

    log("liste Polizeipositionen".into());
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops)?;

    log("initialisiere Cop Startegie".into());
    let Some(mut f) = TimeToWin::new(edges.nr_vertices(), &cop_moves) else {
        return Err("Zu wenig Speicherplatz (Copstrat zu groß)".to_owned());
    };

    log("initialisiere Queue".into());
    let Some(mut queue) = CopStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    for (&fst_index, sub_configs) in &cop_moves.configurations {
        for (rest_index, _packed_rest_cops) in izip!(0.., sub_configs) {
            let index = CompactCopsIndex { fst_index, rest_index };

            let robber_positions = f.nr_moves_left_mut(index);
            for cop_pos in cop_moves.unpack(index) {
                robber_positions[cop_pos] = 0;
                for n in edges.neighbors_of(cop_pos) {
                    robber_positions[n] = 0;
                }
            }

            queue.push(index);
        }
    }

    let nr_map_vertices = edges.nr_vertices();
    //if the current game state has cop configuration `curr`, this contains the time for cops to win
    //for each possible robber position, given that the cops move to `curr`.
    let mut times_should_cops_move_to_curr = vec![UTime::MAX; nr_map_vertices];

    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            log(format!(
                "berechne Copstrategie:\n{:.2}% in Queue ({}), max {}",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len(),
                queue.curr_max()
            ));
            time_until_log_refresh = 2000;
        }

        let curr_times = f.nr_moves_left(curr_cop_positions);
        let mut curr_is_at_max = false;
        for (v, neighs) in izip!(0.., edges.neighbors()) {
            let new_time = neighs.fold(curr_times[v], |acc, n| acc.max(curr_times[n]));
            const MAX_TIME: UTime = UTime::MAX - 1;
            if new_time == MAX_TIME {
                return Err(format!(
                    "Cops brauchen mehr Züge als in {} passen",
                    std::any::type_name::<UTime>()
                ));
            }
            let new_time = new_time.saturating_add(1);
            times_should_cops_move_to_curr[v] = if new_time == queue.curr_max() {
                curr_is_at_max = true;
                UTime::MAX
            } else {
                debug_assert!(new_time < queue.curr_max());
                new_time
            };
        }
        if curr_is_at_max {
            queue.mark_as_at_max(curr_cop_positions);
        }

        for (neigh_rotations, rotated_neigh_cop_positions) in
            cop_moves.lazy_cop_moves_from(&edges, &sym, curr_cop_positions)
        {
            let mut f_neighbor_changed = false;
            for neigh_rotate in neigh_rotations {
                for (v, neigh_time) in izip!(
                    neigh_rotate.backward(),
                    f.nr_moves_left_mut(rotated_neigh_cop_positions)
                ) {
                    let this_time = times_should_cops_move_to_curr[v];
                    if *neigh_time > this_time {
                        debug_assert!(this_time < queue.curr_max());
                        f_neighbor_changed = true;
                        *neigh_time = this_time;
                    }
                }
            }
            if f_neighbor_changed {
                queue.push(rotated_neigh_cop_positions);
            }
        }
    }

    log("berechne Fun Facts".into());
    let mut max_moves = 0;
    let mut cops_win = true;
    for vals in f.time.values() {
        for &val in vals {
            if val == UTime::MAX {
                cops_win = false;
            } else if val > max_moves {
                max_moves = val;
            }
        }
    }

    Ok(CopStrategy {
        symmetry: sym.into_enum(),
        time_to_win: f,
        cop_moves,
        max_moves: max_moves as usize,
        cops_win,
    })
}
