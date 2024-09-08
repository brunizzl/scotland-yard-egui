use std::collections::{BTreeMap, VecDeque};

use bitvec::prelude as bv;
use itertools::{izip, Itertools};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use super::*;

pub mod thread_manager;

mod queues;
use queues::{CopStratQueue, RobberStratQueue};
struct OnDrop<F: Fn()>(F);

impl<F: Fn()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        self.0()
    }
}

/// maximum number of cops for which a bruteforce computation can be started.
/// is not too detrimental, that this number is small, 
/// because the computation is ungodly expensive anyway. 
/// (exponential in complexity, with this in the exponent and nr of graph vertices as base).
/// 
/// Keeping the number of cops at this limit allows us to not use the heap as much.
pub const MAX_COPS: usize = 7;

/// type to carry all cop positions on the stack.
/// to make this thing fast, `MAX_COPS` is kept relatively small.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct RawCops {
    /// only first `self.nr_cops` entries are active, the rest has value `usize::MAX`
    pub cops: [usize; MAX_COPS],
    pub nr_cops: usize,
}

impl RawCops {
    pub fn new(cs: &[usize]) -> Self {
        let mut cops = [usize::MAX; MAX_COPS];
        let nr_cops = cs.len();
        cops[..nr_cops].copy_from_slice(cs);
        Self { cops, nr_cops }
    }

    pub fn uninit(nr_cops: usize) -> Self {
        let cops = [usize::MAX; MAX_COPS];
        Self { cops, nr_cops }
    }

    pub fn from_iter(it: impl Iterator<Item = usize>) -> Self {
        let mut cops = [usize::MAX; MAX_COPS];
        let mut nr_cops = 0;
        for cop in it {
            cops[nr_cops] = cop;
            nr_cops += 1;
        }
        Self { cops, nr_cops }
    }
}

impl std::ops::Deref for RawCops {
    type Target = [usize];

    fn deref(&self) -> &Self::Target {
        &self.cops[..self.nr_cops]
    }
}

impl std::ops::DerefMut for RawCops {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.cops[..self.nr_cops]
    }
}

impl std::fmt::Debug for RawCops {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", &self[..])
    }
}

/// returns the number of distinct mutlisets with [`cardinality`] many elements
/// chosen from a set with [`universe_size`] many elements.
fn multiset_count(universe_size: usize, cardinality: usize) -> Option<usize> {
    fn binomial_coefficient(n: usize, k: usize) -> Option<usize> {
        if k == 0 {
            return Some(0);
        }
        let mut res: usize = 1;
        for i in 1..=k {
            res = res.checked_mul(n + 1 - i)?;
            debug_assert_eq!(res % i, 0);
            res /= i;
        }
        Some(res)
    }
    debug_assert_eq!(binomial_coefficient(10, 3), Some(120));
    debug_assert_eq!(binomial_coefficient(20, 10), Some(184756));
    debug_assert_eq!(binomial_coefficient(8, 6), Some(28));

    binomial_coefficient(universe_size + cardinality - 1, cardinality)
}

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
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct CompactCopsIndex {
    fst_index: usize,
    rest_index: usize,
}

/// keeps track of all multisets of size `self.nr_cops` and elements in  `self.nr_original_vertices`
#[derive(Serialize, Deserialize)]
pub struct CopConfigurations {
    nr_cops: usize, //stored in usize, but this can in praxis only be veeeery small
    nr_map_vertices: usize,
    configurations: BTreeMap<usize, Vec<usize>>,
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
        manager: &thread_manager::LocalManager,
    ) -> Result<Self, String> {
        let nr_map_vertices = edges.nr_vertices();
        if nr_cops <= 1 {
            let sections = sym.class_representatives().map(|r| (r, vec![0]));
            return Ok(Self {
                nr_cops,
                nr_map_vertices,
                configurations: BTreeMap::from_iter(sections),
            });
        }

        if (nr_map_vertices - 1).checked_pow((nr_cops - 1) as u32).is_none() {
            return Err("Polizeipositionen passen nicht in usize".to_string());
        }

        // logging info + management
        let nr_configurations: usize = sym
            .class_representatives()
            .map(|r| multiset_count(nr_map_vertices - r, nr_cops - 1).unwrap())
            .sum();
        let mut i_configuration = 0;
        let mut time_until_log_refresh: usize = 1;
        let log_refresh_interval = (nr_configurations / 10_000).clamp(1000, 100_000);

        let mut configurations = BTreeMap::new();
        for fst_cop in sym.class_representatives() {
            let mut curr_config = RawCops::uninit(nr_cops);
            curr_config.fill(fst_cop);

            let mut configurations_section = Vec::new();
            'fill_section: loop {
                debug_assert!(curr_config.iter().tuple_windows().all(|(a, b)| a <= b));

                // update logs
                i_configuration += 1;
                time_until_log_refresh -= 1;
                if time_until_log_refresh == 0 {
                    manager.update(format!(
                        "liste Coppositionen:\n{:.2}%, aktuell {curr_config:?}",
                        (i_configuration as f64) / (nr_configurations as f64) * 100.0
                    ))?;
                    time_until_log_refresh = log_refresh_interval;
                }

                // test if `curr_config` is representant of equivalency class. if so, add to configs
                let mut config_copy = curr_config;
                let (_, repr) = sym.power_repr(&mut config_copy);
                if curr_config == repr {
                    let packed = pack_rest(nr_map_vertices, &curr_config[1..]);
                    if configurations_section.try_reserve(1).is_err() {
                        return Err("Zu wenig Speicherplatz für Polizeipositionen".to_string());
                    }
                    configurations_section.push(packed);
                }

                // advance `curr_config` to next config.
                // skip configs which aren't sorted.
                for fst_change in (1..curr_config.len()).rev() {
                    let new_val = curr_config[fst_change] + 1;
                    if new_val < nr_map_vertices {
                        for old in &mut curr_config[fst_change..] {
                            *old = new_val;
                        }
                        continue 'fill_section;
                    }
                }
                // all entries (except first) in `curr_config` have value `nr_map_vertices - 1`
                // -> all cops but fst_cop stand on the very last vertex
                // -> there is no config stored in this section after this one.
                debug_assert_eq!(curr_config[0], fst_cop);
                debug_assert!(curr_config[1..].iter().all(|&c| c + 1 == nr_map_vertices));
                break 'fill_section;
            }
            let old = configurations.insert(fst_cop, configurations_section);
            debug_assert!(old.is_none());
        }
        debug_assert_eq!(nr_configurations, i_configuration);

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

    pub fn eager_unpack(&self, index: CompactCopsIndex) -> RawCops {
        RawCops::from_iter(self.unpack(index))
    }

    /// returns the index where the (rotated / mirrored) configuration represented by cops is stored as `_.1`
    /// and returns the transformation that rotated and / or mirrored the input in order to find it in [`self.configurations`]
    pub fn pack<'a, S: SymmetryGroup>(
        &self,
        symmetry: &'a S,
        cops: &mut RawCops,
    ) -> (SmallVec<[&'a S::Auto; 4]>, CompactCopsIndex) {
        debug_assert_eq!(self.nr_cops, cops.nr_cops);
        let (autos, repr) = symmetry.power_repr(cops);
        debug_assert!(repr.iter().tuple_windows().all(|(a, b)| a <= b));

        let fst_cop = repr[0];
        let rest_cops = &repr[1..];
        debug_assert!(rest_cops.is_empty() || fst_cop <= rest_cops[0]);

        let packed = pack_rest(self.nr_map_vertices(), rest_cops);
        let configs_part = self.configurations.get(&fst_cop).unwrap();
        let rest_pos = configs_part.binary_search(&packed);
        (
            autos,
            CompactCopsIndex {
                fst_index: fst_cop,
                rest_index: rest_pos.unwrap(),
            },
        )
    }

    /// returns all cop configurations reachable in a single lazy-cop move from [`cops`],
    /// except the do-nothing move.
    /// the resulting iterator will not yield configurations as stored, but as they are actually reachable.
    pub fn raw_lazy_cop_moves_from<'a>(
        &'a self,
        edges: &'a EdgeList,
        cops: RawCops,
    ) -> impl Iterator<Item = RawCops> + 'a + Clone {
        (0..self.nr_cops).flat_map(move |i| {
            let cop_i_pos = cops[i];
            edges.neighbors_of(cop_i_pos).map(move |n| {
                let mut unpacked_neigh = cops;
                unpacked_neigh[i] = n;
                unpacked_neigh
            })
        })
    }

    /// returns all cop positions reachable in a single lazy-cop move from positions, except the do-nothing move.
    /// because these positions may be stored in a different rotation and / or flipped, that rotation + flip to get from
    /// the move as rotated in the input to the output is also returned (see [`Self::pack`])
    pub fn lazy_cop_moves_from<'a, S: SymmetryGroup>(
        &'a self,
        edges: &'a EdgeList,
        sym: &'a S,
        positions: CompactCopsIndex,
    ) -> impl Iterator<Item = (SmallVec<[&'a S::Auto; 4]>, CompactCopsIndex)> + 'a + Clone {
        self.raw_lazy_cop_moves_from(edges, self.eager_unpack(positions))
            .map(|mut cops| self.pack(sym, &mut cops))
    }

    pub fn all_positions(&self) -> impl Iterator<Item = CompactCopsIndex> + '_ {
        self.configurations.iter().flat_map(move |(&k, configs_part)| {
            (0..configs_part.len()).map(move |i| CompactCopsIndex { fst_index: k, rest_index: i })
        })
    }
}

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// wether that vertex is safe for the robber to stand on or not.
#[derive(Serialize, Deserialize)]
pub struct SafeRobberPositions {
    safe: BTreeMap<usize, bv::BitVec<u32>>,
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
        let mut safe = BTreeMap::new();
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

    pub fn robber_safe_when(&self, index: CompactCopsIndex) -> &bv::BitSlice<u32> {
        let RobberPosRange { fst_index, range } = self.robber_indices_at(index);
        let safe_part = self.safe.get(&fst_index).unwrap();
        &safe_part[range]
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
    #[serde(deserialize_with = "deserialize_explicit")]
    pub symmetry: ExplicitClasses,
    pub safe: SafeRobberPositions,
    pub cop_moves: CopConfigurations,
}

/// the field [`RobberWinData::symmetry`] changed type to [`ExplicitClasses`] from [`SymGroup`].
/// we thus want both versions to be correctly deserialized.
pub fn deserialize_explicit<'de, D>(deserializer: D) -> Result<ExplicitClasses, D::Error>
where
    D: serde::Deserializer<'de>,
{
    //SymGroup::deserialize(deserializer).map(|s| ExplicitClasses::from(s))
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum ExplicitAsMaybeEnum {
        Enum(SymGroup),
        Direct(ExplicitClasses),
    }

    Ok(match ExplicitAsMaybeEnum::deserialize(deserializer)? {
        ExplicitAsMaybeEnum::Enum(e) => e.into(),
        ExplicitAsMaybeEnum::Direct(expl) => expl,
    })
}

impl RobberWinData {
    pub fn safe_vertices(&self, cops: &mut RawCops) -> impl ExactSizeIterator<Item = bool> + '_ {
        let (autos, cop_positions) = self.cop_moves.pack(&self.symmetry, cops);
        let safe_vertices = self.safe.robber_safe_when(cop_positions);
        autos[0].forward().map(|v| safe_vertices[v])
    }
}

#[derive(Serialize, Deserialize)]
pub enum Outcome {
    /// stores for how many cops result was computed and for a graph over how many vertices of what shape
    CopsWin,
    RobberWins(RobberWinData),
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
pub fn compute_safe_robber_positions<S>(
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<Outcome, String>
where
    S: SymmetryGroup + Serialize,
{
    if nr_cops == 0 {
        return Err("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return Err(format!(
            "Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."
        ));
    }

    manager.update("liste Polizeipositionen")?;
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserviere Speicher für Räuberstrategiefunktion")?;
    let Some(mut f) = SafeRobberPositions::new(edges.nr_vertices(), &cop_moves) else {
        return Err("Zu wenig Speicherplatz (Räuberstrategiefunktion zu groß)".to_owned());
    };

    manager.update("reserviere Speicher für Queue")?;
    let Some(mut queue) = RobberStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    let max_degree_less_than_nr_cops = edges.max_degree() < nr_cops;

    manager.update("initialisiere Räuberstrategiefunktion")?;
    for (i, index) in izip!(0.., cop_moves.all_positions()) {
        if i % 4096 == 0 {
            manager.recieve()?;
        }

        // test round trip
        debug_assert!({
            let mut unpacked = cop_moves.eager_unpack(index);
            let (_, packed) = cop_moves.pack(&sym, &mut unpacked);
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

        // special case treatment: if nr cops is larger than nr neighbors of vertex,
        // vertex is not marked cop win and all neighbors are marked, then the robber has lost.
        // This is, because at least one cop is uninvolved in the current stalemate
        // and can come and capture the robber.
        // note: this is REALLY conservative and assumes no cop can guard two of
        // the robber's neighbors at once. only in this case however will this procedure
        // be of advantage.
        if max_degree_less_than_nr_cops {
            for (v, mut neighs) in izip!(0.., edges.neighbors()) {
                let safe_at = |v| f.robber_safe_at(robber_range.at(v));
                if safe_at(v) && neighs.all(|n| !safe_at(n)) {
                    f.mark_robber_at(robber_range.at(v), false);
                }
            }
        }
    }

    let nr_map_vertices = edges.nr_vertices();
    //if the current game state has cop configuration `curr`, this contains all robber positions that
    //where safe last game state, given that the cops move to `curr`.
    let mut safe_should_cops_move_to_curr = vec![false; nr_map_vertices];
    //intersection of `safe_should_cops_move_to_curr` and the vertices previously marked as safe for gamestate bevor curr
    let mut f_temp = vec![false; nr_map_vertices];

    let start = std::time::Instant::now();
    let _on_return = OnDrop(|| {
        println!(
            "beende rechnung nach {:?} fuer {nr_cops} cops auf {nr_map_vertices} knoten",
            std::time::Instant::now() - start
        );
    });

    //lines 4 + 5
    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            manager.update(format!(
                "berechne Räuberstrategie:\n{:.2}% in Queue ({}), Runde {}",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len(),
                queue.rounds_complete(),
            ))?;
            time_until_log_refresh = 1000;
        }

        //line 6
        for (safe_before, safe_now) in izip!(
            &mut safe_should_cops_move_to_curr,
            f.robber_safe_when(curr_cop_positions)
        ) {
            *safe_before = *safe_now;
        }
        for (v, safe_now) in izip!(0.., f.robber_safe_when(curr_cop_positions)) {
            if *safe_now {
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
            debug_assert!(!neigh_rotations.is_empty());

            let mut f_neighbor_changed_some_rotation = false;
            for neigh_rotate in neigh_rotations {
                //line 8
                let mut f_neighbor_changed_this_rotation = false;
                //guarantee that rotations do the right thing
                debug_assert!({
                    let mut unpacked_curr = cop_moves.eager_unpack(curr_cop_positions);
                    //all positions of current cop configuration.
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
                        *marked_safe_for_neigh && !safe_should_cops_move_to_curr[v];
                    f_temp[v] = *marked_safe_for_neigh && safe_should_cops_move_to_curr[v];
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
            if f.robber_safe_when(rotated_neigh_cop_positions).not_any() {
                //it is redundant to return `f` (or `cop_moves`), because no vertex would be marked anyway.
                return Ok(Outcome::CopsWin);
            }
        }
    }

    let result = RobberWinData {
        symmetry: sym.into_enum().into(),
        safe: f,
        cop_moves,
    };
    debug_assert!(verify_continuity(&result, &edges, manager).is_ok());

    Ok(Outcome::RobberWins(result))
}

/// [`data.safe`] is called continuous, if it is Lipschitz-continuous with constant 1,
/// where the distance measurement is the Haussdorff distance induced by the graph distance function.
///
/// Said differently: for every move the police can take, if the robber's vertex is currently marked safe,
/// the robber must be neighboring a safe vertex next move.
/// Further: there must exist a safe vertex for every police arrangement.
pub fn verify_continuity(
    data: &RobberWinData,
    edges: &EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<(), String> {
    // logging things
    let nr_configs = data.cop_moves.nr_configurations();
    let mut i_config = 0;
    let mut time_until_log_refresh = 1;
    let log_refresh_interval = (nr_configs / 10_000).clamp(1000, 100_000);

    let nr_map_vertices = data.cop_moves.nr_map_vertices();
    let mut safe_should_cops_move_to_curr = vec![false; nr_map_vertices];
    for cops_index in data.cop_moves.all_positions() {
        let cops = data.cop_moves.eager_unpack(cops_index);
        // logging things
        time_until_log_refresh -= 1;
        i_config += 1;
        if time_until_log_refresh == 0 {
            let done_percent = (i_config as f64) / (nr_configs as f64) * 100.0;
            manager.update(format!("verifiziere Kontinuität: {done_percent:.2}%"))?;
            time_until_log_refresh = log_refresh_interval;
        }

        let mut some_safe = false;
        safe_should_cops_move_to_curr.fill(false);
        for (v, neighs, safe) in izip!(
            0..,
            edges.neighbors(),
            data.safe.robber_safe_when(cops_index)
        ) {
            if *safe {
                some_safe = true;
                safe_should_cops_move_to_curr[v] = true;
                for n in neighs {
                    safe_should_cops_move_to_curr[n] = true;
                }
            }
        }

        // verify some safe vertex exists for every cop arrangement
        if !some_safe {
            return Err(format!(
                "Räuber ist nicht sicher, wenn Cops {cops:?} besetzen.",
            ));
        }

        // verify for every possible previous cop arrangement,
        // a vertex marked safe there still has a safe neighbor here (safe neighbors precomputed).
        for mut cops_neighs in data.cop_moves.raw_lazy_cop_moves_from(edges, cops) {
            for (v, &safe_before_curr, safe_last_move) in izip!(
                0..,
                &safe_should_cops_move_to_curr,
                data.safe_vertices(&mut cops_neighs)
            ) {
                if safe_last_move && !safe_before_curr {
                    return Err(format!(
                        "Räuberstrategie nicht kontinuierlich an Knoten {v},\
                        wenn Cops von {cops_neighs:?} zu {cops:?} ziehen."
                    ));
                }
            }
        }
    }

    Ok(())
}

pub type UTime = u8;

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// how many more moves the police need at most to catch the robber.
#[derive(Serialize, Deserialize)]
pub struct TimeToWin {
    time: BTreeMap<usize, Vec<UTime>>,
    nr_map_vertices: usize,
}

impl TimeToWin {
    /// returns [`Self`] if enough memory is available
    fn new(nr_map_vertices: usize, cop_moves: &CopConfigurations) -> Option<Self> {
        let mut time = BTreeMap::new();
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
    #[serde(deserialize_with = "deserialize_explicit")]
    pub symmetry: ExplicitClasses,
    pub time_to_win: TimeToWin,
    pub cop_moves: CopConfigurations,
    pub max_moves: usize,
    /// selection of at most 20 cop configurations
    /// where there is a vertex which let's the robber loose in maximal many moves
    #[serde(skip)] // marked skip to stay backwards compatible.
    pub extreme_positions: Vec<Vec<usize>>,
    pub cops_win: bool,
}

impl CopStrategy {
    pub fn pack(&self, cops: &[usize]) -> (SmallVec<[&ExplicitAutomorphism; 4]>, CompactCopsIndex) {
        let mut raw_cops = RawCops::new(cops);
        self.cop_moves.pack(&self.symmetry, &mut raw_cops)
    }

    /// values not stored because that would break the format are computed here
    pub fn compute_serde_skipped(&mut self) {
        if self.extreme_positions.is_empty() {
            let max_moves = self.max_moves as UTime;
            for index in self.cop_moves.all_positions() {
                if self.time_to_win.nr_moves_left(index).contains(&max_moves) {
                    self.extreme_positions.push(self.cop_moves.unpack(index).collect_vec());
                    if self.extreme_positions.len() == 20 {
                        break;
                    }
                }
            }
        }
    }
}

#[allow(dead_code)]
pub fn compute_cop_strategy<S>(
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<CopStrategy, String>
where
    S: SymmetryGroup + Serialize,
{
    if nr_cops == 0 {
        return Err("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return Err(format!(
            "Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."
        ));
    }

    manager.update("liste Polizeipositionen")?;
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserviere Speicher für Cop Startegie")?;
    let Some(mut f) = TimeToWin::new(edges.nr_vertices(), &cop_moves) else {
        return Err("Zu wenig Speicherplatz (Copstrat zu groß)".to_owned());
    };

    manager.update("reserviere Speicher für Queue")?;
    let Some(mut queue) = CopStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    manager.update("initialisiere Queue")?;
    for (i, index) in izip!(0.., cop_moves.all_positions()) {
        if i % 4096 == 0 {
            manager.recieve()?;
        }

        let robber_positions = f.nr_moves_left_mut(index);
        for cop_pos in cop_moves.unpack(index) {
            robber_positions[cop_pos] = 0;
            for n in edges.neighbors_of(cop_pos) {
                robber_positions[n] = 0;
            }
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
            manager.update(format!(
                "berechne Copstrategie:\n{:.2}% in Queue ({}), max {}",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len(),
                queue.curr_max()
            ))?;
            time_until_log_refresh = 2000;
        }

        let curr_times = f.nr_moves_left(curr_cop_positions);
        let mut curr_is_at_max = false;
        for (v, neighs) in izip!(0.., edges.neighbors()) {
            let max_neigh_time = neighs.fold(curr_times[v], |acc, n| acc.max(curr_times[n]));
            const OVERFLOW_NEIGH_TIME: UTime = UTime::MAX - 1;
            if max_neigh_time == OVERFLOW_NEIGH_TIME {
                return Err(format!(
                    "Cops brauchen mehr Züge als in {} passen",
                    std::any::type_name::<UTime>()
                ));
            }
            let new_time = max_neigh_time.saturating_add(1);
            times_should_cops_move_to_curr[v] = if new_time == queue.curr_max() {
                curr_is_at_max = true;
                UTime::MAX
            } else {
                debug_assert!(new_time < queue.curr_max() || new_time == UTime::MAX);
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

    manager.update("berechne Fun Facts")?;
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

    let mut res = CopStrategy {
        symmetry: sym.into_enum().into(),
        time_to_win: f,
        cop_moves,
        max_moves: max_moves as usize,
        extreme_positions: Vec::new(),
        cops_win,
    };
    res.compute_serde_skipped();

    Ok(res)
}
