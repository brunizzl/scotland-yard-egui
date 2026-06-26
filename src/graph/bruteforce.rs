use std::collections::{BTreeMap, VecDeque};

use bitvec::prelude as bv;
use itertools::{Itertools, izip};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use super::*;

pub mod thread_manager;

mod queues;
use queues::{CopStratQueue, RobberStratQueue};

mod rules;
pub use rules::*;

mod fog_util;

mod fog_bf;
pub use fog_bf::*;

mod standard_bf;
pub use standard_bf::*;

mod energy_bf;

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
    #[allow(dead_code)]
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

    pub fn sorted(mut self) -> Self {
        self.sort();
        self
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

impl std::hash::Hash for RawCops {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        for &c in &self[..] {
            state.write_usize(c);
        }
    }
}

/// returns the number of distinct mutlisets with [`cardinality`] many elements
/// chosen from a set with [`universe_size`] many elements.
fn multiset_count(universe_size: usize, cardinality: usize) -> Option<usize> {
    fn binomial_coefficient(n: usize, k: usize) -> Option<usize> {
        if k == 0 {
            return Some(1);
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

/// assume we have multisets of `multiset.len()` many elements in `0..universe_size`.
/// there are `multiset_count(universe_size, multiset.len())` many such multisets.
/// this function computes how many lexicographically smaller multisets exist.
#[allow(dead_code)]
fn multiset_index(universe_size: usize, multiset: &[usize]) -> Option<usize> {
    debug_assert!(multiset.is_sorted());
    debug_assert!(multiset.iter().all(|&x| x < universe_size));

    let mut res = 0usize;
    let mut prev_elem = 0usize;
    for (&elem, nr_later_elems) in izip!(multiset, (0..multiset.len()).rev()) {
        for smaller_val in prev_elem..elem {
            let smaller_universe = universe_size - smaller_val;
            // if elem where smaller_val instead, how many options would there be for the later elems?
            let nr_with_smaller_elem = multiset_count(smaller_universe, nr_later_elems)?;
            res = res.checked_add(nr_with_smaller_elem)?;
        }
        prev_elem = elem;
    }
    Some(res)
}

/// this corresponds to one entry in CopConfigurations:
/// [`self.fst_cop`] is expected to be a vertex symmetry class representative and the (rotated / mirrored) position of one of the cops.
/// [`self.rest_cops`] represents the compacted sorted tuple of the other cops positions (obv. rotated the same as fst_cop).
/// thus `(0..nr_symmetry_classes).contains(&self.fst_cop)`
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
    pub fn unpack(&self, index: CompactCopsIndex) -> impl Iterator<Item = usize> + Clone + use<> {
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

    pub fn all_positions(&self) -> impl Iterator<Item = CompactCopsIndex> + '_ {
        self.configurations.iter().flat_map(move |(&k, configs_part)| {
            (0..configs_part.len()).map(move |i| CompactCopsIndex { fst_index: k, rest_index: i })
        })
    }
}

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// wether that vertex is safe for the robber to stand on or not.
#[derive(Serialize, Deserialize, PartialEq, Eq)]
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

            let vec_data_len = nr_entries.div_ceil(32);
            let mut bit_vec_data = Vec::<u32>::new();
            bit_vec_data.try_reserve_exact(vec_data_len).ok()?;
            bit_vec_data.resize(vec_data_len, u32::MAX);

            let alloced_safe_data = bv::BitVec::try_from_vec(bit_vec_data).ok()?;
            let old = safe.insert(fst_index, alloced_safe_data);
            debug_assert!(old.is_none());
        }

        Some(Self { safe, nr_map_vertices })
    }

    /// clones self, except because these values can be absurdly large,
    /// we take caution to not crash the program on allocation failure.
    fn try_clone(&self) -> Option<Self> {
        let mut safe = BTreeMap::new();
        let nr_map_vertices = self.nr_map_vertices;
        for (&fst_index, safe_data) in &self.safe {
            let raw_old_data = safe_data.as_raw_slice();
            let mut raw_new_data = Vec::new();
            raw_new_data.try_reserve_exact(raw_old_data.len()).ok()?;
            raw_new_data.resize(raw_old_data.len(), 0);
            raw_new_data.clone_from_slice(raw_old_data);

            let new_data = bv::BitVec::try_from_vec(raw_new_data).ok()?;
            let old = safe.insert(fst_index, new_data);
            debug_assert!(old.is_none());
        }
        let new = Self { safe, nr_map_vertices };
        debug_assert!(&new == self);
        Some(new)
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

#[cfg(test)]
mod test {
    use super::*;
    use crate::graph::{Embedding3D, Shape};

    #[test]
    fn multiset_index_works() {
        let graph = Embedding3D::new_map_from(&Shape::SquareGrid, 5);
        let SymGroup::None(sym) = graph.sym_group() else {
            panic!();
        };
        let (_, manager) = thread_manager::build_managers();
        let cop_states = CopConfigurations::new(graph.edges(), sym, 5, &manager).unwrap();
        let fst_index = 0;
        for rest_index in 0..(cop_states.configurations[&fst_index].len()) {
            let cops_index = CompactCopsIndex { fst_index, rest_index };
            let cops_state = cop_states.eager_unpack(cops_index).sorted();
            let computed_index = multiset_index(graph.nr_vertices(), &cops_state[1..]).unwrap();
            assert_eq!(computed_index, cops_index.rest_index);
        }
    }

    #[test]
    fn test_raw_general_cop_moves_from() {
        let q10 = Embedding3D::new_map_from(&Shape::SquareGrid, 10);
        assert_eq!(q10.nr_vertices(), 100);
        let old_cops = RawCops::new(&[0, 13, 70, 85]);
        let neighborss = old_cops
            .iter()
            .map(|&c| q10.edges().neighbors_of(c).collect_vec())
            .collect_vec();
        debug_assert_eq!(&neighborss[0], &[1, 10]);
        debug_assert_eq!(&neighborss[1], &[3, 12, 14, 23]);
        debug_assert_eq!(&neighborss[2], &[60, 71, 80]);
        debug_assert_eq!(&neighborss[3], &[75, 84, 86, 95]);

        // test case 1: lazy cops means a single cop can move
        let mut lazy_1 = LazyCops
            .raw_cop_moves_from(q10.edges(), old_cops)
            .map(RawCops::sorted)
            .collect_vec();
        let mut lazy_2 = GeneralEagerCops(1)
            .raw_cop_moves_from(q10.edges(), old_cops)
            .map(RawCops::sorted)
            .collect_vec();
        lazy_1.sort_by(|a, b| (&a[..]).cmp(&b[..]));
        lazy_2.sort_by(|a, b| (&a[..]).cmp(&b[..]));
        assert_eq!(lazy_1, lazy_2);

        // test case 2: eager cops means all cops can move.
        let mut eager_1 = GeneralEagerCops(4)
            .raw_cop_moves_from(q10.edges(), old_cops)
            .map(RawCops::sorted)
            .collect_vec();
        eager_1.push(old_cops);
        let mut eager_2 = Vec::new();
        let all_eager_options = izip!(&old_cops[..], &neighborss).map(|(&c, ns)| {
            let mut res = ns.clone();
            res.push(c);
            res
        });
        for combo in all_eager_options.multi_cartesian_product() {
            let cops_step = RawCops::new(&combo);
            eager_2.push(cops_step.sorted());
        }
        eager_1.sort_by(|a, b| (&a[..]).cmp(&b[..]));
        eager_2.sort_by(|a, b| (&a[..]).cmp(&b[..]));
        assert_eq!(eager_1.len(), eager_2.len());
        assert_eq!(eager_1, eager_2);

        // between the extremes: if more cops are allowed to move,
        // all moves with fewer moving cops should still be contained.
        let mut allowed_moves: std::collections::HashSet<_> = GeneralEagerCops(0)
            .raw_cop_moves_from(q10.edges(), old_cops)
            .collect();
        assert!(allowed_moves.is_empty());
        for max_moving_cops in 1..=4 {
            let moves: std::collections::HashSet<_> = GeneralEagerCops(max_moving_cops)
                .raw_cop_moves_from(q10.edges(), old_cops)
                .collect();
            assert!(moves.is_superset(&allowed_moves));
            assert!(allowed_moves.len() < moves.len());

            for &move_ in moves.difference(&allowed_moves) {
                // works only if no cops where neighbors in old_cops
                let nr_still = move_.iter().filter(|c| old_cops.contains(c)).count();
                let nr_moving = old_cops.len() - nr_still;
                assert_eq!(nr_moving, max_moving_cops as usize);
            }

            allowed_moves = moves;
        }
    }
}
