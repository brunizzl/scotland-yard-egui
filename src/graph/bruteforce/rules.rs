use super::*;

/// specifies which positions the police can reach within a single turn.
/// (perhaps todo: also extend this to enable more flexible robber behavior)
///
/// note: the assumption is always to have the pieces placed on the graph vertices.
/// only how far a single piece can move / how many cop pieces can move in a single turn can change.
pub trait Rules {
    /// for the current mutiset of police positions [`cops`],
    /// enumerate every multiset of police positions, that can be reached by the police in a single round,
    /// except the do-nothing move.
    ///
    /// note: because the returned [`RawCops`] are meant to represent a multiset,
    /// no guarantee about any ordering in a single [`RawCops`] value is given.
    /// in particular, it may not be the case, that the vertices at each index of a
    /// returned value correspond to the pieces at the same indices of the input.
    fn raw_cop_moves_from<'a>(
        &'a self,
        edges: &'a EdgeList,
        cops: RawCops,
    ) -> impl Iterator<Item = RawCops> + 'a + Clone;

    /// for the current (equivalence class representative of a) mutiset of police positions [`positions`],
    /// enumerate every (equivalence class representative of a) multiset of police positions,
    /// that can be reached by the police in a single round.
    ///
    /// The vertices are connected via [`edges`] and have automorphism group [`sym`].
    /// because these positions stored in [`CopConfigurations`] may be stored in a different rotation and / or flipped,
    /// that rotation + flip to get from the move as rotated in the input to the output
    /// is also returned (see [`CopConfigurations::pack`]).
    fn cop_moves_from<'a, S: SymmetryGroup>(
        &'a self,
        cop_configs: &'a CopConfigurations,
        edges: &'a EdgeList,
        sym: &'a S,
        positions: CompactCopsIndex,
    ) -> impl Iterator<Item = (SmallVec<[&'a S::Auto; 4]>, CompactCopsIndex)> + 'a + Clone {
        self.raw_cop_moves_from(edges, cop_configs.eager_unpack(positions))
            .map(|mut cops| cop_configs.pack(sym, &mut cops))
    }

    /// as this was assumed for some time in the program,
    /// it is important to be able to test for this case.
    const IS_LAZY: bool = false;
}

/// at most one cop can move each round and with speed 1
#[derive(Debug, Clone, Copy)]
pub struct LazyCops;

impl Rules for LazyCops {
    fn raw_cop_moves_from<'a>(
        &'a self,
        edges: &'a EdgeList,
        cops: RawCops,
    ) -> impl Iterator<Item = RawCops> + 'a + Clone {
        (0..cops.nr_cops).flat_map(move |i| {
            let cop_i_pos = cops[i];
            edges.neighbors_of(cop_i_pos).map(move |n| {
                let mut unpacked_neigh = cops;
                unpacked_neigh[i] = n;
                unpacked_neigh
            })
        })
    }

    const IS_LAZY: bool = true;
}

/// at most <argument> many cops can move each round and with speed 1
#[derive(Debug, Clone, Copy)]
pub struct GeneralEagerCops(pub u32);

impl Rules for GeneralEagerCops {
    fn raw_cop_moves_from<'a>(
        &'a self,
        edges: &'a EdgeList,
        cops: RawCops,
    ) -> impl Iterator<Item = RawCops> + 'a + Clone {
        let max_moving_cops = self.0;

        use arrayvec::ArrayVec;
        // start at 1 to bypass the do-nothing move
        let moving_cop_bits = 1..(1 << (cops.len()));
        let allowed_moving_combinations =
            moving_cop_bits.filter(move |&bits: &i32| bits.count_ones() <= max_moving_cops);

        allowed_moving_combinations.flat_map(move |select_moving| {
            let is_selected = |i| select_moving & (1 << i) != 0;

            let non_moving_cops = RawCops::from_iter(
                (0..cops.len()).map(|i| if is_selected(i) { usize::MAX } else { cops[i] }),
            );
            debug_assert_eq!(non_moving_cops.nr_cops, cops.nr_cops);

            // idea: curr_moving is kinda like a c++ iterator array,
            // as in we can take the current value of the iterator (slice index 0)
            // whilst also advancing the iterator (set slice to subslice of itself).
            // if the subslice is empty, all move options for this cop are exausted,
            // thus we advance the cop further down the list and reset ouselfs to the full list of options.
            // the moving_cops_neighs variable is read every time a part of curr_moving is reset.
            // because this process should stop after the options for the last cop are exausted (and not reset)
            // we set the "refreshing value" for the last cop to length 0.
            let mut moving_cops_neighs: ArrayVec<_, MAX_COPS> = (0..cops.len())
                .filter_map(|i| is_selected(i).then_some(edges.raw_neighbors_of(cops[i])))
                .collect();
            let mut curr_moving = moving_cops_neighs.clone();
            if let Some(l) = moving_cops_neighs.last_mut() {
                // set the "refreshing value" to be empty
                *l = &l[0..0];
            }

            let yield_next = move || -> Option<_> {
                let res = {
                    let mut res = non_moving_cops;
                    let mut next_unused_i = 0;
                    for step_options in &curr_moving {
                        // this can only be any other than the step_options of the last cop,
                        // if some cop stands on an isolated vertex.
                        if step_options.is_empty() {
                            return None;
                        }
                        while res.cops[next_unused_i] != usize::MAX {
                            next_unused_i += 1;
                        }
                        res.cops[next_unused_i] = step_options[0].get().unwrap();
                    }
                    debug_assert!(res.iter().all(|&c| c != usize::MAX));
                    Some(res)
                };

                // advance to the next value (this is like a counter, except every digit potentially has a different base)
                for (all_steps, left_steps) in izip!(&moving_cops_neighs, &mut curr_moving) {
                    *left_steps = &left_steps[1..];
                    if left_steps.is_empty() {
                        *left_steps = all_steps;
                    } else {
                        break;
                    }
                }
                res
            };

            std::iter::from_fn(yield_next)
        })
    }
}

/// this is an enumeration of the types implementing [`Rules`].
/// in principle, these types may carry more data than is stored in their counterpart here.
/// this additional data however should only be cashed information like distances between vertices.
/// Note: as of today (November 2025) no rules make use of this possibility.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DynRules {
    #[default]
    Lazy,
    Eager,
    GeneralEagerCops(u32),
}

impl DynRules {
    pub fn raw_cop_moves_from<'a>(&'a self, edges: &'a EdgeList, cops: RawCops) -> Vec<RawCops> {
        match self {
            Self::Lazy => {
                let rs = LazyCops;
                rs.raw_cop_moves_from(edges, cops).collect_vec()
            },
            Self::Eager => {
                let rs = GeneralEagerCops(cops.len() as u32);
                rs.raw_cop_moves_from(edges, cops).collect_vec()
            },
            Self::GeneralEagerCops(n) => {
                let rs = GeneralEagerCops(*n);
                rs.raw_cop_moves_from(edges, cops).collect_vec()
            },
        }
    }

    pub fn compute_cop_strategy<S: SymmetryGroup + Serialize>(
        self,
        nr_cops: usize,
        edges: EdgeList,
        sym: S,
        manager: &thread_manager::LocalManager,
    ) -> Result<CopStrategy, String> {
        match self {
            Self::Lazy => {
                let rs = LazyCops;
                compute_cop_strategy(rs, nr_cops, edges, sym, manager)
            },
            Self::Eager => {
                let rs = GeneralEagerCops(nr_cops as u32);
                compute_cop_strategy(rs, nr_cops, edges, sym, manager)
            },
            Self::GeneralEagerCops(n) => {
                let rs = GeneralEagerCops(n);
                compute_cop_strategy(rs, nr_cops, edges, sym, manager)
            },
        }
    }

    pub fn compute_safe_robber_positions<S: SymmetryGroup + Serialize>(
        self,
        nr_cops: usize,
        edges: EdgeList,
        sym: S,
        manager: &thread_manager::LocalManager,
    ) -> Result<Outcome, String> {
        match self {
            Self::Lazy => {
                let rs = LazyCops;
                compute_safe_robber_positions(rs, nr_cops, edges, sym, manager)
            },
            Self::Eager => {
                let rs = GeneralEagerCops(nr_cops as u32);
                compute_safe_robber_positions(rs, nr_cops, edges, sym, manager)
            },
            Self::GeneralEagerCops(n) => {
                let rs = GeneralEagerCops(n);
                compute_safe_robber_positions(rs, nr_cops, edges, sym, manager)
            },
        }
    }

    pub fn verify_continuity(
        self,
        data: &RobberWinData,
        edges: &EdgeList,
        manager: &thread_manager::LocalManager,
    ) -> Result<(), String> {
        match self {
            Self::Lazy => {
                let rs = LazyCops;
                verify_continuity_robber(rs, data, edges, manager)
            },
            Self::Eager => {
                let rs = GeneralEagerCops(data.cop_moves.nr_cops as u32);
                verify_continuity_robber(rs, data, edges, manager)
            },
            Self::GeneralEagerCops(n) => {
                let rs = GeneralEagerCops(n);
                verify_continuity_robber(rs, data, edges, manager)
            },
        }
    }

    /// this is meant as an identifier for e.g. file names.
    pub fn id_string(self, nr_cops: usize) -> String {
        match self {
            // Lazy used to be the only avaliable ruleset.
            // file names without explicitly stated rules are thus meant as lazy cops.
            Self::Lazy => String::new(),
            Self::Eager => format!("-Eager_{nr_cops}"),
            Self::GeneralEagerCops(nr) => format!("-Eager_{nr}"),
        }
    }

    /// this is meant as an idenifier in the ui
    pub fn name(self) -> String {
        match self {
            Self::Lazy => "Lazy".into(),
            Self::Eager => "Eager".into(),
            Self::GeneralEagerCops(n) => format!("Mix({n})"),
        }
    }
}
