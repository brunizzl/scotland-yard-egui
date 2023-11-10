
pub mod hex_pos;

use std::collections::VecDeque;

use egui::{Pos2, vec2};
use itertools::Itertools;

pub struct Graph {
    positions: Vec<Pos2>,

    //position i lists all neighbors of node i
    neighbors: Vec<Vec<usize>>,
}

impl Graph {
    pub fn len(&self) -> usize {
        debug_assert_eq!(self.positions.len(), self.neighbors.len());
        self.positions.len()
    }

    pub fn empty() -> Self {
        Graph { positions: Vec::new(), neighbors: Vec::new() }
    }

    pub fn new_plane_tiles_hexagon(levels: usize, origin: Pos2, radius: f32) -> Self {
        let mut positions = Vec::new();
        let mut neighbors = Vec::new();

        use hex_pos::*;

        //push origin
        positions.push(origin);
        neighbors.push(if levels > 0 {
            vec![1, 2, 3, 4, 5, 6]
        }
        else {
            Vec::new()
        });

        //same formula as in HexPos::index::level_start
        //gaussian sum formula n * (n - 1) / 2, then times 6 because hexagon 
        //  and plus 1 because level 0 also has one vertex
        let total_nodes = levels * (levels + 1) * 3 + 1;
        let edge_length = radius / std::cmp::max(1, levels) as f32;

        //push all remaining nodes
        for level in 1..levels + 1 {
            for sector in 0..6 {
                let nr_nodes_in_sector = level;
                for steps_from_sector_start in 0..nr_nodes_in_sector {
                    let name = HexPos::new_from(level, sector, steps_from_sector_start);
                    let pos = name.to_cartesian(edge_length, origin);
                    positions.push(pos);
                    let this_neighbors = [
                        name + HexPos::UNIT_H,
                        name - HexPos::UNIT_H,
                        name + HexPos::UNIT_A,
                        name - HexPos::UNIT_A,
                        name + HexPos::UNIT_D,
                        name - HexPos::UNIT_D,
                    ].map(|n| n.normalize().index());

                    if level < levels {
                        neighbors.push(this_neighbors.into());
                    }
                    else { //only keep neigbors on same and lower levels
                        let without_next_level = this_neighbors.into_iter().filter(|id| id < &&total_nodes).collect();
                        neighbors.push(without_next_level);
                    }
                }
            }
        }

        Graph { positions, neighbors }
    }

    pub fn new_plane_tiles_pentagon(levels: usize, origin: Pos2, radius: f32) -> Self {
        let mut positions = Vec::new();
        let mut neighbors = Vec::<Vec<_>>::new();

        let add_edge = |neighs: &mut Vec<Vec<_>>, v1: usize, v2: usize| {
            neighs[v1].push(v2);
            neighs[v2].push(v1);
        };
        let add_edges_between = |neighs: &mut Vec<Vec<_>>, vertices: &[usize]| {
            for (&v1, &v2) in vertices.iter().tuple_windows() {
                add_edge(neighs, v1, v2);
            }
        };

        //push origin
        positions.push(origin);
        neighbors.push(Vec::new());
        if levels < 1 {
            return Graph { positions, neighbors };
        }
        let mut next_free_index = 1;

        //idea: build sector by sector, first the nodes on the sector borders
        let edge_length = radius / std::cmp::max(1, levels) as f32;
        let mut sector_borders = Vec::new();
        for border_nr in 0..5 {
            let border = std::iter::once(0)
                .chain(next_free_index..(next_free_index + levels))
                .collect::<Vec<_>>();
            next_free_index += levels;
            let angle = 2.0 * std::f32::consts::PI * (border_nr as f32) / 5.0;
            let unit_dir = vec2(angle.cos(), angle.sin()) * edge_length;
            sector_borders.push((border, unit_dir));
            for dist_to_origin in 1..(levels + 1) {
                positions.push(origin + (dist_to_origin as f32) * unit_dir);
                neighbors.push(Vec::new());
            }
            debug_assert_eq!(positions.len(), neighbors.len());
            debug_assert_eq!(positions.len(), next_free_index);
        }
        for ((b1, u1), (b2, u2)) in sector_borders.iter().circular_tuple_windows() {
            let unit_diff = *u2 - *u1;

            //add all edges within the border itself
            add_edges_between(&mut neighbors, &b1);
            add_edge(&mut neighbors, b1[1], b2[1]);

            let mut last_levels_nodes = vec![b1[1], b2[1]];
            for level in 2..(levels + 1) {
                let level_start_pos = origin + (level as f32) * *u1;

                let mut this_levels_nodes = vec![b1[level]];
                let nr_inner_nodes = level - 1;
                for node in 1..(nr_inner_nodes + 1) {
                    this_levels_nodes.push(next_free_index);                    
                    next_free_index += 1;

                    positions.push(level_start_pos + (node as f32) * unit_diff);
                    neighbors.push(Vec::new());

                    debug_assert_eq!(positions.len(), neighbors.len());
                    debug_assert_eq!(positions.len(), next_free_index);
                }
                this_levels_nodes.push(b2[level]);

                //connext level to itself
                add_edges_between(&mut neighbors, &this_levels_nodes);
                //connect level to previous levels
                for ((v_last1, v_last2), v_curr) in last_levels_nodes.iter()
                    .tuple_windows()
                    .zip(this_levels_nodes[1..(nr_inner_nodes + 1)].iter()) {
                    
                        add_edge(&mut neighbors, *v_last1, *v_curr);
                        add_edge(&mut neighbors, *v_last2, *v_curr);
                }
                last_levels_nodes = this_levels_nodes;
            }
        }

        Graph { positions, neighbors }
    }

    pub fn nodes(&self) -> &[Pos2] {
        &self.positions
    }

    pub fn edges(&self) -> impl Iterator<Item = (&Pos2, &Pos2)> {
        self.neighbors
            .iter()
            .enumerate()
            .flat_map(move |(i1, neighbors)| 
                neighbors
                    .iter()
                    .map(move |i2| (&self.positions[i1], &self.positions[*i2])))        
    }

    pub fn neighbors(&self, node: usize) -> &[usize] {
        &self.neighbors[node]
    } 

    pub fn neigbors_of(&self, node: usize) -> impl Iterator<Item = (usize, &Pos2)> {
        self.neighbors[node].iter().map(move |i| (*i, &self.positions[*i]))
    }

    //everything in queue is starting point and expected to already have the correct distance
    pub fn calc_distances_to(&self, queue: &mut VecDeque<usize>, distances: &mut Vec<usize>) {
        while let Some(node) = queue.pop_front() {
            let dist = distances[node];
            for &neigh in &self.neighbors[node] {
                if distances[neigh] > dist + 1 {
                    distances[neigh] = dist + 1;
                    queue.push_back(neigh);
                }
            }
        }
    }

    //paintbucket tool
    pub fn recolor_region<Color: Eq + Clone>(&self, (old, new): (Color, Color), 
        colors: &mut [Color], queue: &mut VecDeque<usize>) {

        while let Some(node) = queue.pop_front() {
            for &neigh in self.neighbors(node) {
                if colors[neigh] == old {
                    colors[neigh] = new.clone();
                    queue.push_back(neigh);
                }
            }
        }
    }
}


