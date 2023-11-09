
pub mod hex_pos;

use std::collections::VecDeque;

use egui::Pos2;

use super::Bool3;

pub struct Graph {
    positions: Vec<Pos2>,

    //position i lists all neighbors of node i
    neighbors: Vec<Vec<usize>>,
}

impl Graph {
    pub fn empty() -> Self {
        Graph { positions: Vec::new(), neighbors: Vec::new() }
    }

    pub fn new_plane_tiles_hexagon(levels: usize, origin: Pos2, radius: f32) -> Self {
        let mut positions = Vec::new();
        let mut neighbors = Vec::new();

        use hex_pos::*;

        //push origin
        positions.push(origin);
        if levels > 0 {
            neighbors.push(vec![1, 2, 3, 4, 5, 6]);
        }

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

    pub fn mut_nodes(&mut self) -> &mut [Pos2] {
        &mut self.positions
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

    pub fn neigbors_of(&self, node: usize) -> impl Iterator<Item = (usize, &Pos2)> {
        self.neighbors[node].iter().map(move |i| (*i, &self.positions[*i]))
    }

    //to save on allocations, this function expects both the storage place for 
    //  the result and the used queue to come provided.
    pub fn calc_distances_to(&self, pos: usize, distances: &mut Vec<usize>, queue: &mut VecDeque<usize>) {
        distances.clear();
        distances.resize(self.nodes().len(), usize::MAX);
        distances[pos] = 0;

        queue.clear();
        queue.push_back(pos);

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
}


