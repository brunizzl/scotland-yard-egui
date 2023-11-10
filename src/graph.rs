
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

    pub fn add_vertex(&mut self, pos: Pos2) -> usize {
        let new_index = self.len();
        self.positions.push(pos);
        self.neighbors.push(Vec::new());
        new_index
    }

    pub fn add_edge(&mut self, v1: usize, v2: usize) {
        self.neighbors[v1].push(v2);
        self.neighbors[v2].push(v1);
    }

    pub fn add_edges_between<'a>(&mut self, vs: impl Iterator<Item = &'a usize>) {
        for (&v1, &v2) in vs.tuple_windows() {
            self.add_edge(v1, v2);
        }
    }

    pub fn new_plane_tiles_regular_ngon(sides: usize, levels: usize, origin: Pos2, radius: f32) -> Self {
        let mut graph = Self::empty();
        graph.add_vertex(origin);
        if levels < 1 {
            return graph;
        }

        //idea: build sector by sector, first the nodes on the sector borders
        let edge_length = radius / std::cmp::max(1, levels) as f32;
        let mut sector_borders = Vec::new();
        for border_nr in 0..sides {
            let next_free_index = graph.len();
            let border = std::iter::once(0)
                .chain(next_free_index..(next_free_index + levels))
                .collect::<Vec<_>>();
            let angle = 2.0 * std::f32::consts::PI * (border_nr as f32) / (sides as f32);
            let unit_dir = vec2(angle.cos(), angle.sin()) * edge_length;
            sector_borders.push((border, unit_dir));
            for dist_to_origin in 1..(levels + 1) {
                graph.add_vertex(origin + (dist_to_origin as f32) * unit_dir);
            }
        }
        for ((b1, u1), (b2, u2)) in sector_borders.iter().circular_tuple_windows() {
            //add all edges within the (fist) border itself
            graph.add_edges_between(b1.iter());
            graph.add_edge(b1[1], b2[1]);

            let unit_diff = *u2 - *u1;
            let mut last_levels_nodes = vec![b1[1], b2[1]];
            for level in 2..(levels + 1) {
                let level_start_pos = origin + (level as f32) * *u1;

                let mut this_levels_nodes = vec![b1[level]];
                let nr_inner_nodes = level - 1;
                for node in 1..(nr_inner_nodes + 1) {
                    let node_pos = level_start_pos + (node as f32) * unit_diff;
                    let node_index = graph.add_vertex(node_pos);
                    this_levels_nodes.push(node_index);
                }
                this_levels_nodes.push(b2[level]);

                //connext level to itself
                graph.add_edges_between(this_levels_nodes.iter());
                //connect level to previous levels
                graph.add_edges_between(last_levels_nodes.iter()
                    .interleave(this_levels_nodes[1..(nr_inner_nodes + 1)].iter()));
                
                last_levels_nodes = this_levels_nodes;
            }
        }

        graph
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


