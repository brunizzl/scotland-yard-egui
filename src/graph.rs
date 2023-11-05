
pub mod hex_pos;

use egui::Pos2;

pub type NodeID = usize;

//ID is implicit as position in vector

pub struct PlanarNode {
    pos: Pos2,
    dist_to_cops: usize,
    dist_to_robber: usize,

    //convex hull of cops (e.g. on a shortest path between two cops or on a component surrounded by such shortest paths)
    in_convex_hull: bool, 
}

impl PlanarNode {
    fn new(pos: Pos2) -> Self {
        PlanarNode {
            pos,
            dist_to_cops: usize::MAX,
            dist_to_robber: usize::MAX,
            in_convex_hull: false,
        }
    }
}

pub struct PlanarGraph {
    //position i has NodeID i
    nodes: Vec<PlanarNode>,

    //position i lists all neighbors of node i
    edges: Vec<Vec<NodeID>>,

    cops: Vec<NodeID>,
    robber: NodeID,
}

use hex_pos::*;
impl PlanarGraph {
    pub fn new_plane_tiles_hexagon(levels: usize, origin: Pos2, diameter: f32) -> Self {
        let mut nodes = Vec::new();
        let mut edges = Vec::new();

        //push origin
        nodes.push(PlanarNode::new(Pos2::new(0.0, 0.0)));
        edges.push(vec![1, 2, 3, 4, 5, 6]);

        //same formula as in HexPos::index::level_start
        //gaussian sum formula n * (n - 1) / 2, then times 6 because hexagon 
        //  and plus 1 because level 0 also has one vertex
        let total_nodes = levels * (levels - 1) * 3 + 1;
        let edge_length = diameter / std::cmp::max(1, levels) as f32;

        //push all remaining nodes
        for level in 1..levels {
            for sector in 0..6 {
                let nr_nodes_in_sector = level;
                for steps_from_sector_start in 0..nr_nodes_in_sector {
                    let name = HexPos::new_from(level, sector, steps_from_sector_start);
                    let pos = name.to_cartesian(edge_length, origin);
                    nodes.push(PlanarNode::new(pos));
                    let neighbors = [
                        name + HexPos::UNIT_H,
                        name - HexPos::UNIT_H,
                        name + HexPos::UNIT_A,
                        name - HexPos::UNIT_A,
                        name + HexPos::UNIT_D,
                        name - HexPos::UNIT_D,
                    ].map(|n| n.normalize().index());

                    if level + 1 < levels {
                        edges.push(neighbors.into());
                    }
                    else { //only keep neigbors on same and lower levels
                        let without_next_level = neighbors.into_iter().filter(|id| id < &&total_nodes).collect();
                        edges.push(without_next_level);
                    }
                }
            }
        }
        debug_assert!(total_nodes == nodes.len());

        let cops = Vec::new();
        let robber = 0;

        PlanarGraph { nodes, edges, cops, robber }
    }
}


