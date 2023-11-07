
pub mod hex_pos;

use egui::Pos2;

use self::hex_pos::HexPos;

pub type NodeID = usize;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Bool3 { No, Yes, Perhaps }


struct Advantage {
    robber_closer: bool,
    dist_to_next: isize, //dist to next figure, indepentnt from that figure's lawlessness
}

//input interpreted the following way
//if robber as least close as next cop: nr nodes - dist to robber
//else: - nr nodes + dist to next cop
//uninitialized: 0
fn isize_to_advantage(nr_nodes: isize, a: isize) -> Advantage {
    Advantage{ robber_closer: a >= 0, dist_to_next: nr_nodes - a.abs() }
}

fn advantage_to_isize(nr_nodes: isize, a: Advantage) -> isize {
    if a.robber_closer {
        nr_nodes - a.dist_to_next
    }
    else {
        -nr_nodes + a.dist_to_next
    }
}


//ID is implicit as position in vector
pub struct Node {
    pub pos: Pos2,

    //if robber closer than next cop: nr nodes - dist to robber
    //else: - nr nodes + dist to next cop
    //uninitialized: 0
    pub robber_advantage: isize,

    //convex hull of cops (e.g. on a shortest path between two cops or on a component surrounded by such shortest paths)
    pub in_convex_hull: Bool3, 
}

impl Node {
    fn new(pos: Pos2) -> Self {
        Node {
            pos,
            robber_advantage: 0,
            in_convex_hull: Bool3::Perhaps,
        }
    }
}

pub struct Graph {
    //position i has NodeID i
    nodes: Vec<Node>,

    //position i lists all neighbors of node i
    edges: Vec<Vec<NodeID>>,
}

impl Graph {
    pub fn empty() -> Self {
        Graph { nodes: Vec::new(), edges: Vec::new() }
    }

    pub fn new_plane_tiles_hexagon(levels: usize, origin: Pos2, radius: f32) -> (Self, Vec<hex_pos::HexPos>) {
        let mut nodes = Vec::new();
        let mut names = Vec::new();
        let mut edges = Vec::new();

        use hex_pos::*;

        //push origin
        nodes.push(Node::new(origin));
        names.push(HexPos::new(0, 0, 0));
        if levels > 0 {
            edges.push(vec![1, 2, 3, 4, 5, 6]);
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
                    names.push(name);
                    let pos = name.to_cartesian(edge_length, origin);
                    nodes.push(Node::new(pos));
                    let neighbors = [
                        name + HexPos::UNIT_H,
                        name - HexPos::UNIT_H,
                        name + HexPos::UNIT_A,
                        name - HexPos::UNIT_A,
                        name + HexPos::UNIT_D,
                        name - HexPos::UNIT_D,
                    ].map(|n| n.normalize().index());

                    if level < levels {
                        edges.push(neighbors.into());
                    }
                    else { //only keep neigbors on same and lower levels
                        let without_next_level = neighbors.into_iter().filter(|id| id < &&total_nodes).collect();
                        edges.push(without_next_level);
                    }
                }
            }
        }

        (Graph { nodes, edges }, names)
    }

    pub fn mut_nodes(&mut self) -> &mut [Node] {
        &mut self.nodes
    }

    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub fn edges(&self) -> impl Iterator<Item = (&Node, &Node)> {
        self.edges
            .iter()
            .enumerate()
            .flat_map(move |(i1, neighbors)| 
                neighbors
                    .iter()
                    .map(move |i2| (&self.nodes[i1], &self.nodes[*i2])))        
    }

    pub fn neigbors_of(&self, node: usize) -> impl Iterator<Item = (usize, &Node)> {
        self.edges[node].iter().map(move |i| (*i, &self.nodes[*i]))
    }
    
    pub fn calc_robber_advantage(&mut self, robber: usize, cops: &[usize]) {
        let nr_nodes = self.nodes.len() as isize;
        for node in self.mut_nodes() {
            node.robber_advantage = 0;
        }
        self.nodes[robber].robber_advantage = nr_nodes;
        let mut update_next = std::collections::VecDeque::with_capacity(self.nodes.len() / 10);

        update_next.push_back(robber);
        //compute all advantages as if there where no cops
        while let Some(node) = update_next.pop_front() {
            let own_advantage = self.nodes[node].robber_advantage;
            for neigbor in &self.edges[node] {
                let neighbor_node = &mut self.nodes[*neigbor];
                if neighbor_node.robber_advantage < own_advantage - 1 {
                    neighbor_node.robber_advantage = own_advantage - 1;
                    update_next.push_back(*neigbor);
                }
            }
        }

        //now add cops
        for cop in cops {
            self.nodes[*cop].robber_advantage = -nr_nodes;
            update_next.push_back(*cop);
        }
        while let Some(node) = update_next.pop_front() {
            let own_advantage = isize_to_advantage(nr_nodes, self.nodes[node].robber_advantage);
            let own_dist = own_advantage.dist_to_next;
            assert!(!own_advantage.robber_closer);
            for neigbor in &self.edges[node] {
                let neighbor_node = &mut self.nodes[*neigbor];
                let old = isize_to_advantage(nr_nodes, neighbor_node.robber_advantage);
                if !old.robber_closer && own_dist + 1 < old.dist_to_next 
                || old.robber_closer && own_dist + 1 <= old.dist_to_next {
                    update_next.push_back(*neigbor);
                    neighbor_node.robber_advantage = advantage_to_isize(nr_nodes, 
                        Advantage{ robber_closer: false, dist_to_next: own_dist + 1});
                }
            }
        }
    }

    pub fn calc_hex_convex_police_hull(&mut self, names: &[HexPos], cops: &[usize]) {
        for v in self.mut_nodes() {
            v.in_convex_hull = Bool3::Perhaps;
        }

        let mut mark_hull_at = |hex: HexPos| {
            let index = hex.index();
            if index < names.len() {
                self.mut_nodes()[index].in_convex_hull = Bool3::Yes;
            }
        };

        for c1 in cops {
            for c2 in cops {
                let mut c1_hex = names[*c1];
                let mut c2_hex = names[*c2];

                let diff = c2_hex - c1_hex; 
                for (len, dir) in [
                    (diff.horizontal.abs(), HexPos::UNIT_H * diff.horizontal.signum()),
                    (diff.ascending.abs(), HexPos::UNIT_A * diff.ascending.signum()),
                    (diff.descending.abs(), HexPos::UNIT_D * diff.descending.signum()),
                ] {
                    for _ in 0..len {
                        mark_hull_at(c1_hex);
                        mark_hull_at(c2_hex);
                        c1_hex = c1_hex + dir;
                        c2_hex = c2_hex - dir;
                    }
                }
            }
        }        
        let mut update_next = std::collections::VecDeque::with_capacity(self.nodes.len() / 10);
        for (i, (neighbors, node)) in self.edges.iter().zip(self.nodes.iter_mut()).enumerate() {
            if neighbors.len() < 6 && node.in_convex_hull == Bool3::Perhaps {
                node.in_convex_hull = Bool3::No;
                update_next.push_back(i);
            }
        }

        while let Some(i) = update_next.pop_front() {
            for neighbor in &self.edges[i] {
                if self.nodes[*neighbor].in_convex_hull == Bool3::Perhaps {
                    self.nodes[*neighbor].in_convex_hull = Bool3::No;
                    update_next.push_back(*neighbor);
                }
            }
        }

        for node in self.mut_nodes() {
            if node.in_convex_hull == Bool3::Perhaps {
                node.in_convex_hull = Bool3::Yes;
            }
        }
    }
}


