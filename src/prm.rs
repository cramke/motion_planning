use core::panic;

use petgraph::Undirected;
use petgraph::algo::{astar};
use rand::Rng;
use petgraph::graph::{Graph, NodeIndex, NodeIndices};

use crate::node::Node2D;
use crate::boundaries::Boundaries;
use crate::optimizer::{Optimizer};

#[derive(Clone)]
pub struct PRM {
    start: Node2D,
    goal: Node2D,
    boundaries: Boundaries,
    pub graph: Graph<[f64;2], f64, Undirected>,
    is_node_in_collision: fn(&Node2D) -> bool,
    is_edge_in_collision: fn() -> bool,
    pub solution: Option<(f64, Vec<NodeIndex>)>,
    optimizer: &'static dyn Optimizer,
}

impl PRM {

    pub fn new(start: Node2D, goal: Node2D, bounds: Boundaries, is_collision: fn(&Node2D) -> bool, 
        is_edge_in_collision: fn() -> bool, optimizer: &'static dyn Optimizer) -> Self {
        let setup: PRM = PRM {  start: start, 
            goal: goal, 
            boundaries: bounds,
            graph: Graph::new_undirected(),
            is_node_in_collision: is_collision,
            is_edge_in_collision: is_edge_in_collision,
            solution: None,
            optimizer,
        };
        return  setup;
    }

    // run init before starting any planning task. 
    pub fn init(&mut self) {    
        if !self.is_in_boundaries() {
            panic!("Start or goal not inside boundaries.");
        }

        if (self.is_node_in_collision)(&self.start) {
            panic!("Start is in collision.");
        }

        if (self.is_node_in_collision)(&self.goal) {
            panic!("Goal is in collision.");
        }

        let start_index: usize = self.graph.add_node([self.start.x, self.start.y]).index();
        self.start.idx = start_index;

        let goal_index: usize = self.graph.add_node([self.goal.x, self.goal.y]).index();
        self.goal.idx = goal_index;

        if !self.optimizer.init() {
            panic!("Optimizer could not be initialized");
        }

        println!("Setup is ready for planning")

    }

    pub fn run(&mut self) {
        loop {
            let added_nodes: Vec<Node2D> = self.add_batch_of_random_nodes();
            println!("{}", added_nodes.len());
            self.connect_added_nodes_to_graph(added_nodes);

            if self.is_problem_solved() {
                println!("Solved");
            }

            if self.is_termination_criteria_met() {
                println!("Termination Criteria met");
            }

            break;
        }
    }

    fn add_batch_of_random_nodes(&mut self) -> Vec<Node2D> {
        let mut list_of_added_nodes: Vec<Node2D> = Vec::new();
    
        while list_of_added_nodes.len() < 200 {
            let mut node: Node2D = self.find_permissable_node();
            self.insert_node_in_graph(&mut node);
            list_of_added_nodes.push(node);
        }
        return list_of_added_nodes;
    }

    fn find_permissable_node(&self) -> Node2D {
        loop {
            let node: Node2D = self.generate_random_configuration();
            if (self.is_node_in_collision)(&node) {
                continue;
            }
    
            if self.is_node_already_in_graph() {
                continue;
            }
            return node;
        }
    }

    fn insert_node_in_graph(&mut self, node: &mut Node2D) {
        let index: usize = self.graph.add_node([node.x, node.y]).index();
        node.idx = index;
    }

    fn is_in_boundaries(&self) -> bool {
        return true;
    }

    fn connect_added_nodes_to_graph(&mut self, added_nodes: Vec<Node2D>) {
        for node in added_nodes {
            let nearest_neighbors: NodeIndices = self.get_n_nearest_neighbours(node);
            for neighbor in nearest_neighbors {
                if (self.is_edge_in_collision)() {
                    continue;
                }
    
                if self.is_edge_already_in_graph() {
                    continue;
                }
    
                self.insert_edge_in_graph(&node, neighbor);
            }
        }
    }

    fn generate_random_configuration(&self) -> Node2D {
        let mut rng = rand::thread_rng();
        let x: f64 = rng.gen_range(self.boundaries.x_lower..self.boundaries.x_upper);
        let y: f64 = rng.gen_range(self.boundaries.y_lower..self.boundaries.y_upper);
        let node: Node2D = Node2D { x: x, y: y, idx: 0};
        return node;
    }

    fn insert_edge_in_graph(&mut self, begin: &Node2D, end: NodeIndex) {
        let node_coords = self.graph.node_weight(end).unwrap();
        let end_node: Node2D = Node2D::new_index(node_coords[0], node_coords[1], end.index());
        let weight: f64 = self.optimizer.get_edge_weight(begin, &end_node);
        let a: NodeIndex<u32> = NodeIndex::new(begin.idx);
        if a == end { // do not insert edge from a node to itself
            return;
        }
        self.graph.add_edge(a, end, weight);

    }

    fn is_problem_solved(&mut self) -> bool {
        let start: NodeIndex = NodeIndex::new(self.start.idx);
        self.solution = astar(&self.graph, start, |finish| finish == NodeIndex::new(self.goal.idx), |e| *e.weight(), |_| 0f64);
        match &self.solution {
            None => return false,
            Some(_a) => {
                println!("found a solution.");
                return true;
            }
        }
    }
    
    fn is_termination_criteria_met(&self) -> bool {
        return true;
    }
    
    fn is_edge_already_in_graph(&self) -> bool {
        return false;
    }
    
    fn is_node_already_in_graph(&self) -> bool {
        return false;
    }
    
    fn get_n_nearest_neighbours(&self, _node: Node2D) -> NodeIndices {
        // TODO: Returns all nodes. A smarter algorithm would select based on proximity. Important for large graphs
        let node_iterator = self.graph.node_indices();
        return node_iterator;
    }

    pub fn get_graph(&self) -> &Graph<[f64;2], f64, Undirected> {
        return &self.graph;
    }

}
