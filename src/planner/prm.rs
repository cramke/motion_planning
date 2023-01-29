use core::panic;

use petgraph::Undirected;
use petgraph::algo::{astar};
use petgraph::dot::{Dot, Config};
use rand::Rng;
use petgraph::graph::{Graph, NodeIndex, NodeIndices};

use crate::node::Node2D;
use crate::boundaries::Boundaries;
use crate::optimizer::{Optimizer};
use crate::planner::planner::Planner;
use crate::planner::graph_utils as pg;

pub struct PRM {
    start: Node2D,
    goal: Node2D,
    boundaries: Boundaries,
    pub graph: Graph<Node2D, f64, Undirected>,
    is_node_in_collision: fn(&Node2D) -> bool,
    is_edge_in_collision: fn() -> bool,
    solution: Option<(f64, Vec<NodeIndex>)>,
    pub solution_cost: f64,
    pub solution_path: Vec<Node2D>,
    optimizer: Box<dyn Optimizer>,
    pub is_solved: bool,
}

impl Planner for PRM {
    // run init before starting any planning task. 
    fn init(&mut self) {    
        if !self.boundaries.is_node_inside(&self.start) {
            panic!("Start is not inside boundaries.");
        }

        if !self.boundaries.is_node_inside(&self.goal) {
            panic!("Goal is not inside boundaries.");
        }

        if (self.is_node_in_collision)(&self.start) {
            panic!("Start is in collision.");
        }

        if (self.is_node_in_collision)(&self.goal) {
            panic!("Goal is in collision.");
        }

        let start_index: usize = self.graph.add_node(self.start).index();
        self.start.idx = start_index;

        let goal_index: usize = self.graph.add_node(self.goal).index();
        self.goal.idx = goal_index;

        if !self.optimizer.init() {
            panic!("Optimizer could not be initialized");
        }

        println!("Setup is ready for planning")

    }

    fn run(&mut self) {
        loop {
            let added_nodes: Vec<Node2D> = self.add_batch_of_random_nodes();
            println!("{}", added_nodes.len());
            self.connect_added_nodes_to_graph(added_nodes);

            if self.check_solution() {
                self.process_solution();
                println!("Solved");
            }

            if self.is_termination_criteria_met() {
                println!("Termination Criteria met");
            }

            break;
        }
    }

    fn get_solution_cost(&self) -> f64 {
        return self.solution_cost;
    }

    fn get_solution_path(&self) -> Vec<Node2D> {
        return self.solution_path.clone();
    }

    fn is_solved(&self) -> bool {
        return self.is_solved;
    }

    fn print_statistics(&self) {
        let nodes: usize = self.graph.node_count();
        println!("Graph contains {} nodes", nodes);

        let edges: usize = self.graph.edge_count();
        println!("Graph contains {} edges", edges);
    }

}

impl PRM {

    pub fn new(start: Node2D, goal: Node2D, bounds: Boundaries, is_collision: fn(&Node2D) -> bool, 
        is_edge_in_collision: fn() -> bool, optimizer: Box<dyn Optimizer>) -> Self {
        let setup: PRM = PRM {  start: start, 
            goal: goal, 
            boundaries: bounds,
            graph: Graph::new_undirected(),
            is_node_in_collision: is_collision,
            is_edge_in_collision: is_edge_in_collision,
            solution: None,
            solution_cost: f64::MAX,
            solution_path: Vec::new(),
            optimizer,
            is_solved: false,
        };
        return  setup;
    }

    fn add_batch_of_random_nodes(&mut self) -> Vec<Node2D> {
        let mut list_of_added_nodes: Vec<Node2D> = Vec::new();
    
        while list_of_added_nodes.len() < 8 {
            let mut node: Node2D = self.find_permissable_node();
            pg::insert_node_in_graph(&mut self.graph, &mut node);
            list_of_added_nodes.push(node);
        }
        return list_of_added_nodes;
    }

    fn find_permissable_node(&mut self) -> Node2D {
        loop {
            let node: Node2D = self.boundaries.generate_random_configuration();
            if (self.is_node_in_collision)(&node) {
                continue;
            }
    
            if pg::is_node_already_in_graph(&self.graph, &node) {
                continue;
            }
            return node;
        }
    }

    fn connect_added_nodes_to_graph(&mut self, added_nodes: Vec<Node2D>) {
        for node in added_nodes {
            let nearest_neighbors: NodeIndices = self.get_n_nearest_neighbours(node);
            for neighbor in nearest_neighbors {
                if (self.is_edge_in_collision)() {
                    continue;
                }
    
                if pg::is_edge_already_in_graph(&self.graph, &node, neighbor) {
                    continue;
                }

                let end_node2d = self.graph.node_weight(neighbor).unwrap();
                let weight: f64 = self.optimizer.get_edge_weight(&node, end_node2d);
                pg::insert_edge_in_graph(&mut self.graph, &node, neighbor, weight);
            }
        }
    }

    fn check_solution(&mut self) -> bool {
        let start: NodeIndex = NodeIndex::new(self.start.idx);
        self.solution = astar(&self.graph, start, |finish| finish == NodeIndex::new(self.goal.idx), |e| *e.weight(), |_| 0f64);
        match &self.solution {
            None => return false,
            Some(_) => {
                self.is_solved = true;
                return true;
            }
        }
    }

    fn process_solution(&mut self) {
        let (cost, temp) = &self.solution.clone().unwrap();
        self.solution_cost = *cost;
        for el in temp {
            let weight = self.graph.node_weight(*el).unwrap();
            self.solution_path.push(*weight);
        }
    }
    
    fn is_termination_criteria_met(&self) -> bool {
        return true;
    }
    
    fn get_n_nearest_neighbours(&self, _node: Node2D) -> NodeIndices {
        // TODO: Returns all nodes. A smarter algorithm would select based on proximity. Important for large graphs
        let node_iterator = self.graph.node_indices();
        return node_iterator;
    }

    pub fn get_graph(&self) -> &Graph<Node2D, f64, Undirected> {
        return &self.graph;
    }
    
    pub fn print_graph(&self) {
        println!("{:?}", Dot::with_config(self.get_graph(), &[Config::EdgeNoLabel]));
    }
}
