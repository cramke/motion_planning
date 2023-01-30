use core::panic;

use petgraph::Undirected;
use petgraph::algo::{astar};
use petgraph::graph::{Graph, NodeIndex, NodeIndices};

use crate::node::Node2D;
use crate::boundaries::Boundaries;
use crate::optimizer::{Optimizer};
use crate::planner::planner::Planner;
use crate::planner::graph_utils as pg;
use crate::problem::Parameter;


/// # Probabilisic Road Map PRM 
/// It is an algorithm which is: 
/// - probabilistically complete and 
/// - probabilistically optimal algorithm
/// - Multi-query capable It can be used to do multi-queries.
/// 
/// # Source / Credits
/// Kavraki, L. E.; Svestka, P.; Latombe, J.-C.; Overmars, M. H. (1996), "Probabilistic roadmaps for path planning in high-dimensional configuration spaces", IEEE Transactions on Robotics and Automation, 12 (4): 566–580, doi:10.1109/70.508439
/// 
/// # Example
/// 
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
    max_batch_size: usize,
}

impl Planner for PRM {
    /// run init before starting any planning task. 
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

    /// Starts building the graph. 
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

    /// Lower cost means it is a preferrable solution. If no solution was found, the returned cost will be f64::MAX.
    fn get_solution_cost(&self) -> f64 {
        return self.solution_cost;
    }

    fn get_solution_path(&self) -> Vec<Node2D> {
        return self.solution_path.clone();
    }

    /// Returns a bool saying if any solution between start and goal was found. 
    fn is_solved(&self) -> bool {
        return self.is_solved;
    }

    /// Prints some basic statistics of the graph.
    fn print_statistics(&self, path:&str) {
        let nodes: usize = self.graph.node_count();
        println!("Graph contains {} nodes", nodes);

        let edges: usize = self.graph.edge_count();
        println!("Graph contains {} edges", edges);

        pg::write_graph_to_file(&self.graph, path);
    }

    /// Allows update of parameters after creation. 
    /// 
    /// # Arguments:
    ///   * `params.max_batch_size` - Parameter struct: Determines the graph size when to stop the algorithm.
    /// 
    fn set_params(self: &mut PRM, params: &Parameter) {
        self.max_batch_size = params.max_size;
    }

}

impl PRM {

    pub fn new(start: Node2D, goal: Node2D, bounds: Boundaries, is_collision: fn(&Node2D) -> bool, 
        is_edge_in_collision: fn() -> bool, optimizer: Box<dyn Optimizer>, param1: usize) -> Self {
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
            max_batch_size: param1,
        };
        return  setup;
    }

    fn add_batch_of_random_nodes(&mut self) -> Vec<Node2D> {
        let mut list_of_added_nodes: Vec<Node2D> = Vec::new();
    
        while list_of_added_nodes.len() < self.max_batch_size {
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
            let nearest_neighbors_type: NodeIndices = self.get_n_nearest_neighbours(node);
            let mut query_edges: Vec<(Node2D, Node2D)> = Vec::new();

            let mut nearest_neighbors: Vec<NodeIndex> = Vec::new();
            for neighbor in nearest_neighbors_type {
                nearest_neighbors.push(neighbor);
            }

            // retain keeps element were function returns true, and removes elements where false
            nearest_neighbors.retain(|x: &NodeIndex| !(self.is_edge_in_collision)());
            nearest_neighbors.retain(|x: &NodeIndex| !(pg::is_edge_already_in_graph(&self.graph, &node, *x)));

            for neighbor in nearest_neighbors {
                let node_weight: &Node2D = self.graph.node_weight(neighbor).unwrap();
                let end: Node2D = Node2D { x: node_weight.x, y: node_weight.y, idx: neighbor.index() };
                query_edges.push((node, end));
            }

            let costs: Vec<f64> = self.optimizer.get_edge_weight(query_edges.clone());
            for it in query_edges.iter().zip(costs.iter()) {
                let edge: (Node2D, Node2D) = *it.0;
                let weight: f64 = *it.1;
                pg::insert_edge_in_graph(&mut self.graph, &edge.0, edge.1.get_index_type(), weight);

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
        pg::print_graph(self.get_graph());
    }
}
