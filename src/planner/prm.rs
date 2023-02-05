use core::panic;
use std::collections::HashMap;

use petgraph::Undirected;
use petgraph::algo::{astar};
use petgraph::graph::{Graph, NodeIndex};
use geo_types::Point;
use rstar::RTree;
use wkt::ToWkt;

use crate::collision_checker::CollisionChecker;
use crate::boundaries::Boundaries;
use crate::optimizer::{Optimizer};
use crate::planner::base_planner::Planner;
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
    start: Point,
    goal: Point,
    boundaries: Boundaries,
    pub graph: Graph<Point, f64, Undirected>,
    solution: Option<(f64, Vec<NodeIndex>)>,
    pub solution_cost: f64,
    pub solution_path: Vec<Point>,
    optimizer: Box<dyn Optimizer>,
    pub is_solved: bool,
    max_size: usize,
    collision_checker: Box<dyn CollisionChecker>,
    tree: RTree<[f64; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,
    k_nearest_neighbors: usize,
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

        if self.collision_checker.is_node_colliding(&self.start) {
            panic!("Start is in collision");
        }

        if self.collision_checker.is_node_colliding(&self.goal) {
            panic!("Goal is in collision");
        }

        self.add_node(self.start);
        self.add_node(self.goal);

        if !self.optimizer.init() {
            panic!("Optimizer could not be initialized");
        }

        println!("Setup is ready for planning")

    }

    /// Starts building the graph. 
    fn run(&mut self) {
        loop {
            let added_node: Point = self.add_random_node();
            self.connect_node_to_graph(added_node);

            self.check_solution();

            if self.is_termination_criteria_met() {
                println!("Termination Criteria met");
                break;
            }
        }
    }

    /// Lower cost means it is a preferrable solution. If no solution was found, the returned cost will be f64::MAX.
    fn get_solution_cost(&self) -> f64 {
        let (cost, _) = self.solution.clone().unwrap();
        return cost;
    }

    /// Returns empty Vec if no solution was found.
    fn get_solution_path(& self) -> Vec<Point> {
        let (_, temp) = &self.solution.clone().unwrap();
        let mut path: Vec<Point> = Vec::new();
        for el in temp {
            let weight = self.graph.node_weight(*el).unwrap();
            path.push(*weight);
        }
        return path;
    }

    /// Returns a bool saying if any solution between start and goal was found. 
    fn is_solved(&self) -> bool {
        self.is_solved
    }

    /// Prints some basic statistics of the graph.
    fn print_statistics(&self, path:&str) {
        let nodes: usize = self.graph.node_count();
        println!("Graph contains {nodes} nodes");

        let edges: usize = self.graph.edge_count();
        println!("Graph contains {edges} edges");

        pg::write_graph_to_file(&self.graph, path);
    }

    /// Allows update of parameters after creation. 
    /// 
    /// # Arguments:
    ///   * `params.max_size` - Parameter struct: Determines the graph size when to stop the algorithm.
    /// 
    fn set_params(self: &mut PRM, params: &Parameter) {
        self.max_size = params.max_size;
        self.k_nearest_neighbors = params.k_nearest_neighbors;
    }

}

impl PRM {

    /// Default constructor
    pub fn new(start: Point, goal: Point, boundaries: Boundaries, optimizer: Box<dyn Optimizer>, 
        params: Parameter, collision_checker: Box<dyn CollisionChecker>) -> Self {
        let tree = RTree::new();
        let index_node_lookup: HashMap<String, NodeIndex> = HashMap::new();
        let setup: PRM = PRM { start, 
            goal, 
            boundaries,
            graph: Graph::new_undirected(),
            solution: None,
            solution_cost: f64::MAX,
            solution_path: Vec::new(),
            optimizer,
            is_solved: false,
            max_size: params.max_size,
            collision_checker,
            tree,
            index_node_lookup,
            k_nearest_neighbors: params.k_nearest_neighbors
        };
        return setup;
    }

    /// Adds a node to the graph, lookup for nodeindex to point.wkt, and the rtree.
    fn add_node(&mut self, node: Point) {
        let index = self.graph.add_node(node);
        self.index_node_lookup.insert(node.to_wkt().to_string(), index);
        self.tree.insert([node.x(), node.y()]);
    }

    /// Generates a random node and adds it to the graph, if:
    /// - It is not in collision
    /// - It is not already in the graph
    fn add_random_node(&mut self) -> Point {
        let mut candidate: Point;
        loop {
            candidate = self.boundaries.generate_random_configuration();

            if self.collision_checker.is_node_colliding(&candidate) {
                continue;
            }

            match self.index_node_lookup.get(&candidate.to_wkt().to_string()) {
                // If candidate node is found in graph, then continue with next random node.
                None => {},
                Some(_) => continue,
            }

            break;
        }
        self.add_node(candidate);
        return candidate;
    }

    /// Try to connect a node to its k nearest neigbors.
    fn connect_node_to_graph(&mut self, node: Point) {
        let mut iterator = self.tree.nearest_neighbor_iter(&[node.x(), node.y()]);
        for _ in 0..4 {
            let neighbor = iterator.next();
            let neighbor_point: Point = match neighbor {
                Some(a) => Point::new(a[0], a[1]),
                None => continue,
            };

            if node == neighbor_point {
                continue;
            }

            if self.collision_checker.is_edge_colliding(&node, &neighbor_point) {
                continue;
            }

            let weight = self.optimizer.get_edge_weight(node, neighbor_point).2;
            let a = self.index_node_lookup.get(&node.to_wkt().to_string()).unwrap();
            let b = self.index_node_lookup.get(&neighbor_point.to_wkt().to_string()).unwrap();
            self.graph.add_edge(*a, *b, weight);
        }
    }

    fn check_solution(&mut self) -> bool {
        let start = *self.index_node_lookup.get(&self.start.to_wkt().to_string()).unwrap();
        let goal = *self.index_node_lookup.get(&self.goal.to_wkt().to_string()).unwrap();
        self.solution = astar(
            &self.graph, 
            start, 
            |finish| finish == goal, 
            |e| *e.weight(), 
            |_| 0f64);

        self.is_solved = self.solution.is_some();
        return self.is_solved;
    }
    
    fn is_termination_criteria_met(&self) -> bool {
        if self.graph.node_count() >= self.max_size {
            return true;
        }
        return false;
    }

    pub fn get_graph(&self) -> &Graph<Point, f64, Undirected> {
        &self.graph
    }
    
    pub fn print_graph(&self) {
        pg::print_graph(self.get_graph())
    }
}
