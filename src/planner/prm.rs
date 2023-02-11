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
    pub is_solved: bool,
    collision_checker: Box<dyn CollisionChecker>,
    tree: RTree<[f64; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,
    params: Parameter,
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

        println!("Setup is ready for planning");
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
        match &self.solution {
            Some(sol) => sol.0,
            None => f64::MAX,
        }
    }

    /// Returns empty Vec if no solution was found.
    fn get_solution_path(& self) -> Vec<Point> {
        match &self.solution {
            None => Vec::new(),
            Some(sol) => sol.1
                .iter()
                .map(|idx| *self.graph.node_weight(*idx).unwrap())
                .collect(),
        }
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

}

impl PRM {

    /// Standard constructor
    pub fn new(start: Point, goal: Point, boundaries: Boundaries, params: Parameter, 
        collision_checker: Box<dyn CollisionChecker>) -> Self {
        PRM { start, 
            goal, 
            boundaries,
            graph: Graph::new_undirected(),
            solution: None,
            is_solved: false,
            collision_checker,
            tree: RTree::new(),
            index_node_lookup: HashMap::new(),
            params,
        }
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
        candidate
    }

    /// Try to connect a node to its k nearest neigbors.
    fn connect_node_to_graph(&mut self, node: Point) {
        let mut iterator = self.tree.nearest_neighbor_iter_with_distance_2(&[node.x(), node.y()]);
        for _ in 0..self.params.k_nearest_neighbors {
            let neighbor = iterator.next();
            let (neighbor_point, distance): (Point, f64) = match neighbor {
                Some((node, distance)) => (Point::new(node[0], node[1]), distance),
                None => continue,
            };

            if node == neighbor_point {
                continue;
            }

            if self.collision_checker.is_edge_colliding(&node, &neighbor_point) {
                continue;
            }

            let a: NodeIndex = *self.index_node_lookup.get(&node.to_wkt().to_string()).unwrap();
            let b: NodeIndex = *self.index_node_lookup.get(&neighbor_point.to_wkt().to_string()).unwrap();
            self.graph.add_edge(a, b, distance);
        }
    }

    /// Applies the A* algorithm to the graph.
    fn check_solution(&mut self) {
        let start = *self.index_node_lookup.get(&self.start.to_wkt().to_string()).unwrap();
        let goal = *self.index_node_lookup.get(&self.goal.to_wkt().to_string()).unwrap();
        self.solution = astar(
            &self.graph, 
            start, 
            |finish| finish == goal, 
            |e| *e.weight(), 
            |_| 0f64);

        self.is_solved = self.solution.is_some();
    }

    /// Determines which criteria is used to stop the algorithm. Check the max_size parameter and compares it to the number of nodes in the graph.     
    fn is_termination_criteria_met(&self) -> bool {
        self.graph.node_count() >= self.params.max_size
    }

    /// Returns the graph object (petgraph)
    pub fn get_graph(&self) -> &Graph<Point, f64, Undirected> {
        &self.graph
    }
    
    /// Print basic information of the graph.
    pub fn print_graph(&self) {
        pg::print_graph(self.get_graph())
    }
}


mod test {

    #[test]
    fn test_prm_new() {
        use geo::Point;
        use super::PRM;
        use crate::{boundaries::Boundaries, collision_checker::{NaiveCollisionChecker, CollisionChecker}, problem::Parameter};

        let start: Point = Point::new(0f64, 0f64);
        let goal: Point = Point::new(3f64, 3f64);
        let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let params = Parameter::new(25usize, 3usize);
        let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
        let planner = PRM::new(start, goal, bounds, params, cc);

        assert!(!planner.is_solved);
    }

    #[test]
    fn test_prm_add_node() {
        use geo::Point;
        use super::PRM;
        use crate::{boundaries::Boundaries, collision_checker::{NaiveCollisionChecker, CollisionChecker}, problem::Parameter};

        let start: Point = Point::new(0f64, 0f64);
        let goal: Point = Point::new(3f64, 3f64);
        let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let params = Parameter::new(25usize, 3usize);
        let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
        let mut planner = PRM::new(start, goal, bounds, params, cc);

        assert_eq!(planner.graph.node_count(), 0);
        assert_eq!(planner.tree.size(), 0);
        assert_eq!(planner.index_node_lookup.len(), 0);
        let p1: Point = Point::new(1.8, 2.0);
        planner.add_node(p1);
        assert_eq!(planner.graph.node_count(), 1);
        assert_eq!(planner.tree.size(), 1);
        assert_eq!(planner.index_node_lookup.len(), 1);
    }

    #[test]
    fn test_prm_get_solution() {
        use geo::Point;
        use super::PRM;
        use crate::{boundaries::Boundaries, collision_checker::{NaiveCollisionChecker, CollisionChecker}, problem::Parameter};
        use crate::planner::base_planner::Planner;
        
        let start: Point = Point::new(0f64, 0f64);
        let goal: Point = Point::new(3f64, 3f64);
        let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let params = Parameter::new(25usize, 3usize);
        let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
        let planner = PRM::new(start, goal, bounds, params, cc);

        assert_eq!(planner.get_solution_cost(), f64::MAX);
        assert_eq!(planner.get_solution_path(), Vec::new());
    }
}