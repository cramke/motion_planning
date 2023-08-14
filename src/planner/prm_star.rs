use core::panic;
use std::collections::HashMap;
use std::marker::PhantomData;

use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;
use rstar::RTree;

use crate::boundaries::Boundaries;
use crate::collision_checker::{CollisionChecker, NaiveCollisionChecker};
use crate::core::Metric2D;
use crate::optimizer::{DefaultOptimizer, Optimizer};
use crate::planner::base_planner::Planner;
use crate::planner::graph_utils as pg;
use crate::space::Point;

pub struct Config {
    default_nearest_neighbors: u8,
    max_size: usize,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            default_nearest_neighbors: 10u8,
            max_size: 32usize,
        }
    }
}

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
pub struct PRMstar<T: Metric2D> {
    pub start: Point<T>,
    pub goal: Point<T>,
    pub boundaries: Boundaries<T>,
    pub graph: Graph<Point<T>, T, Undirected>,
    pub solution: Option<(T, Vec<NodeIndex>)>,
    pub optimizer: Box<dyn Optimizer<T>>,
    pub is_solved: bool,
    pub collision_checker: Box<dyn CollisionChecker<T>>,
    tree: RTree<[T; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,
    pub config: Config,
}

impl<T: Metric2D> Planner<T> for PRMstar<T> {
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

        println!("Setup is ready for planning");
    }

    /// Starts building the graph.
    fn _run(&mut self) {
        loop {
            let added_node: Point<T> = self.add_random_node();
            self.connect_node_to_graph(added_node);

            self.check_solution();

            if self.is_termination_criteria_met() {
                println!("Termination Criteria met");
                break;
            }
        }
    }

    /// Lower cost means it is a preferrable solution. If no solution was found, the returned cost will be f64::MAX.
    fn get_solution_cost(&self) -> T {
        match &self.solution {
            Some(sol) => sol.0,
            None => T::MAX,
        }
    }

    /// Returns empty Vec if no solution was found.
    fn get_solution_path(&self) -> Vec<Point<T>> {
        match &self.solution {
            None => Vec::new(),
            Some(sol) => sol
                .1
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
    fn print_statistics(&self, path: &str) {
        let nodes: usize = self.graph.node_count();
        println!("Graph contains {nodes} nodes");

        let edges: usize = self.graph.edge_count();
        println!("Graph contains {edges} edges");

        pg::write_graph_to_file(&self.graph, path);
    }
}

impl<T: Metric2D> PRMstar<T> {
    /// Standard constructor
    pub fn new(
        start: Point<T>,
        goal: Point<T>,
        boundaries: Boundaries<T>,
        optimizer: Box<dyn Optimizer<T>>,
        collision_checker: Box<dyn CollisionChecker<T>>,
    ) -> Self {
        PRMstar {
            start,
            goal,
            boundaries,
            graph: Graph::new_undirected(),
            solution: None,
            optimizer,
            is_solved: false,
            collision_checker,
            tree: RTree::new(),
            index_node_lookup: HashMap::new(),
            config: Config::default(),
        }
    }

    /// Adds a node to the graph, lookup for nodeindex to point.wkt, and the rtree.
    fn add_node(&mut self, node: Point<T>) {
        let index = self.graph.add_node(node);
        self.index_node_lookup
            .insert(node.to_wkt().to_string(), index);
        self.tree.insert([node.x, node.y]);
    }

    /// Generates a random node and adds it to the graph, if:
    /// - It is not in collision
    /// - It is not already in the graph
    fn add_random_node(&mut self) -> Point<T> {
        let mut candidate: Point<T>;
        loop {
            candidate = self.boundaries.generate_random_configuration();

            if self.collision_checker.is_node_colliding(&candidate) {
                continue;
            }

            match self.index_node_lookup.get(&candidate.to_wkt().to_string()) {
                // If candidate node is found in graph, then continue with next random node.
                None => {}
                Some(_) => continue,
            }

            break;
        }
        self.add_node(candidate);
        candidate
    }

    /// Try to connect a node to its k nearest neigbors.
    fn connect_node_to_graph(&mut self, node: Point<T>) {
        let mut iterator = self.tree.nearest_neighbor_iter(&[node.x, node.y]);
        for _ in 0..self.config.default_nearest_neighbors {
            let neighbor: Option<&[T; 2]> = iterator.next();
            let neighbor_point: Point<T> = match neighbor {
                Some(node) => Point {
                    x: node[0],
                    y: node[1],
                },
                None => continue,
            };

            if node == neighbor_point {
                continue;
            }

            if self
                .collision_checker
                .is_edge_colliding(&node, &neighbor_point)
            {
                continue;
            }

            let weight: T = self.optimizer.get_edge_weight(node, neighbor_point).2;
            let a: NodeIndex = *self
                .index_node_lookup
                .get(&node.to_wkt().to_string())
                .unwrap();
            let b: NodeIndex = *self
                .index_node_lookup
                .get(&neighbor_point.to_wkt().to_string())
                .unwrap();
            self.graph.add_edge(a, b, weight);
        }
    }

    /// Applies the A* algorithm to the graph.
    fn check_solution(&mut self) {
        let start = *self
            .index_node_lookup
            .get(&self.start.to_wkt().to_string())
            .unwrap();
        let goal = *self
            .index_node_lookup
            .get(&self.goal.to_wkt().to_string())
            .unwrap();
        self.solution = astar(
            &self.graph,
            start,
            |finish| finish == goal,
            |e| *e.weight(),
            |_| T::default(),
        );

        self.is_solved = self.solution.is_some();
    }

    /// Determines which criteria is used to stop the algorithm. Check the max_size parameter and compares it to the number of nodes in the graph.     
    fn is_termination_criteria_met(&self) -> bool {
        self.graph.node_count() >= self.config.max_size
    }

    /// Returns the graph object (petgraph)
    pub fn get_graph(&self) -> &Graph<Point<T>, T, Undirected> {
        &self.graph
    }

    /// Print basic information of the graph.
    pub fn print_graph(&self) {
        pg::print_graph(self.get_graph())
    }
}

impl Default for PRMstar<f64> {
    fn default() -> Self {
        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
            phantom: PhantomData,
        });
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        PRMstar::new(start, goal, bounds, optimizer, cc)
    }
}

mod test {

    #[test]
    fn test_prm_new() {
        use crate::space::Point;

        use super::PRMstar;
        use crate::{
            boundaries::Boundaries,
            collision_checker::{CollisionChecker, NaiveCollisionChecker},
            optimizer::DefaultOptimizer,
            optimizer::Optimizer,
        };
        use std::marker::PhantomData;

        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
            phantom: PhantomData,
        });
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let planner = PRMstar::new(start, goal, bounds, optimizer, cc);

        assert!(!planner.is_solved);
    }

    #[test]
    fn test_default() {
        use super::PRMstar;

        let prmstar = PRMstar::default();
        assert!(!prmstar.is_solved);
    }

    #[test]
    fn test_prm_add_node() {
        use super::PRMstar;
        use crate::space::Point;
        use crate::{
            boundaries::Boundaries,
            collision_checker::{CollisionChecker, NaiveCollisionChecker},
            optimizer::DefaultOptimizer,
            optimizer::Optimizer,
        };
        use std::marker::PhantomData;

        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
            phantom: PhantomData,
        });
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let mut planner = PRMstar::new(start, goal, bounds, optimizer, cc);

        assert_eq!(planner.graph.node_count(), 0);
        assert_eq!(planner.tree.size(), 0);
        assert_eq!(planner.index_node_lookup.len(), 0);
        let p1: Point<f64> = Point { x: 1.8, y: 2.0 };
        planner.add_node(p1);
        assert_eq!(planner.graph.node_count(), 1);
        assert_eq!(planner.tree.size(), 1);
        assert_eq!(planner.index_node_lookup.len(), 1);
    }

    #[test]
    fn test_prm_get_solution() {
        use crate::space::Point;

        use super::PRMstar;
        use crate::planner::base_planner::Planner;
        use crate::{
            boundaries::Boundaries,
            collision_checker::{CollisionChecker, NaiveCollisionChecker},
            optimizer::DefaultOptimizer,
            optimizer::Optimizer,
        };
        use std::marker::PhantomData;

        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
            phantom: PhantomData,
        });
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let planner = PRMstar::new(start, goal, bounds, optimizer, cc);

        assert_eq!(
            crate::planner::base_planner::Planner::get_solution_cost(&planner),
            f64::MAX
        );
        assert_eq!(planner.get_solution_path(), Vec::new());
    }
}
