use std::collections::HashMap;
use std::marker::PhantomData;

use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;
use rstar::RTree;

use crate::boundaries::Boundaries;
use crate::collision_checker::{CollisionChecker, NaiveCollisionChecker};
use crate::optimizer::{DefaultOptimizer, Optimizer};
use crate::planner::base_planner::Planner;
use crate::planner::graph_utils as pg;
use crate::space::Point;
use crate::types::SpaceContinuous;

/// # Holds configuration parameters for PRM*
/// It does configure:
/// - default_nearest_neighbors: Limits the number of nodes that are used to calculate motionCost to the n closest ones
/// - max_size: Limits the number of Nodes in the graph before termination of the algrithm
pub struct Config {
    pub default_nearest_neighbors: u8,
    pub max_size: usize,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            default_nearest_neighbors: 10u8,
            max_size: 32usize,
        }
    }
}

/// # Probabilisic Road Map PRM* for optimal planning
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
pub struct PRMstar<T: SpaceContinuous> {
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

impl<T: SpaceContinuous> Planner<T> for PRMstar<T> {
    /// Setter for start
    fn set_start(&mut self, start: Point<T>) {
        self.start = start;
    }

    /// Setter for goal
    fn set_goal(&mut self, goal: Point<T>) {
        self.goal = goal;
    }

    /// Setter for boundaries
    fn set_boundaries(&mut self, boundaries: Boundaries<T>) {
        self.boundaries = boundaries;
    }

    /// Setter for Collision Checker
    fn set_collision_checker(&mut self, cc: Box<dyn CollisionChecker<T>>) {
        self.collision_checker = cc;
    }

    /// Initializes the problem by adding the start and goal fields into the solution graph
    fn init(&mut self) {
        self.add_node(self.start);
        self.add_node(self.goal);
    }

    /// Use the current configuration to solve the problem
    fn solve(&mut self) {
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

    /// Returns the solution cost.
    /// - f64::MAX: No solution was found
    /// - cost: The cost of the solution. Implies that a solution was found.
    fn get_solution_cost(&self) -> T {
        match &self.solution {
            None => T::MAX,
            Some((cost, _)) => *cost,
        }
    }
}

impl<T: SpaceContinuous> PRMstar<T> {
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
        if self.collision_checker.is_node_colliding(&node) {
            return;
        }

        if self
            .index_node_lookup
            .contains_key(&node.to_wkt().to_string())
        {
            return;
        }

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

            if self
                .index_node_lookup
                .contains_key(&candidate.to_wkt().to_string())
            {
                continue;
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
            if let Some(neighbor) = iterator.next() {
                let neighbor_point = Point {
                    x: neighbor[0],
                    y: neighbor[1],
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

                let weight = self.optimizer.get_edge_weight(node, neighbor_point).2;
                let a = *self
                    .index_node_lookup
                    .get(&node.to_wkt().to_string())
                    .unwrap();
                let b = *self
                    .index_node_lookup
                    .get(&neighbor_point.to_wkt().to_string())
                    .unwrap();
                self.graph.add_edge(a, b, weight);
            }
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

#[cfg(test)]
mod test {
    use crate::{planner::base_planner::Planner, space::Point};

    use super::PRMstar;
    use crate::{
        boundaries::Boundaries,
        collision_checker::{CollisionChecker, NaiveCollisionChecker},
        optimizer::DefaultOptimizer,
        optimizer::Optimizer,
    };
    use std::marker::PhantomData;

    #[test]
    fn test_prm_new() {
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

    // Test that a new PRMstar planner is created with start and goal points outside of the boundaries
    #[test]
    fn test_prm_new_outside_boundaries() {
        let start: Point<f64> = Point { x: -1f64, y: -1f64 };
        let goal: Point<f64> = Point { x: 4f64, y: 4f64 };
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

    // Test that a new PRMstar planner is created with the specified custom configuration
    #[test]
    fn test_prm_new_custom_configuration() {
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
        // Add assertions for the custom configuration
        assert_eq!(planner.config.default_nearest_neighbors, 10u8);
        assert_eq!(planner.config.max_size, 32usize);
    }

    // Test that adding a node to the planner with a point that is already in the graph does not add a new node
    #[test]
    fn test_prm_add_existing_node() {
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

        // Add the same node again
        planner.add_node(p1);

        // Node count, tree size, and index node lookup should remain the same
        assert_eq!(planner.graph.node_count(), 1);
        assert_eq!(planner.tree.size(), 1);
        assert_eq!(planner.index_node_lookup.len(), 1);
    }

    // Test that the 'set_start' and 'set_goal' methods properly set the start and goal points of the PRMstar planner
    #[test]
    fn test_prm_set_start_and_goal() {
        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
            phantom: PhantomData,
        });
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let mut planner: PRMstar<f64> = PRMstar::new(start, goal, bounds, optimizer, cc);

        let new_start: Point<f64> = Point { x: 1f64, y: 1f64 };
        let new_goal: Point<f64> = Point { x: 2f64, y: 2f64 };

        planner.set_start(new_start);
        planner.set_goal(new_goal);

        assert_eq!(planner.start, new_start);
        assert_eq!(planner.goal, new_goal);
        planner.init();
        assert_eq!(planner.graph.node_count(), 2);
        assert_eq!(planner.tree.size(), 2);
        assert_eq!(planner.index_node_lookup.len(), 2);
    }
}
