use std::collections::HashMap;
use std::marker::PhantomData;

use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;
use rstar::RTree;

use crate::boundaries::Boundaries;
use crate::collision_checker::{CollisionChecker, NaiveCollisionChecker};
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

/// # Probabilisic Road Map PRM
/// It is an algorithm which is:
/// - probabilistically complete and
/// - probabilistically optimal algorithm
/// - Multi-query capable It can be used to do multi-queries.
///
/// # Source / Credits
/// Kavraki, L. E.; Svestka, P.; Latombe, J.-C.; Overmars, M. H. (1996), "Probabilistic roadmaps for path planning in high-dimensional configuration spaces", IEEE Transactions on Robotics and Automation, 12 (4): 566–580, doi:10.1109/70.508439
///
pub struct PRM<T: SpaceContinuous> {
    pub start: Point<T>,
    pub goal: Point<T>,
    pub boundaries: Boundaries<T>,
    pub graph: Graph<Point<T>, T, Undirected>,
    pub solution: Option<(T, Vec<NodeIndex>)>,
    pub is_solved: bool,
    pub collision_checker: Box<dyn CollisionChecker<T>>,
    tree: RTree<[T; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,
    pub config: Config,
}

impl<T: SpaceContinuous> Planner<T> for PRM<T> {
    fn set_start(&mut self, start: Point<T>) {
        self.start = start;
    }

    fn set_goal(&mut self, goal: Point<T>) {
        self.goal = goal;
    }

    fn set_boundaries(&mut self, boundaries: Boundaries<T>) {
        self.boundaries = boundaries;
    }

    fn set_collision_checker(&mut self, cc: Box<dyn CollisionChecker<T>>) {
        self.collision_checker = cc;
    }

    fn init(&mut self) {
        self.add_node(self.start);
        self.add_node(self.goal);
    }

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

impl<T: SpaceContinuous> PRM<T> {
    /// Standard constructor
    pub fn new(collision_checker: Box<dyn CollisionChecker<T>>) -> Self {
        PRM {
            start: Point::default(),
            goal: Point::default(),
            boundaries: Boundaries::default(),
            graph: Graph::new_undirected(),
            solution: None,
            is_solved: false,
            collision_checker,
            tree: RTree::new(),
            index_node_lookup: HashMap::new(),
            config: Config::default(),
        }
    }

    /// Adds a node to the graph, lookup for nodeindex to point.wkt, and the rtree.
    ///
    /// # Arguments
    ///
    /// - `&mut self`: a mutable reference to the current instance of the struct or class that contains the method.
    /// - `node: Point<T>`: a `Point` object representing the node to be added to the graph.
    ///
    /// # Code Analysis
    ///
    /// This method adds a new node to the graph data structure. It performs three operations:
    /// 1. Adds the `node` to the graph using the `add_node` method of the `graph` object.
    /// 2. Inserts a mapping between the WKT representation of the `node` and its index in the lookup table using the `insert` method of the `index_node_lookup` hashmap.
    /// 3. Inserts the coordinates of the `node` into the tree data structure using the `insert` method of the `tree`.
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
    /// Adds a random node to the data structure.
    ///
    /// This method generates a candidate node using the `generate_random_configuration` method of the `boundaries` object.
    /// It then checks if the candidate node collides with any existing nodes using the `is_node_colliding` method of the `collision_checker` object.
    /// If there is a collision, it continues to the next iteration of the loop.
    /// If there is no collision, it checks if the candidate node already exists in the `index_node_lookup` map.
    ///     If it does, it continues to the next iteration.
    ///     If it doesn't, it adds the candidate node to the data structure and returns it.
    ///
    fn add_random_node(&mut self) -> Point<T> {
        loop {
            let candidate: Point<T> = self.boundaries.generate_random_configuration();

            if self.collision_checker.is_node_colliding(&candidate) {
                continue;
            }

            if self
                .index_node_lookup
                .contains_key(&candidate.to_wkt().to_string())
            {
                continue;
            }

            self.add_node(candidate);
            return candidate;
        }
    }

    /// Try to connect a node to its k nearest neigbors.
    /// Connects a given node to a graph.
    ///
    /// This method connects a given node to a graph by iterating over its nearest neighbors and checking for collisions with existing edges. If there is no collision, it adds an edge between the node and the neighbor to the graph.
    /// # Arguments
    ///
    /// - `&mut self`: A mutable reference to the current instance of the struct that contains the method.
    /// - `node: Point<T>`: The node to be connected to the graph.
    /// # Outputs
    /// None. The method modifies the graph by adding edges between the node and its neighbors.
    fn connect_node_to_graph(&mut self, node: Point<T>) {
        let mut iterator = self
            .tree
            .nearest_neighbor_iter_with_distance_2(&[node.x, node.y]);

        for _ in 0..self.config.default_nearest_neighbors {
            if let Some((neighbor, distance)) = iterator.next() {
                let neighbor_point = Point {
                    x: neighbor[0],
                    y: neighbor[1],
                };

                if node == neighbor_point
                    || self
                        .collision_checker
                        .is_edge_colliding(&node, &neighbor_point)
                {
                    continue;
                }

                let a = *self
                    .index_node_lookup
                    .get(&node.to_wkt().to_string())
                    .unwrap();
                let b = *self
                    .index_node_lookup
                    .get(&neighbor_point.to_wkt().to_string())
                    .unwrap();

                self.graph.add_edge(a, b, distance);
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

impl<T: SpaceContinuous + 'static> Default for PRM<T> {
    fn default() -> Self {
        let collision_checker: Box<dyn CollisionChecker<T>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        PRM::new(collision_checker)
    }
}

#[cfg(test)]
mod test {
    use super::PRM;
    use crate::boundaries::Boundaries;
    use crate::collision_checker::{CollisionChecker, NaiveCollisionChecker};
    use crate::planner::base_planner::Planner;
    use crate::space::Point;
    use std::marker::PhantomData;

    // Test that the function 'test_default_f64' returns a PRM instance with the 'is_solved' field set to false.
    #[test]
    fn test_default_f64_returns_false() {
        let prm: PRM<f64> = PRM::default();
        assert!(!prm.is_solved);
    }

    // Test that the function 'test_default_f32' returns a PRM instance with the 'is_solved' field set to false.
    #[test]
    fn test_default_f32_returns_false() {
        let prm: PRM<f32> = PRM::default();
        assert!(!prm.is_solved);
    }

    #[test]
    fn test_prm_add_node() {
        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let mut planner: PRM<f64> = PRM::default();
        planner.set_start(start);
        planner.set_goal(goal);
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        planner.set_boundaries(bounds);

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
    fn test_setup_from_problem() {
        let mut prm: PRM<f64> = PRM::default();
        prm.set_start(Point { x: 8f64, y: 9f64 });
        prm.set_goal(Point { x: 10f64, y: 11f64 });

        assert_eq!(prm.start, Point { x: 8f64, y: 9f64 });
        assert_eq!(prm.goal, Point { x: 10f64, y: 11f64 });
    }

    #[test]
    fn test_setup_boundaries() {
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let mut prm = PRM::new(cc);
        let bounds: Boundaries<f64> = Boundaries::new(1f64, 2f64, 3f64, 4f64);
        assert_ne!(prm.boundaries.get_x_lower(), bounds.get_x_lower());
        assert_ne!(prm.boundaries.get_x_upper(), bounds.get_x_upper());
        assert_ne!(prm.boundaries.get_y_lower(), bounds.get_y_lower());
        assert_ne!(prm.boundaries.get_y_upper(), bounds.get_y_upper());
        prm.set_boundaries(bounds.clone());
        assert_eq!(prm.boundaries.get_x_lower(), bounds.get_x_lower());
        assert_eq!(prm.boundaries.get_x_upper(), bounds.get_x_upper());
        assert_eq!(prm.boundaries.get_y_lower(), bounds.get_y_lower());
        assert_eq!(prm.boundaries.get_y_upper(), bounds.get_y_upper());
    }

    // Test if adding a node to the planner increments the node count, tree size, and index node lookup by 1.
    #[test]
    fn test_prm_add_node_increment() {
        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let mut planner: PRM<f64> = PRM::default();
        planner.set_start(start);
        planner.set_goal(goal);
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        planner.set_boundaries(bounds);

        assert_eq!(planner.graph.node_count(), 0);
        assert_eq!(planner.tree.size(), 0);
        assert_eq!(planner.index_node_lookup.len(), 0);
        let p1: Point<f64> = Point { x: 1.8, y: 2.0 };
        planner.add_node(p1);
        assert_eq!(planner.graph.node_count(), 1);
        assert_eq!(planner.tree.size(), 1);
        assert_eq!(planner.index_node_lookup.len(), 1);
    }

    // Test if adding a node with the same coordinates as the start point does not change the node count, tree size, and index node lookup.
    #[test]
    fn test_prm_add_node_same_coordinates_as_start() {
        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let mut planner: PRM<f64> = PRM::default();
        planner.set_start(start);
        planner.set_goal(goal);
        planner.init();
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        planner.set_boundaries(bounds);

        assert_eq!(planner.graph.node_count(), 2);
        assert_eq!(planner.tree.size(), 2);
        assert_eq!(planner.index_node_lookup.len(), 2);
        let p1: Point<f64> = Point { x: 0f64, y: 0f64 };
        planner.add_node(p1);
        assert_eq!(planner.graph.node_count(), 2);
        assert_eq!(planner.tree.size(), 2);
        assert_eq!(planner.index_node_lookup.len(), 2);
    }

    // Test if adding a node with the same coordinates as the goal point keeps the node count, tree size, and index node lookup as 0.
    #[test]
    fn test_prm_add_node_same_coordinates_as_goal() {
        let start: Point<f64> = Point { x: 0f64, y: 0f64 };
        let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
        let mut planner: PRM<f64> = PRM::default();
        planner.set_start(start);
        planner.set_goal(goal);
        planner.init();
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        planner.set_boundaries(bounds);

        assert_eq!(planner.graph.node_count(), 2);
        assert_eq!(planner.tree.size(), 2);
        assert_eq!(planner.index_node_lookup.len(), 2);
        let p1: Point<f64> = Point { x: 3f64, y: 3f64 };
        planner.add_node(p1);
        assert_eq!(planner.graph.node_count(), 2);
        assert_eq!(planner.tree.size(), 2);
        assert_eq!(planner.index_node_lookup.len(), 2);
    }
}
