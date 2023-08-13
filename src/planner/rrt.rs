use core::panic;
use std::collections::HashMap;

use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;
use rstar::RTree;

use crate::boundaries::Boundaries;
use crate::collision_checker::CollisionChecker;
use crate::core::Metric2D;
use crate::planner::base_planner::Planner;
use crate::space::Point;

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

/// # Rapidly-Exploring Random Trees - RRT
/// It is an algorithm which is:
/// - probabilistically complete
/// - probabilistically optimal algorithm
/// - Single query
///
/// # Source / Credits
/// LaValle, S. M. (), "Rapidly-Exploring Random Trees: A New Tool for Path"
/// [Link](https://www.cs.csustan.edu/~xliang/Courses/CS4710-21S/Papers/06%20RRT.pdf)
///
/// # Example
pub struct RRT<T: Metric2D> {
    solution: Option<(T, Vec<NodeIndex>)>,
    pub is_solved: bool,
    start: Point<T>,
    goal: Point<T>,
    pub graph: Graph<Point<T>, T, Undirected>,
    tree: RTree<[T; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,
    boundaries: Boundaries<T>,
    collision_checker: Box<dyn CollisionChecker<T>>,
    pub config: Config,
}

impl<T: Metric2D> RRT<T> {
    /// Constructor
    pub fn new(
        mut boundaries: Boundaries<T>,
        collision_checker: Box<dyn CollisionChecker<T>>,
    ) -> Self {
        RRT {
            config: Config::default(),
            solution: None,
            is_solved: false,
            start: boundaries.generate_random_configuration(),
            goal: boundaries.generate_random_configuration(),
            graph: Graph::new_undirected(),
            tree: RTree::new(),
            index_node_lookup: HashMap::new(),

            boundaries,
            collision_checker,
        }
    }

    /// Adds a node to the graph, lookup for nodeindex to point.wkt, and the rtree.
    fn add_node(&mut self, node: Point<T>) {
        let index = self.graph.add_node(node);
        self.index_node_lookup
            .insert(node.to_wkt().to_string(), index);
        self.tree.insert([node.x, node.y]);
    }

    /// Adds an edge to the graph and
    fn add_edge(&mut self, begin: Point<T>, end: Point<T>) {
        let weight: T = begin.euclidean_distance(&end);
        let a: NodeIndex = *self
            .index_node_lookup
            .get(&begin.to_wkt().to_string())
            .unwrap();
        let b: NodeIndex = *self
            .index_node_lookup
            .get(&end.to_wkt().to_string())
            .unwrap();
        self.graph.add_edge(a, b, weight);
    }

    /// Applies A* and checks if a solution exists
    fn check_solution(&mut self) {
        for coords in self.tree.nearest_neighbor_iter(&[self.goal.x, self.goal.y]) {
            let neighbor: Point<T> = Point {
                x: coords[0],
                y: coords[1],
            };
            if self
                .collision_checker
                .is_edge_colliding(&neighbor, &self.goal)
            {
                continue;
            } else {
                self.add_edge(neighbor, self.goal);
                break;
            }
        }

        for coords in self
            .tree
            .nearest_neighbor_iter(&[self.start.x, self.start.y])
        {
            let neighbor: Point<T> = Point {
                x: coords[0],
                y: coords[1],
            };
            if self
                .collision_checker
                .is_edge_colliding(&neighbor, &self.goal)
            {
                continue;
            } else {
                self.add_edge(neighbor, self.goal);
                break;
            }
        }

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

    /// Returns Option to the nearest neighbor from the given point
    /// - None: No neighbor
    /// - Some(Point): Contains nearest neighbor
    fn get_nearest_neighbor(&self, node: Point<T>) -> Option<Point<T>> {
        let neighbor: Option<&[T; 2]> = self.tree.nearest_neighbor(&[node.x, node.y]);
        neighbor.map(|coords| Point {
            x: coords[0],
            y: coords[1],
        })
    }
}

impl<T: Metric2D> Planner<T> for RRT<T> {
    /// Run once before running the algorithm
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

    /// Builds the graph until a termination criteria is met.
    fn run(&mut self) {
        loop {
            let random_node: Point<T> = self.boundaries.generate_random_configuration();

            let nearest_neighbour = match self.get_nearest_neighbor(random_node) {
                Some(point) => point,
                None => continue,
            };

            if self.collision_checker.is_node_colliding(&random_node) {
                continue;
            }
            if self
                .collision_checker
                .is_edge_colliding(&random_node, &nearest_neighbour)
            {
                continue;
            }

            self.add_node(random_node);
            self.add_edge(random_node, nearest_neighbour);

            self.check_solution();

            if self.is_termination_criteria_met() {
                println!("Termination Criteria met");
                break;
            }
        }
    }

    /// Getter method to check if a solution was found.
    /// - true: There is a solution
    /// - false: No solution was found yet. This could mean the algorithm needs to run longer.
    fn is_solved(&self) -> bool {
        self.is_solved
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

    /// Returns empty Vector if no soulution was found.
    fn get_solution_path(&self) -> Vec<Point<T>> {
        match &self.solution {
            None => Vec::new(),
            Some((_, path_idx)) => path_idx
                .iter()
                .map(|idx| *self.graph.node_weight(*idx).unwrap())
                .collect(),
        }
    }

    /// Print overview of the planner.
    fn print_statistics(&self, _path: &str) {
        println!("Was a solution found? {}", self.is_solved);
        let nodes: usize = self.graph.node_count();
        println!("Graph contains {nodes} nodes");

        let edges: usize = self.graph.edge_count();
        println!("Graph contains {edges} edges");
    }
}

mod test {

    #[test]
    fn test_new() {
        use crate::planner::rrt::RRT;
        use crate::{
            boundaries::Boundaries,
            collision_checker::{CollisionChecker, NaiveCollisionChecker},
            planner::rrt::Config,
        };
        use std::marker::PhantomData;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let rrt = RRT::new(bounds, cc);
        assert_eq!(rrt.config.max_size, Config::default().max_size);
        assert!(!rrt.is_solved)
    }

    #[test]
    fn test_run() {
        use crate::planner::base_planner::Planner;
        use crate::planner::rrt::RRT;
        use crate::{
            boundaries::Boundaries,
            collision_checker::{CollisionChecker, NaiveCollisionChecker},
        };
        use std::marker::PhantomData;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let mut rrt = RRT::new(bounds, cc);
        rrt.init();
        rrt.run();
        rrt.print_statistics("a");
        assert!(rrt.is_solved);
    }
}
