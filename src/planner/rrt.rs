use core::panic;
use std::collections::HashMap;

use geo::EuclideanDistance;
use petgraph::Undirected;
use petgraph::algo::{astar};
use petgraph::graph::{Graph, NodeIndex};
use rstar::RTree;
use wkt::ToWkt;

use crate::space::Point;
use crate::boundaries::Boundaries;
use crate::collision_checker::CollisionChecker;
use crate::planner::base_planner::Planner;
use crate::problem::Parameter;

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
pub struct RRT {
    parameters: Parameter,
    solution: Option<(f64, Vec<NodeIndex>)>,
    pub is_solved: bool,

    start: Point<f64>,
    goal: Point<f64>,

    pub graph: Graph<Point<f64>, f64, Undirected>,
    tree: RTree<[f64; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,

    boundaries: Boundaries<f64>,
    collision_checker: Box<dyn CollisionChecker>,
}

impl RRT {
    /// Constructor
    pub fn new(boundaries: Boundaries<f64>, collision_checker: Box<dyn CollisionChecker>) -> Self {
        RRT {
            parameters: Parameter::default(),
            solution: None,
            is_solved: false,
            start: Point{x:0.0, y:0.0},
            goal: Point{x:3.0, y:3.0},
            graph: Graph::new_undirected(),
            tree: RTree::new(),
            index_node_lookup: HashMap::new(),

            boundaries,
            collision_checker,
        }
    }

    /// Adds a node to the graph, lookup for nodeindex to point.wkt, and the rtree.
    fn add_node(&mut self, node: Point<f64>) {
        let index = self.graph.add_node(node);
        self.index_node_lookup.insert(node.to_wkt().to_string(), index);
        self.tree.insert([node.x, node.y]);
    }

    /// Adds an edge to the graph and 
    fn add_edge(&mut self, begin: Point<f64>, end: Point<f64>) {
        let weight: f64 = begin.euclidean_distance(&end);
        if weight == 0.0 {return;}
        let a: NodeIndex = *self.index_node_lookup.get(&begin.to_wkt().to_string()).unwrap();
        let b: NodeIndex = *self.index_node_lookup.get(&end.to_wkt().to_string()).unwrap();
        self.graph.add_edge(a, b, weight);
    }

    /// Applies A* and checks if a solution exists
    fn check_solution(&mut self) {
        for coords in self.tree.nearest_neighbor_iter(&[self.goal.x, self.goal.y]) {
            let neighbor: Point<f64> = Point{x:coords[0], y:coords[1]};
            if self.collision_checker.is_edge_colliding(&neighbor, &self.goal) {
                continue
            } else {
                self.add_edge(neighbor, self.goal);
                break;
            }
        }

        for coords in self.tree.nearest_neighbor_iter(&[self.start.x, self.start.y]) {
            let neighbor: Point<f64> = Point{x:coords[0], y:coords[1]};
            if self.collision_checker.is_edge_colliding(&neighbor, &self.goal) {
                continue
            } else {
                self.add_edge(neighbor, self.goal);
                break;
            }
        }

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
        self.graph.node_count() >= self.parameters.max_size
    }

    /// Returns Option to the nearest neighbor from the given point
    /// - None: No neighbor
    /// - Some(Point): Contains nearest neighbor
    fn get_nearest_neighbor(&self, node: Point<f64>) -> Option<Point<f64>> {
        let neighbor = self.tree.nearest_neighbor(&[node.x, node.y]);
        neighbor.map(|coords| Point{x:coords[0], y:coords[1]})
    }
}

impl Planner for RRT {
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
            let random_node: Point<f64> = self.boundaries.generate_random_configuration();

            let nearest_neighbour = match self.get_nearest_neighbor(random_node) {
                Some(point) => point,
                None => continue,
            };

            if self.collision_checker.is_node_colliding(&random_node) {continue}
            if self.collision_checker.is_edge_colliding(&random_node, &nearest_neighbour) {continue}

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
    fn get_solution_cost(&self) -> f64 {
        match &self.solution {
            None => f64::MAX,
            Some((cost, _)) => *cost,
        }
    }

    /// Returns empty Vector if no soulution was found. 
    fn get_solution_path(&self) -> Vec<Point<f64>> {
        match &self.solution {
            None => Vec::new(),
            Some((_, path_idx)) => path_idx
            .iter()
            .map(|idx| *self.graph.node_weight(*idx).unwrap())
            .collect(),
        }
    }

    /// Print overview of the planner. 
    fn print_statistics(&self, _path:&str) {
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
        use crate::{problem::Parameter, planner::rrt::RRT};
        use crate::{boundaries::Boundaries, collision_checker::{NaiveCollisionChecker, CollisionChecker}};

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
        let rrt = RRT::new(bounds, cc);
        assert_eq!(rrt.parameters.max_size, Parameter::default().max_size);
        assert!(!rrt.is_solved)        
    }

    #[test]
    fn test_run() {
        use crate::planner::rrt::RRT;
        use crate::{boundaries::Boundaries, collision_checker::{NaiveCollisionChecker, CollisionChecker}};
        use crate::planner::base_planner::Planner;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
        let mut rrt = RRT::new(bounds, cc);
        rrt.init();
        rrt.run();
        rrt.print_statistics("a");
        assert!(rrt.is_solved);
    }
}