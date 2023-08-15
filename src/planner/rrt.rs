use std::collections::HashMap;

use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;
use rstar::RTree;

use crate::boundaries::Boundaries;
use crate::collision_checker::{CollisionChecker, NaiveCollisionChecker};
use crate::space::Point;
use crate::types::SpaceContinuous;

use super::base_planner::Planner;

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
pub struct RRT<T: SpaceContinuous> {
    pub solution: Option<(T, Vec<NodeIndex>)>,
    pub is_solved: bool,
    pub start: Point<T>,
    pub goal: Point<T>,
    pub graph: Graph<Point<T>, T, Undirected>,
    tree: RTree<[T; 2]>,
    index_node_lookup: HashMap<String, NodeIndex>,
    pub boundaries: Boundaries<T>,
    pub collision_checker: Box<dyn CollisionChecker<T>>,
    pub config: Config,
}

impl<T: SpaceContinuous> Planner<T> for RRT<T> {
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

impl<T: SpaceContinuous + 'static> Default for RRT<T> {
    fn default() -> Self {
        RRT {
            config: Config::default(),
            solution: None,
            is_solved: false,
            start: Point::default(),
            goal: Point::default(),
            graph: Graph::new_undirected(),
            tree: RTree::new(),
            index_node_lookup: HashMap::new(),
            boundaries: Boundaries::default(),
            collision_checker: NaiveCollisionChecker::new_box(),
        }
    }
}

impl<T: SpaceContinuous> RRT<T> {
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

#[cfg(test)]
mod test {
    use crate::planner::rrt::Config;
    use crate::planner::rrt::RRT;
    use crate::{
        boundaries::Boundaries,
        collision_checker::{CollisionChecker, NaiveCollisionChecker},
    };
    use std::marker::PhantomData;

    #[test]
    fn test_new() {
        let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });
        let rrt = RRT::new(bounds, cc);
        assert_eq!(rrt.config.max_size, Config::default().max_size);
        assert!(!rrt.is_solved)
    }
}
