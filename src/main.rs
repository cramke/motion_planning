use rand::Rng;
use petgraph::graph::Graph;
use petgraph::dot::{Dot, Config};

/** Parameters
 *  Are global because are required during compilation
 */

const NUMBER_OF_NODES_TO_ADD: usize = 5;

#[derive(Clone)]
struct PlanningSetup {
    start: Node,
    goal: Node,
    boundaries: Boundaries,
    graph: Graph<[f64;2], f64>,
    is_node_in_collision: fn(&Node) -> bool,
    is_edge_in_collision: fn() -> bool,
}

#[derive(Debug, Copy, Clone)]
struct Boundaries {
    x_lower: f64,
    x_upper: f64,
    y_lower: f64,
    y_upper: f64,
}

#[derive(Debug, Copy, Clone)]
struct Node {
    x: f64,
    y: f64,
}

impl PlanningSetup {

    fn init(&mut self) {
        let start: Node = Node { x: self.start.x, y: self.start.y};
        let goal: Node = Node { x: self.goal.x, y: self.goal.y};

        if !self.is_in_boundaries() {
            panic!("Start or goal not inside boundaries.");
        }

        if is_collision(&start) {
            panic!("Start is in collision.");
        }

        if is_collision(&goal) {
            panic!("Goal is in collision.");
        }

        self.insert_node_in_graph(&start);
        self.insert_node_in_graph(&goal);
        println!("Setup is ready for planning")

    }

    fn run(&mut self) {
        loop {
            let added_nodes: Vec<Node> = self.add_batch_of_random_nodes();
            println!("{}", added_nodes.len());
            self.connect_added_nodes_to_graph(added_nodes);

            if self.is_problem_solved() {
                println!("Solved");
            }

            if self.is_termination_criteria_met() {
                println!("Termination Criteria met");
            }

            break;
        }
    }

    fn add_batch_of_random_nodes(&mut self) -> Vec<Node> {
        let mut counter_added_nodes: usize = 0;
        let mut list_of_added_nodes: Vec<Node> = Vec::new();
    
        while counter_added_nodes < NUMBER_OF_NODES_TO_ADD {
            let node: Node = self.find_permissable_node();
            self.insert_node_in_graph(&node);
            list_of_added_nodes.push(node);
            counter_added_nodes += 1;
        }
        return list_of_added_nodes;
    }

    fn find_permissable_node(&self) -> Node {
        loop {
            let node: Node = self.generate_random_configuration();
            if is_collision(&node) {
                continue;
            }
    
            if self.is_node_already_in_graph() {
                continue;
            }
            return node;
        }
    }

    fn insert_node_in_graph(&mut self, node: &Node) {
        self.graph.add_node([node.x, node.y]);
    }

    fn is_in_boundaries(&self) -> bool {
        return true;
    }

    fn connect_added_nodes_to_graph(&self, added_nodes: Vec<Node>) {
        for node in added_nodes {
            let nearest_neighbors: Vec<Node> = self.get_n_nearest_neighbours(node);
            for neighbor in nearest_neighbors {
                if is_edge_in_collision() {
                    continue;
                }
    
                if self.is_edge_already_in_graph() {
                    continue;
                }
    
                self.insert_edge_in_graph();
            }
        }
    }

    fn generate_random_configuration(&self) -> Node {
        let mut rng = rand::thread_rng();
        let x: f64 = rng.gen_range(self.boundaries.x_lower..self.boundaries.x_upper);
        let y: f64 = rng.gen_range(self.boundaries.y_lower..self.boundaries.y_upper);
        let node: Node = Node { x: x, y: y };
        return node;
    }

    fn insert_edge_in_graph(&self) {
    }

    fn is_problem_solved(&self) -> bool {
        return false;
    }
    
    fn is_termination_criteria_met(&self) -> bool {
        return true;
    }
    
    fn is_edge_already_in_graph(&self) -> bool {
        return false;
    }
    
    fn is_node_already_in_graph(&self) -> bool {
        return false;
    }
    
    fn get_n_nearest_neighbours(&self, node: Node) -> Vec<Node> {
        let nearest_neighbors: Vec<Node> = Vec::new();
        return nearest_neighbors;
    }
}

/** -------------------------------------------------------------------
 *  Setup and/or configure for the specific planning problem.
 */

fn is_collision(node: &Node) -> bool {
    if node.x > 1.0 && node.x < 2.0 {
        return true;
    }
    if node.y > 1.0 && node.y < 2.0 {
        return true;
    }

    return false;
}

fn is_edge_in_collision() -> bool {
    return false;
}

fn main() {
    println!("Hello, world!");
    let start: Node = Node { x: 0f64, y: 0f64 };
    let goal: Node = Node { x: 3f64, y: 3f64 };
    let bounds: Boundaries = Boundaries { x_lower: 0f64, x_upper: 3f64, y_lower: 0f64, y_upper: 3f64 };
    let mut setup: PlanningSetup = PlanningSetup {  start: start, 
                                                goal: goal, 
                                                boundaries: bounds,
                                                graph: Graph::new(),
                                                is_node_in_collision: is_collision,
                                                is_edge_in_collision: is_edge_in_collision,};
                                                
    setup.init();
    setup.run();
   
    println!("{:?}", Dot::with_config(&setup.graph, &[Config::EdgeNoLabel]));
}

#[cfg(test)]
mod test {
    use crate::Node;

    #[test]
    fn test_dummy() {
        let tree: f64 = 3f64;
        assert_eq!(tree, 3.0);
    }

    #[test]
    fn test_node() {
        let node: Node = Node {x: 0f64, y: 0f64};
        assert_eq!(node.x, 0f64);
        assert_eq!(node.y, 0f64);
    }
}