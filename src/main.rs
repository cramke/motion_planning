use rand::Rng;
use petgraph::graph::{Graph, NodeIndex, NodeIndices};
use petgraph::dot::{Dot, Config};

/** Parameters
 *  Are global because are required during compilation
 */

const NUMBER_OF_NODES_TO_ADD: usize = 5;

#[derive(Clone)]
struct PlanningSetup {
    start: Node2D,
    goal: Node2D,
    boundaries: Boundaries,
    graph: Graph<[f64;2], f64>,
    is_node_in_collision: fn(&Node2D) -> bool,
    is_edge_in_collision: fn() -> bool,
    get_edge_weight: fn() -> f64,
}

#[derive(Debug, Copy, Clone)]
struct Boundaries {
    x_lower: f64,
    x_upper: f64,
    y_lower: f64,
    y_upper: f64,
}

#[derive(Debug, Copy, Clone)]
struct Node2D {
    x: f64,
    y: f64,
    idx: usize,
}

impl Node2D {
    fn new(x: f64, y: f64) -> Self {
        let idx = 0;
        return Node2D { x: x, y: y, idx: idx }
    }
}

impl PlanningSetup {

    fn init(&mut self) {
        let mut start: Node2D = Node2D { x: self.start.x, y: self.start.y, idx: 0};
        let mut goal: Node2D = Node2D { x: self.goal.x, y: self.goal.y, idx: 0};

        if !self.is_in_boundaries() {
            panic!("Start or goal not inside boundaries.");
        }

        if is_collision(&start) {
            panic!("Start is in collision.");
        }

        if is_collision(&goal) {
            panic!("Goal is in collision.");
        }

        self.insert_node_in_graph(&mut start);
        self.insert_node_in_graph(&mut goal);
        println!("Setup is ready for planning")

    }

    fn run(&mut self) {
        loop {
            let added_nodes: Vec<Node2D> = self.add_batch_of_random_nodes();
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

    fn add_batch_of_random_nodes(&mut self) -> Vec<Node2D> {
        let mut list_of_added_nodes: Vec<Node2D> = Vec::new();
    
        while list_of_added_nodes.len() < NUMBER_OF_NODES_TO_ADD {
            let mut node: Node2D = self.find_permissable_node();
            self.insert_node_in_graph(&mut node);
            list_of_added_nodes.push(node);
        }
        return list_of_added_nodes;
    }

    fn find_permissable_node(&self) -> Node2D {
        loop {
            let node: Node2D = self.generate_random_configuration();
            if is_collision(&node) {
                continue;
            }
    
            if self.is_node_already_in_graph() {
                continue;
            }
            return node;
        }
    }

    fn insert_node_in_graph(&mut self, node: &mut Node2D) {
        let index: usize = self.graph.add_node([node.x, node.y]).index();
        node.idx = index;
    }

    fn is_in_boundaries(&self) -> bool {
        return true;
    }

    fn connect_added_nodes_to_graph(&mut self, added_nodes: Vec<Node2D>) {
        for node in added_nodes {
            let nearest_neighbors: NodeIndices = self.get_n_nearest_neighbours(node);
            for neighbor in nearest_neighbors {
                if is_edge_in_collision() {
                    continue;
                }
    
                if self.is_edge_already_in_graph() {
                    continue;
                }
    
                self.insert_edge_in_graph(&node, neighbor);
            }
        }
    }

    fn generate_random_configuration(&self) -> Node2D {
        let mut rng = rand::thread_rng();
        let x: f64 = rng.gen_range(self.boundaries.x_lower..self.boundaries.x_upper);
        let y: f64 = rng.gen_range(self.boundaries.y_lower..self.boundaries.y_upper);
        let node: Node2D = Node2D { x: x, y: y, idx: 0};
        return node;
    }

    fn insert_edge_in_graph(&mut self, begin: &Node2D, end: NodeIndex) {
        let weight: f64 = get_edge_weight();
        let a: NodeIndex<u32> = NodeIndex::new(begin.idx);
        if a == end { // do not insert edge from a node to itself
            return;
        }
        self.graph.add_edge(a, end, weight);

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
    
    fn get_n_nearest_neighbours(&self, node: Node2D) -> NodeIndices {
        let node_iterator = self.graph.node_indices();
        return node_iterator;
    }
}

/** -------------------------------------------------------------------
 *  Setup and/or configure for the specific planning problem.
 */

fn is_collision(node: &Node2D) -> bool {
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

fn get_edge_weight() -> f64 {
    let mut rng = rand::thread_rng();
    let cost: f64 = rng.gen_range(0f64..100f64);
    return cost;
}

fn main() {
    println!("Hello, world!");
    let start: Node2D = Node2D { x: 0f64, y: 0f64, idx: 0 };
    let goal: Node2D = Node2D { x: 3f64, y: 3f64, idx: 0 };
    let bounds: Boundaries = Boundaries { x_lower: 0f64, x_upper: 3f64, y_lower: 0f64, y_upper: 3f64 };
    let mut setup: PlanningSetup = PlanningSetup {  start: start, 
                                                goal: goal, 
                                                boundaries: bounds,
                                                graph: Graph::new(),
                                                is_node_in_collision: is_collision,
                                                is_edge_in_collision: is_edge_in_collision,
                                                get_edge_weight: get_edge_weight,};
                                                
    setup.init();
    setup.run();
   
    println!("{:?}", Dot::with_config(&setup.graph, &[Config::EdgeNoLabel]));
}