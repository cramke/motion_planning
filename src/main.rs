/** Parameters
 *  Are global because are required during compilation
 */

const NUMBER_OF_NODES_TO_ADD: usize = 10;
const CONNECT_WITH_N_NEAREST_NEIGHBORS: usize = 5;

struct PlanningSetup {
    start: [f64; 2],
    goal: [f64; 2],
}

struct Boundaries {
    x_lower: f64,
    x_upper: f64,
    y_lower: f64,
    y_upper: f64, 
}

struct Node {
    x: f64,
    y: f64,
}

fn run() {
    let batch_size: i8 = 20;
    loop {
        let added_nodes: [Node; NUMBER_OF_NODES_TO_ADD] = add_batch_of_random_nodes();
        connect_added_nodes_to_graph(added_nodes);

        if _is_problem_solved() {
            break;
        }

        if _is_termination_criteria_met() {
            break;
        }
    }
    
}

fn add_batch_of_random_nodes() -> [Node; NUMBER_OF_NODES_TO_ADD] {
    let mut counter_added_nodes: usize = 0;
    let mut list_of_added_nodes: [Node; NUMBER_OF_NODES_TO_ADD];

    while counter_added_nodes < NUMBER_OF_NODES_TO_ADD {
        let node: Node = find_permissable_node();
        list_of_added_nodes[counter_added_nodes] = node;
        insert_node_in_graph(node);
        counter_added_nodes += 1;
    }
    return list_of_added_nodes;

}

fn connect_added_nodes_to_graph(added_nodes: [Node; NUMBER_OF_NODES_TO_ADD]) {
    for node in added_nodes {
        let nearest_neighbors: [Node; CONNECT_WITH_N_NEAREST_NEIGHBORS] = get_n_nearest_neighbours(node);
        for neighbor in nearest_neighbors {
            if is_edge_between_nodes_is_collision(node, neighbor) {
                continue;
            }

            if is_edge_already_in_graph(node, neighbor) {
                continue;
            }

            insert_edge_in_graph(node, neighbor);
        }

    }
}

fn generate_random_configuration() -> Node {
    let node: Node;
    return node;
}

fn find_permissable_node() -> Node {
    loop {
        let mut node: Node = generate_random_configuration();
        if is_collision(node) {
            continue;
        }

        if is_node_already_in_graph(node) {
            continue;
        }
        return node;
    }
}

fn is_collision(node: Node) -> bool {
    return false;
}

fn is_edge_between_nodes_is_collision(node: Node, neighbor: Node) -> bool {
    return false;
}

fn insert_node_in_graph(node: Node) {

}

fn _is_problem_solved() -> bool {
    return false;
}

fn _is_termination_criteria_met() -> bool {
    return true;
}

fn is_edge_already_in_graph(node: Node, neighbor: Node) -> bool {
    return false;
}

fn is_node_already_in_graph(node: Node) -> bool {
    return false;
}

fn insert_edge_in_graph(node: Node, neighbor: Node) {

}

fn get_n_nearest_neighbours(node: Node) -> [Node; CONNECT_WITH_N_NEAREST_NEIGHBORS] {
    let nearest_neighbors: [Node; CONNECT_WITH_N_NEAREST_NEIGHBORS];
    return nearest_neighbors;
}

fn main() {
    println!("Hello, world!");
    let bounds: Boundaries = Boundaries { x_lower: 0f64, x_upper: 3f64, y_lower: 0f64, y_upper: 3f64 };
    let setup: PlanningSetup = PlanningSetup { start: [0f64, 0f64], goal: [3f64, 3f64] };
}
