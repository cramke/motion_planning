use rand::Rng;
use petgraph::dot::{Dot, Config};

use prm::node::Node2D;
use prm::boundaries::Boundaries;
use prm::planning_setup::PlanningSetup;


/** Parameters
 *  Are global because are required during compilation
 */
const NUMBER_OF_NODES_TO_ADD: usize = 5;

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
    let mut setup = PlanningSetup::new( start, goal, bounds, is_collision, is_edge_in_collision, get_edge_weight,);                                       
    setup.init();
    setup.run();
   
    println!("{:?}", Dot::with_config(setup.get_graph(), &[Config::EdgeNoLabel]));
}