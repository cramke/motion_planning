use mopla::node::Node2D;
use mopla::boundaries::Boundaries;
use mopla::optimizer::{self, Optimizer};
use mopla::problem::ProblemDefinition;

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

pub fn main() {
    let start: Node2D = Node2D { x: 0f64, y: 0f64, idx: 0 };
    let goal: Node2D = Node2D { x: 3f64, y: 3f64, idx: 0 };
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(optimizer::DefaultOptimizer);
    let mut pdef= ProblemDefinition::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer);                                       
    pdef.solve();
    pdef.print_statistics();
}