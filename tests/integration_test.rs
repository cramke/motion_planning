use prm::{self, problem::ProblemDefinition, optimizer::{self, DefaultOptimizer}};

#[test]
fn test_prm() {
    use prm;
    use prm::{node::Node2D, boundaries::Boundaries};

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

    let start: Node2D = Node2D { x: 0f64, y: 0f64, idx: 0 };
    let goal: Node2D = Node2D { x: 3f64, y: 3f64, idx: 0 };
    let bounds: Boundaries = Boundaries { x_lower: 0f64, x_upper: 3f64, y_lower: 0f64, y_upper: 3f64 };
    let optimizer: &'static DefaultOptimizer = &optimizer::DefaultOptimizer;
    let mut pdef= ProblemDefinition::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer);                                       
    pdef.solve();
}