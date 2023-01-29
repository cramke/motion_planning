use prm::{self, problem::ProblemDefinition, optimizer::{DefaultOptimizer, Optimizer}};

#[test]
fn test_prm_default_scenario() {
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
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer);                                       
    pdef.solve();
    let cost = pdef.get_solution_cost();
    assert!(cost > 4f64);
    assert!(cost < 5f64);

}


#[test]
fn test_prm_naiv_scenario() {
    use prm;
    use prm::{node::Node2D, boundaries::Boundaries};

    fn is_collision(node: &Node2D) -> bool {
        return false;
    }
    
    fn is_edge_in_collision() -> bool {
        return false;
    }

    let start: Node2D = Node2D { x: 0f64, y: 0f64, idx: 0 };
    let goal: Node2D = Node2D { x: 3f64, y: 3f64, idx: 0 };
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer);                                       
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    println!("{}", cost);
    assert!(cost < 4.5f64);
    
}


#[test]
fn test_prm_unsolvable_scenario() {
    use prm;
    use prm::{node::Node2D, boundaries::Boundaries};

    fn is_collision(node: &Node2D) -> bool {
        return false;
    }
    
    fn is_edge_in_collision() -> bool {
        return true;
    }

    let start: Node2D = Node2D { x: 0f64, y: 0f64, idx: 0 };
    let goal: Node2D = Node2D { x: 3f64, y: 3f64, idx: 0 };
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64 );
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer);                                       
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    assert_eq!(f64::MAX, cost);
}