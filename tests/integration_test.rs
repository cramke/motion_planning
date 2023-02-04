use mpf::{self, problem::{ProblemDefinition, Parameter}, optimizer::{DefaultOptimizer, Optimizer}, collision_checker::{CollisionChecker, NaiveCollisionChecker}};

#[test]
fn test_mpf_default_scenario() {
    use mpf;
    use mpf::{boundaries::Boundaries};
    use geo_types::Point;

    fn is_collision(node: &Point) -> bool {
        if node.x() > 1.0 && node.x() < 2.0 {
            return true;
        }
        if node.y() > 1.0 && node.y() < 2.0 {
            return true;
        }
    
        return false;
    }
    
    fn is_edge_in_collision() -> bool {
        return false;
    }

    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let params = Parameter::new(18usize);
    let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);                                       
    pdef.solve();
    let cost = pdef.get_solution_cost();
    assert!(cost > 4f64);
    assert!(cost < 5f64);

}


#[test]
fn test_mpf_naiv_scenario() {
    use mpf;
    use mpf::{node::Node2D, boundaries::Boundaries};
    use geo_types::Point;

    fn is_collision(node: &Point) -> bool {
        return false;
    }
    
    fn is_edge_in_collision() -> bool {
        return false;
    }

    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let params = Parameter::new(18usize);
    let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);                                       
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    println!("{}", cost);
    assert!(cost < 4.5f64);
    
}