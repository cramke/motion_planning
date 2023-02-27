use geo::{Contains, Intersects};
use mpl::{self, problem::{ProblemDefinition, Parameter}, optimizer::{DefaultOptimizer, Optimizer}, collision_checker::{CollisionChecker, NaiveCollisionChecker}, boundaries::Boundaries};

#[test]
fn test_mpl_default_scenario() {
    use mpl;
    use mpl::{boundaries::Boundaries};
    use geo::Point;

    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let params = Parameter::new(18usize, 3usize);
    let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);                                       
    pdef.solve();
    let cost = pdef.get_solution_cost();
    assert!(cost > 3f64);
    assert!(cost < 6f64);
}


#[test]
fn test_mpl_naiv_scenario() {
    use mpl;
    use mpl::boundaries::Boundaries;
    use geo_types::Point;

    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let params = Parameter::new(25usize, 3usize);
    let cc: Box<dyn CollisionChecker> = Box::new(NaiveCollisionChecker{});
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);                                       
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    println!("{}", cost);
    assert!(cost < 6f64);
}

#[test]
fn test_geo_collision() {
    use geo::{Point, LineString, Polygon};

    pub struct GeoCollisionChecker {
        poly: Polygon,
    }
    
    impl CollisionChecker for GeoCollisionChecker {
        fn init(&self) -> bool {
            return true;
        }
    
        fn is_edge_colliding(&self, begin: &Point, end: &Point) -> bool {
            let a = Point::new(begin.x(), begin.y());
            let b = Point::new(end.x(), end.y());
            let line = LineString::from(vec![a, b]);
            return self.poly.intersects(&line);
        }
    
        fn is_node_colliding(&self, node: &Point) -> bool {
            return self.poly.contains(node);
        }
    }

    let poly = Polygon::new(
    LineString::from(vec![(1., 1.), (2., 1.), (2., 2.), (1., 2.)]),
    vec![],
    );

    let cc: Box<dyn CollisionChecker> = Box::new(GeoCollisionChecker{poly});
    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer);
    let params = Parameter::new(30usize, 3usize);
    let mut pdef: ProblemDefinition = ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);                                       
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    println!("{}", cost);
    assert!(cost > 3.5f64);
    assert!(cost < f64::MAX);
}