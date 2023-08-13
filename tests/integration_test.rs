use std::marker::PhantomData;

use geo::{Contains, Intersects};
use mpl::{
    self,
    boundaries::Boundaries,
    collision_checker::{CollisionChecker, NaiveCollisionChecker},
    optimizer::{DefaultOptimizer, Optimizer},
    planner,
    problem::ProblemDefinition,
};

#[test]
fn test_mpl_default_scenario() {
    use mpl;
    use mpl::boundaries::Boundaries;
    use mpl::space::Point;

    let start: Point<f64> = Point { x: 0f64, y: 0f64 };
    let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
        phantom: PhantomData,
    });
    let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
        phantom: PhantomData,
    });
    let planner = Box::new(planner::prm::PRM::default());
    let mut pdef: ProblemDefinition<f64> =
        ProblemDefinition::new(start, goal, bounds, optimizer, cc, planner);
    pdef.solve();
    let cost = pdef.get_solution_cost();
    assert!(cost > 3f64);
    assert!(cost < 10f64);
}

#[test]
fn test_mpl_naiv_scenario() {
    use mpl;
    use mpl::boundaries::Boundaries;
    use mpl::space::Point;

    let start: Point<f64> = Point { x: 0f64, y: 0f64 };
    let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
        phantom: PhantomData,
    });
    let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
        phantom: PhantomData,
    });
    let planner = Box::new(planner::prm_star::PRMstar::default());
    let mut pdef: ProblemDefinition<f64> =
        ProblemDefinition::new(start, goal, bounds, optimizer, cc, planner);
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    println!("{}", cost);
    assert!(cost < 10f64);
}

#[test]
fn test_geo_collision() {
    use geo::Point as gp;
    use geo::{LineString, Polygon};
    use mpl::space::Point;

    pub struct GeoCollisionChecker {
        poly: Polygon,
    }

    impl CollisionChecker<f64> for GeoCollisionChecker {
        fn init(&self) -> bool {
            return true;
        }

        fn is_edge_colliding(&self, begin: &Point<f64>, end: &Point<f64>) -> bool {
            let a = gp::new(begin.x, begin.y);
            let b = gp::new(end.x, end.y);
            let line = LineString::from(vec![a, b]);
            return self.poly.intersects(&line);
        }

        fn is_node_colliding(&self, node: &Point<f64>) -> bool {
            let geo_node = gp::new(node.x, node.y);
            return self.poly.contains(&geo_node);
        }
    }

    let poly = Polygon::new(
        LineString::from(vec![(1., 1.), (2., 1.), (2., 2.), (1., 2.)]),
        vec![],
    );

    let cc: Box<dyn CollisionChecker<f64>> = Box::new(GeoCollisionChecker { poly });
    let start: Point<f64> = Point { x: 0f64, y: 0f64 };
    let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
        phantom: PhantomData,
    });
    let planner = Box::new(planner::prm_star::PRMstar::default());
    let mut pdef: ProblemDefinition<f64> =
        ProblemDefinition::new(start, goal, bounds, optimizer, cc, planner);
    pdef.solve();
    let cost: f64 = pdef.get_solution_cost();
    println!("{}", cost);
    assert!(cost > 2.0f64);
    assert!(cost < f64::MAX);
}
