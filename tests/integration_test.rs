use std::marker::PhantomData;

use geo::{Contains, Intersects};
use mpl::{
    boundaries::Boundaries,
    collision_checker::CollisionChecker,
    optimizer::{DefaultOptimizer, Optimizer},
    planner::{base_planner::Planner2, prm::PRM, prm_star::PRMstar},
    problem::ProblemDefinition2,
    setup::PlanningSetup,
    space::Point,
};

#[test]
fn test_prmstar_scenario() {
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let planner = Box::new(PRMstar::default());
    let start = Point { x: 0f64, y: 0f64 };
    let goal = Point { x: 3f64, y: 3f64 };
    let pdef: ProblemDefinition2<f64> = ProblemDefinition2::new(start, goal);

    let setup: PlanningSetup<f64> = PlanningSetup {
        planner,
        problem: pdef,
        boundaries: bounds,
        ready: false,
    };
    let cost = setup.planner.get_solution_cost();
    assert!(cost > 2f64);
    assert!(cost < 10f64);
}

#[test]
fn test_prm_naiv_scenario() {
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let planner = Box::new(PRM::default());
    let start = Point { x: 0f64, y: 0f64 };
    let goal = Point { x: 3f64, y: 3f64 };
    let pdef: ProblemDefinition2<f64> = ProblemDefinition2::new(start, goal);

    let setup: PlanningSetup<f64> = PlanningSetup {
        planner,
        problem: pdef,
        boundaries: bounds,
        ready: false,
    };
    let cost = setup.planner.get_solution_cost();
    assert!(cost > 2f64);
    assert!(cost < 10f64);
}

#[test]
#[should_panic]
fn test_rrt_naiv_scenario() {
    panic!("Not yet implemented");
}

#[test]
fn test_geo_collision() {
    use geo::Point as gp;
    use geo::{LineString, Polygon};

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

    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer<f64>> = Box::new(DefaultOptimizer {
        phantom: PhantomData,
    });
    let mut pdef: ProblemDefinition2<f64> = ProblemDefinition2::default();
    let start: Point<f64> = Point { x: 0f64, y: 0f64 };
    let goal: Point<f64> = Point { x: 3f64, y: 3f64 };
    pdef.set_start(start);
    pdef.set_goal(goal);

    let mut planner: Box<PRMstar<f64>> = Box::new(PRMstar::default());
    planner.optimizer = optimizer;
    let cc: Box<GeoCollisionChecker> = Box::new(GeoCollisionChecker { poly });
    planner.set_collision_checker(cc);

    let mut setup: PlanningSetup<f64> = PlanningSetup {
        planner,
        problem: pdef,
        boundaries: bounds,
        ready: false,
    };

    setup.solve();
    let cost: f64 = setup.planner.get_solution_cost();
    println!("{}", cost);
    assert!(cost > 2.0f64);
    assert!(cost < f64::MAX);
}
