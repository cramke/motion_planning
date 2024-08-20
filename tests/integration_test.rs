use std::marker::PhantomData;

use geo::{Contains, Intersects};
use mpl::{
    boundaries::Boundaries,
    collision_checker::CollisionChecker,
    optimizer::{DefaultOptimizer, Optimizer},
    planner::{base_planner::Planner, prm::PRM, prm_star::PRMstar},
    problem::ProblemDefinition,
    setup::PlanningSetup,
    space::Point,
};

#[test]
fn test_prmstar_scenario() {
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let planner = Box::<PRMstar>::default();
    let start = Point::new(0f64, 0f64);
    let goal = Point::new(3f64, 3f64);
    let pdef: ProblemDefinition = ProblemDefinition::new(start, goal);
    let mut setup: PlanningSetup = PlanningSetup {
        planner,
        problem: pdef,
        boundaries: bounds,
        ready: false,
    };
    setup.setup();
    setup.solve();
    let cost = setup.planner.get_solution_cost();

    assert!(cost > 2f64);
    assert!(cost < 10f64);
}

#[test]
fn test_prm_naiv_scenario() {
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let planner = Box::<PRM>::default();
    let start = Point::new(0f64, 0f64);
    let goal = Point::new(3f64, 3f64);
    let pdef: ProblemDefinition = ProblemDefinition::new(start, goal);

    let mut setup: PlanningSetup = PlanningSetup {
        planner,
        problem: pdef,
        boundaries: bounds,
        ready: false,
    };
    setup.setup();
    setup.solve();
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

    impl CollisionChecker for GeoCollisionChecker {
        fn init(&self) -> bool {
            true
        }

        fn is_edge_colliding(&self, begin: &Point, end: &Point) -> bool {
            let a = gp::new(begin.get_x(), begin.get_y());
            let b = gp::new(end.get_x(), end.get_y());
            let line = LineString::from(vec![a, b]);
            self.poly.intersects(&line)
        }

        fn is_node_colliding(&self, node: &Point) -> bool {
            let geo_node = gp::new(node.get_x(), node.get_y());
            self.poly.contains(&geo_node)
        }
    }

    let poly = Polygon::new(
        LineString::from(vec![(1., 1.), (2., 1.), (2., 2.), (1., 2.)]),
        vec![],
    );

    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(DefaultOptimizer {
        phantom: PhantomData,
    });
    let mut pdef: ProblemDefinition = ProblemDefinition::default();
    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    pdef.set_start(start);
    pdef.set_goal(goal);

    let mut planner: Box<PRMstar> = Box::default();
    planner.optimizer = optimizer;
    let cc: Box<GeoCollisionChecker> = Box::new(GeoCollisionChecker { poly });
    planner.set_collision_checker(cc);

    let mut setup: PlanningSetup = PlanningSetup {
        planner,
        problem: pdef,
        boundaries: bounds,
        ready: false,
    };
    setup.setup();
    setup.solve();
    let cost: f64 = setup.planner.get_solution_cost();
    println!("{}", cost);

    assert!(cost > 2.0f64);
    assert!(cost < f64::MAX);
}
