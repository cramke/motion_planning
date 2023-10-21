use std::time::Instant;

use mpl::boundaries::Boundaries;
use mpl::planner::base_planner::Planner;
use mpl::planner::prm::PRM;
use mpl::problem::ProblemDefinition;
use mpl::setup::PlanningSetup;
use mpl::space::Point;

fn main() {
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let mut planner: Box<PRM<f64>> = Box::default();
    planner.set_start(Point::new(1f64, 1f64));
    planner.set_goal(Point::new(2f64, 2f64));

    let mut setup = PlanningSetup {
        planner,
        problem: ProblemDefinition::default(),
        boundaries: bounds,
        ready: false,
    };

    println!("#### mpl ####");
    let start = Instant::now();
    setup.setup();
    setup.solve();
    let duration = start.elapsed();
    println!("Time elapsed in expensive_function() is: {duration:?}");
}
