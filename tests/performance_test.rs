use mpl::{
    boundaries::Boundaries, planner::prm::PRM, problem::ProblemDefinition, setup::PlanningSetup,
    space::Point,
};
use std::time::Instant;

#[test]
fn test_performance_prm() {
    let start: Point = Point::new(1f64, 1f64);
    let goal: Point = Point::new(2f64, 2f64);

    let mut planner: Box<PRM> = Box::default();
    planner.config.max_size = 10usize;
    let problem = ProblemDefinition::new(start, goal);
    let boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);

    let mut setup: PlanningSetup = PlanningSetup {
        planner,
        problem,
        boundaries,
        ready: false,
    };
    setup.setup();

    println!("#### mpl ####");
    let start = Instant::now();
    setup.solve();
    let duration = start.elapsed();
    println!("Time elapsed in expensive_function() is: {duration:?}");
    let cost1: f64 = setup.get_statistics();

    let mut planner2: Box<PRM> = Box::default();
    planner2.config.max_size = 1000usize;
    setup.planner = planner2;
    setup.setup();
    println!("#### mpl ####");
    let start2 = Instant::now();
    setup.solve();
    let duration2 = start2.elapsed();
    println!("Time elapsed in expensive_function() is: {duration2:?}");
    let cost2: f64 = setup.get_statistics();

    let time_increase = if duration2 > 3 * duration {
        true
    } else {
        false
    };
    assert!(time_increase);
    let cost_improvement: bool = if cost2 < 0.5 * cost1 { true } else { false };
    assert!(cost_improvement)
}
