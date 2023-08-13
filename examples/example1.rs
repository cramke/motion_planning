use std::time::Instant;

use mpl::boundaries::Boundaries;
use mpl::collision_checker::{CollisionChecker, NaiveCollisionChecker};
use mpl::planner;
use mpl::problem::ProblemDefinition;
use mpl::space::Point;

fn main() {
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let cc: Box<dyn CollisionChecker<f64>> = NaiveCollisionChecker::new_box();
    let mut planner = Box::new(planner::prm::PRM::default());
    planner.start = Point { x: 0f64, y: 0f64 };
    planner.goal = Point { x: 3f64, y: 3f64 };
    planner.boundaries = bounds;
    planner.collision_checker = cc;
    let mut pdef = ProblemDefinition::new(planner);

    println!("#### mpl ####");
    let start = Instant::now();
    pdef.solve();
    let duration = start.elapsed();
    println!("Time elapsed in expensive_function() is: {duration:?}");

    let path: &str = "./graph.dot";
    pdef.print_statistics(path);
    let path: &str = "./solution_path.txt";
    pdef.write_solution_path(path);
}
