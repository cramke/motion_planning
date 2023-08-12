use std::time::Instant;

use mpl::collision_checker::{CollisionChecker, NaiveCollisionChecker};
use mpl::boundaries::Boundaries;
use mpl::optimizer::{self, Optimizer};
use mpl::problem::{ProblemDefinition, Parameter};
use mpl::space::Point;

fn main() {
    let start: Point<f64> = Point{x:0f64, y:0f64};
    let goal: Point<f64> = Point{x:3f64, y:3f64};
    let bounds: Boundaries<f64> = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(optimizer::DefaultOptimizer);
    let params = Parameter::new(10usize, 1usize);
    let cc: Box<dyn CollisionChecker> = NaiveCollisionChecker::new_box();
    let mut pdef= ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);    

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