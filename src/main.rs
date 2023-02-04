use mpf::collision_checker::{CollisionChecker, NaiveCollisionChecker};
use mpf::boundaries::Boundaries;
use mpf::optimizer::{self, Optimizer};
use mpf::problem::{ProblemDefinition, Parameter};

use geo_types::Point;

fn main() {
    let start: Point = Point::new(0f64, 0f64);
    let goal: Point = Point::new(3f64, 3f64);
    let bounds: Boundaries = Boundaries::new(0f64, 3f64, 0f64, 3f64);
    let optimizer: Box<dyn Optimizer> = Box::new(optimizer::DefaultOptimizer);
    let params = Parameter::new(50usize);
    let cc: Box<dyn CollisionChecker> = NaiveCollisionChecker::new();
    let mut pdef= ProblemDefinition::new( start, goal, bounds, optimizer, params, cc);                                       
    pdef.solve();
    let path: &str = "./examples/example1/graph.dot";
    pdef.print_statistics(path);
    let path: &str = "./examples/example1/solution_path.txt";
    pdef.write_solution_path(path);
}