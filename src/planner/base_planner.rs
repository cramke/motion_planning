use crate::{space::Point, core::Metric2D};

/// Custom planners can use these traits to implement other algorithm for planning. 
/// 
/// # Available Planners:
/// As of the time of writing the number is still extremely limited. This framework should simplify the implementation of further algorithms. 
/// - PRM: Probabilistic Road Maps
pub trait Planner<T: Metric2D> {
    fn init(&mut self);
    fn run(&mut self);
    fn is_solved(&self) -> bool;
    fn print_statistics(&self, path:&str);
    fn get_solution_cost(&self) -> T;
    fn get_solution_path(&self) -> Vec<Point<T>>;
}