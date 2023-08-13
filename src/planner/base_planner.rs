use rand::distributions::uniform::SampleUniform;
use std::ops::{Sub, Add, Mul};

use crate::space::Point;

/// Custom planners can use these traits to implement other algorithm for planning. 
/// 
/// # Available Planners:
/// As of the time of writing the number is still extremely limited. This framework should simplify the implementation of further algorithms. 
/// - PRM: Probabilistic Road Maps
pub trait Planner<T: PartialOrd + SampleUniform + Sub + Add + Mul + std::fmt::Debug> {
    fn init(&mut self);
    fn run(&mut self);
    fn is_solved(&self) -> bool;
    fn print_statistics(&self, path:&str);
    fn get_solution_cost(&self) -> T;
    fn get_solution_path(&self) -> Vec<Point<T>>;
}