use crate::node::Node2D;

pub trait Planner {
    fn init(&mut self);
    fn run(&mut self);
    fn get_solution_cost(&self) -> f64;
    fn get_solution_path(&self) -> Vec<Node2D>;
    fn is_solved(&self) -> bool;
    fn print_statistics(&self, path:&str);
}