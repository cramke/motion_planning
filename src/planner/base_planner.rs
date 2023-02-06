use geo_types::Point;

/// Custom planners can use these traits to implement other algorithm for planning. 
/// 
/// # Available Planners:
/// As of the time of writing the number is still extremely limited. This framework should simplify the implementation of further algorithms. 
/// - PRM: Probabilistic Road Maps
pub trait Planner {
    fn init(&mut self);
    fn run(&mut self);
    fn is_solved(&self) -> bool;
    fn print_statistics(&self, path:&str);
    fn get_solution_cost(&self) -> f64;
    fn get_solution_path(&self) -> Vec<Point>;
}