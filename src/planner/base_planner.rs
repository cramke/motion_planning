use crate::{
    boundaries::Boundaries, collision_checker::CollisionChecker, space::Point, types::Metric2D,
};

/// Custom planners can use these traits to implement other algorithm for planning.
///
/// # Available Planners:
/// As of the time of writing the number is still extremely limited. This framework should simplify the implementation of further algorithms.
/// - PRM: Probabilistic Road Maps
pub trait Planner<T: Metric2D> {
    fn init(&mut self);
    fn _run(&mut self);
    fn is_solved(&self) -> bool;
    fn print_statistics(&self, path: &str);
    fn get_solution_cost(&self) -> T;
    fn get_solution_path(&self) -> Vec<Point<T>>;
}

pub trait Planner2<T: Metric2D> {
    fn set_start(&mut self, start: Point<T>);
    fn set_goal(&mut self, goal: Point<T>);
    fn set_boundaries(&mut self, boundaries: Boundaries<T>);
    fn set_collision_checker(&mut self, cc: Box<dyn CollisionChecker<T>>);
    fn init(&mut self);
    fn solve(&mut self);
    fn get_solution_cost(&self) -> T;
}
