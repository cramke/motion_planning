use crate::{
    boundaries::Boundaries, collision_checker::CollisionChecker, space::Point,
    types::SpaceContinuous,
};

/// Custom planners can use these traits to implement other algorithm for planning.
///
/// # Available Planners:
/// As of the time of writing the number is still extremely limited. This framework should simplify the implementation of further algorithms.
/// - PRM: Probabilistic Road Maps
///
/// # Planner
/// Defines a trait called `Planner` with generic type `T` that represents a continuous space.
///
/// The trait provides several methods for setting the start and goal points, boundaries, and collision checker, as well as initializing the planner, solving the planning problem, and getting the solution cost.
///
pub trait Planner<T: SpaceContinuous> {
    fn set_start(&mut self, start: Point<T>);
    fn set_goal(&mut self, goal: Point<T>);
    fn set_boundaries(&mut self, boundaries: Boundaries<T>);
    fn set_collision_checker(&mut self, cc: Box<dyn CollisionChecker<T>>);
    fn init(&mut self);
    fn solve(&mut self);
    fn get_solution_cost(&self) -> T;
}
