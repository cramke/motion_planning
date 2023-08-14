use crate::{
    boundaries::Boundaries, core::Metric2D, planner::base_planner::Planner,
    problem::ProblemDefinition,
};

pub struct PlanningSetup<T: Metric2D> {
    pub planner: Box<dyn Planner<T>>,
    pub problem: ProblemDefinition<T>,
    pub boundaries: Boundaries<T>,
}
