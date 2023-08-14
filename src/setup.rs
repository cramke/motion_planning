use crate::{planner::base_planner::Planner, problem::ProblemDefinition, core::Metric2D};

pub struct PlanningSetup<T: Metric2D> {
    pub planner: Box<dyn Planner<T>>,
    pub problem: ProblemDefinition<T>
}