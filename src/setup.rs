use crate::{
    boundaries::Boundaries, planner::base_planner::Planner, problem::ProblemDefinition,
    types::SpaceContinuous,
};

pub struct PlanningSetup<T: SpaceContinuous> {
    pub planner: Box<dyn Planner<T>>,
    pub problem: ProblemDefinition<T>,
    pub boundaries: Boundaries<T>,
    pub ready: bool,
}

impl<T: SpaceContinuous> PlanningSetup<T> {
    pub fn setup(&mut self) {
        self.planner.set_start(self.problem.get_start());
        self.planner.set_goal(self.problem.get_goal());
        self.planner.set_boundaries(self.boundaries.clone());
        if self.sanity_check() {
            self.planner.init();
            self.ready = true;
        } else {
            self.ready = false;
        }
    }

    fn sanity_check(&self) -> bool {
        true
    }

    pub fn solve(&mut self) {
        if self.ready {
            self.planner.solve();
        } else {
            panic!("PlanningSetup is not ready for solving.")
        }
    }
}

#[cfg(test)]
mod test {
    use crate::boundaries::Boundaries;
    use crate::planner::prm::PRM;
    use crate::problem::ProblemDefinition;
    use crate::setup::PlanningSetup;
    use crate::space::Point;

    #[test]
    fn test_setup_with_prm_new() {
        let setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };

        assert!(!setup.ready);
    }

    #[test]
    fn test_setup_with_prm_setup() {
        let mut setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };

        assert!(!setup.ready);
        setup.setup();
        assert!(setup.ready);
    }

    #[test]
    #[should_panic]
    fn test_setup_with_prm_solve_wo_setup() {
        let mut setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };
        setup.solve();
    }

    #[test]
    fn test_setup_with_prm_solve() {
        let start: Point<f64> = Point::new(1f64, 1f64);
        let goal: Point<f64> = Point::new(2f64, 2f64);

        let mut setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::new(start, goal),
            boundaries: Boundaries::default(),
            ready: false,
        };

        setup.setup();
        setup.solve();
    }
}
