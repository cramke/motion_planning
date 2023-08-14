use crate::{
    boundaries::Boundaries, core::Metric2D, planner::base_planner::Planner2,
    problem::ProblemDefinition2,
};

pub struct PlanningSetup<T: Metric2D> {
    pub planner: Box<dyn Planner2<T>>,
    pub problem: ProblemDefinition2<T>,
    pub boundaries: Boundaries<T>,
    ready: bool,
}

impl<T: Metric2D> PlanningSetup<T> {
    pub fn setup(&mut self) {
        self.planner.set_start(self.problem.get_start());
        self.planner.set_goal(self.problem.get_goal());
        self.planner.set_boundaries(self.boundaries.clone());

        self.ready = true;
    }

    pub fn solve(&mut self) {
        self.planner.solve();
    }
}

#[cfg(test)]
mod test {
    use crate::boundaries::Boundaries;
    use crate::collision_checker::{CollisionChecker, NaiveCollisionChecker};
    use crate::planner::prm::PRM;
    use crate::problem::ProblemDefinition2;
    use crate::setup::PlanningSetup;
    use std::marker::PhantomData;

    #[test]
    fn test_setup_with_prm_new() {
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });

        let setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::new(PRM::new(cc)),
            problem: ProblemDefinition2::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };

        assert!(!setup.ready);
    }

    #[test]
    fn test_setup_with_prm_setup() {
        let cc: Box<dyn CollisionChecker<f64>> = Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        });

        let mut setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::new(PRM::new(cc)),
            problem: ProblemDefinition2::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };

        assert!(!setup.ready);
        setup.setup();
        assert!(setup.ready);
    }
}
