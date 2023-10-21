use crate::{
    boundaries::Boundaries, planner::base_planner::Planner, problem::ProblemDefinition,
    types::SpaceContinuous,
};

/// Defines a struct called `PlanningSetup` with generic type `T` that has four fields: `planner`, `problem`, `boundaries`, and `ready`.
pub struct PlanningSetup<T: SpaceContinuous> {
    pub planner: Box<dyn Planner<T>>,
    pub problem: ProblemDefinition<T>,
    pub boundaries: Boundaries<T>,
    pub ready: bool,
}

impl<T: SpaceContinuous> PlanningSetup<T> {
    /// Sets up the planner by configuring the start point, goal point, boundaries, and performing a sanity check.
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

    /// Checks if the lower boundaries are less than or equal to the upper boundaries for both the x and y axes.
    ///
    /// # Returns
    ///
    /// A boolean value indicating whether the boundaries are valid or not. `true` if the boundaries are valid, `false` otherwise.
    fn sanity_check(&self) -> bool {
        if self.boundaries.get_x_lower() > self.boundaries.get_x_upper() {
            return false;
        }
        if self.boundaries.get_y_lower() > self.boundaries.get_y_upper() {
            return false;
        }
        true
    }

    /// Solves the planning problem.
    ///
    /// # Panics
    /// Panics if the `PlanningSetup` component is not ready for solving.
    pub fn solve(&mut self) {
        if self.ready {
            self.planner.solve();
        } else {
            panic!("PlanningSetup is not ready for solving.")
        }
    }

    /// Retrieves the solution cost from the `planner` object and prints it to the console.
    ///
    /// # Returns
    ///
    /// The solution cost.
    pub fn get_statistics(&self) -> T {
        let cost = self.planner.get_solution_cost();
        println!("Cost: {}", cost);
        cost
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

    // Test that the 'solve' method panics if the 'ready' field is false.
    #[test]
    #[should_panic]
    fn test_solve_panics_if_not_ready() {
        let mut setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };
        setup.solve();
    }

    // Test the 'get_statistics' method of the 'test_setup_with_prm_new' function
    #[test]
    fn test_setup_with_prm_new_get_statistics() {
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
        let cost: f64 = setup.get_statistics();
        let expected_cost: f64 = f64::INFINITY;
        assert_eq!(cost, expected_cost);
    }

    // Test that the sanity check method returns true when the lower boundaries are less than or equal to the upper boundaries for both the x and y axes.
    #[test]
    fn test_sanity_check_returns_true() {
        let setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries: Boundaries::default(),
            ready: false,
        };

        let result = setup.sanity_check();

        assert_eq!(result, true);
    }

    // Test that the sanity check method returns true when the lower boundaries are equal to the upper boundaries for both the x and y axes.
    #[test]
    fn test_sanity_check_returns_true_when_boundaries_are_equal() {
        let boundaries = Boundaries::new(0.0, 0.0, 1.0, 1.0);
        let setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries,
            ready: false,
        };

        let result = setup.sanity_check();

        assert_eq!(result, true);
    }

    // Test that the sanity check method returns false when the lower y boundary is greater than the upper y boundary.
    #[test]
    fn test_sanity_check_returns_false_when_lower_y_boundary_is_greater_than_upper_y_boundary() {
        let boundaries = Boundaries::new(0.0, 0.0, 1.0, -1.0);
        let setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries,
            ready: false,
        };

        let result = setup.sanity_check();

        assert_eq!(result, false);
    }

    // Test that the sanity check method returns false when the lower x boundary is greater than the upper x boundary.
    #[test]
    fn test_sanity_check_returns_false_when_lower_x_boundary_is_greater_than_upper_x_boundary() {
        let boundaries = Boundaries::new(2.0, 0.0, 1.0, 1.0);
        let setup: PlanningSetup<f64> = PlanningSetup {
            planner: Box::<PRM<f64>>::default(),
            problem: ProblemDefinition::default(),
            boundaries,
            ready: false,
        };

        let result = setup.sanity_check();

        assert_eq!(result, false);
    }
}
