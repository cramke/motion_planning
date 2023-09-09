use crate::space::Point;
use crate::types::SpaceContinuous;

/// The Problem Definition serves as a collector for various planners and problems.
pub struct ProblemDefinition<T: SpaceContinuous> {
    pub start: Point<T>,
    pub goal: Point<T>,
    pub solution: Vec<Point<T>>,
}

/// Implements methods for a generic struct called `ProblemDefinition`.
///
/// The `ProblemDefinition` struct provides functionality to create a new instance, get and set the start and goal points, and retrieve the start and goal points.
///
///
/// # Type Parameters
/// - `T`: A type that implements the `SpaceContinuous` trait.
impl<T: SpaceContinuous> ProblemDefinition<T> {
    /// Creates a new instance of `ProblemDefinition` with the provided start and goal points, and an empty solution vector.
    ///
    /// # Arguments
    /// - `start`: The start point of the problem.
    /// - `goal`: The goal point of the problem.
    ///
    /// # Returns
    /// A new instance of `ProblemDefinition` with the provided start and goal points.
    pub fn new(start: Point<T>, goal: Point<T>) -> Self {
        ProblemDefinition {
            start,
            goal,
            solution: Vec::new(),
        }
    }

    /// Returns the start point of the problem.
    ///
    /// # Returns
    /// The start point of the problem.
    pub fn get_start(&self) -> Point<T> {
        self.start
    }

    /// Returns the goal point of the problem.
    ///
    /// # Returns
    /// The goal point of the problem.
    pub fn get_goal(&self) -> Point<T> {
        self.goal
    }

    /// Updates the start point of the problem with the provided point.
    ///
    /// # Arguments
    /// - `start`: The new start point of the problem.
    pub fn set_start(&mut self, start: Point<T>) {
        self.start = start;
    }

    /// Updates the goal point of the problem with the provided point.
    ///
    /// # Arguments
    /// - `goal`: The new goal point of the problem.
    pub fn set_goal(&mut self, goal: Point<T>) {
        self.goal = goal;
    }
}

impl<T: SpaceContinuous> Default for ProblemDefinition<T> {
    fn default() -> Self {
        ProblemDefinition::new(Point::default(), Point::default())
    }
}

#[cfg(test)]
mod tests {
    use super::ProblemDefinition;
    use crate::space::Point;

    // Test that a new instance of ProblemDefinition is created with default values with type f64.
    #[test]
    fn test_create_new_instance_with_default_values_f64() {
        let prd: ProblemDefinition<f64> = ProblemDefinition::default();
        assert_eq!(prd.get_start(), Point::default());
        assert_eq!(prd.get_goal(), Point::default());
        assert_eq!(prd.solution.len(), 0);
    }

    // Test that a new instance of ProblemDefinition is created with default values with type f32.
    #[test]
    fn test_create_new_instance_with_default_values_f32() {
        let prd: ProblemDefinition<f32> = ProblemDefinition::default();
        assert_eq!(prd.get_start(), Point::default());
        assert_eq!(prd.get_goal(), Point::default());
        assert_eq!(prd.solution.len(), 0);
    }

    // Test that a new instance of ProblemDefinition is created with custom start and goal points.
    #[test]
    fn test_create_custom_instance() {
        // Arrange
        let start: Point<f64> = Point::new(1.0, 2.0);
        let goal: Point<f64> = Point::new(3.0, 4.0);

        // Act
        let problem_def: ProblemDefinition<f64> = ProblemDefinition::new(start, goal);

        // Assert
        assert_eq!(problem_def.get_start(), start);
        assert_eq!(problem_def.get_goal(), goal);
    }

    // Test the functionality of retrieving the start point of a ProblemDefinition instance.
    #[test]
    fn retrieve_start_point() {
        // Create a new ProblemDefinition instance with a start point of (1, 2)
        let start_point: Point<f64> = Point::new(1.0, 2.0);
        let problem_definition: ProblemDefinition<f64> =
            ProblemDefinition::new(start_point, Point::default());

        // Retrieve the start point
        let retrieved_start_point: Point<f64> = problem_definition.get_start();

        // Check if the retrieved start point is equal to the original start point
        assert_eq!(retrieved_start_point, start_point);
    }

    // Test the functionality of retrieving the goal point from a ProblemDefinition instance.
    #[test]
    fn retrieve_goal_point() {
        // Create a new ProblemDefinition instance with a start point and a goal point
        let start: Point<f64> = Point::new(0.0, 0.0);
        let goal: Point<f64> = Point::new(1.0, 1.0);
        let problem_definition: ProblemDefinition<f64> = ProblemDefinition::new(start, goal);

        // Retrieve the goal point from the ProblemDefinition instance
        let retrieved_goal: Point<f64> = problem_definition.get_goal();

        // Check if the retrieved goal point matches the expected goal point
        assert_eq!(retrieved_goal, goal);
    }

    // Test if the start point of a ProblemDefinition instance is updated correctly.
    #[test]
    fn update_start_point() {
        // Create a new ProblemDefinition instance
        let mut problem_def: ProblemDefinition<f64> = ProblemDefinition::default();

        // Set the start point to (1.0, 2.0)
        let start_point: Point<f64> = Point::new(1.0, 2.0);
        problem_def.set_start(start_point);

        // Check if the start point is updated correctly
        assert_eq!(problem_def.get_start(), start_point);
    }

    // Test the functionality of updating the goal point of a ProblemDefinition instance.
    #[test]
    fn update_goal_point() {
        // Create a new ProblemDefinition instance with start and goal points
        let mut problem_def: ProblemDefinition<f64> =
            ProblemDefinition::new(Point::new(0.0, 0.0), Point::new(1.0, 1.0));

        // Update the goal point
        problem_def.set_goal(Point::new(2.0, 2.0));

        // Check if the goal point is updated correctly
        assert_eq!(problem_def.get_goal(), Point::new(2.0, 2.0));
    }

    // Test that the solution vector is empty
    #[test]
    fn test_solution_vector_empty() {
        let prd: ProblemDefinition<f32> = ProblemDefinition::default();
        assert_eq!(prd.solution.len(), 0);
    }
}
