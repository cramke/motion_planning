use crate::space::Point;
use crate::types::SpaceContinuous;

/// The Problem Definition serves as a collector for various planners and problems.
pub struct ProblemDefinition<T: SpaceContinuous> {
    pub start: Point<T>,
    pub goal: Point<T>,
    pub solution: Vec<Point<T>>,
}

impl<T: SpaceContinuous> ProblemDefinition<T> {
    pub fn new(start: Point<T>, goal: Point<T>) -> Self {
        ProblemDefinition {
            start,
            goal,
            solution: Vec::new(),
        }
    }

    pub fn get_start(&self) -> Point<T> {
        self.start
    }

    pub fn get_goal(&self) -> Point<T> {
        self.goal
    }

    pub fn set_start(&mut self, start: Point<T>) {
        self.start = start;
    }

    pub fn set_goal(&mut self, goal: Point<T>) {
        self.goal = goal;
    }
}

impl<T: SpaceContinuous> Default for ProblemDefinition<T> {
    fn default() -> Self {
        ProblemDefinition::new(Point::default(), Point::default())
    }
}

mod tests {

    #[test]
    fn test_dummy() {
        assert!(true);
    }
}
