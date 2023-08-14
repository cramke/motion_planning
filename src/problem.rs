use std::fs::File;
use std::io::Write;

use crate::planner::base_planner::Planner;
use crate::space::Point;
use crate::types::Metric2D;

/// The Problem Definition serves as a collector for various planners and problems.
pub struct ProblemDefinition<T: Metric2D> {
    planner: Box<dyn Planner<T>>,
}

impl<T: Metric2D> ProblemDefinition<T> {
    pub fn new(planner: Box<dyn Planner<T>>) -> Self {
        ProblemDefinition { planner }
    }

    pub fn solve(&mut self) {
        self.planner.init();
        self.planner._run();
    }

    pub fn print_statistics(&self, path: &str) {
        self.planner.print_statistics(path);

        if self.planner.is_solved() {
            let path: Vec<Point<T>> = self.get_solution_path();
            println!(
                "Solution cost -{}- with {} nodes",
                self.get_solution_cost(),
                path.len()
            );
            for el in path {
                print!("{el:?}, ");
            }
            println!();
        } else {
            println!("No solution was found");
        }
    }

    pub fn get_solution_cost(&self) -> T {
        self.planner.get_solution_cost()
    }

    pub fn get_solution_path(&self) -> Vec<Point<T>> {
        self.planner.get_solution_path()
    }

    pub fn write_solution_path(&self, path: &str) {
        let mut file = match File::create(path) {
            Ok(file) => file,
            Err(_) => {
                println!("Could not open file {path}");
                return;
            }
        };

        match write!(file, "[") {
            Ok(_) => {}
            Err(_) => {
                println!("Could not write the solution path to file! -> {path}");
                return;
            }
        }

        for node in self.get_solution_path() {
            match write!(file, "{node:?}, ") {
                Ok(_) => {}
                Err(_) => {
                    println!("Could not write the solution path to file! -> {path}");
                    return;
                }
            }
        }

        match write!(file, "]") {
            Ok(_) => {}
            Err(_) => {
                println!("Could not write the solution path to file! -> {path}");
                return;
            }
        }
        println!("Written Solution path to file.");
    }
}

/// The Problem Definition serves as a collector for various planners and problems.
pub struct ProblemDefinition2<T: Metric2D> {
    pub start: Point<T>,
    pub goal: Point<T>,
    pub solution: Vec<Point<T>>,
}

impl<T: Metric2D> ProblemDefinition2<T> {
    pub fn new(start: Point<T>, goal: Point<T>) -> Self {
        ProblemDefinition2 {
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

impl<T: Metric2D> Default for ProblemDefinition2<T> {
    fn default() -> Self {
        ProblemDefinition2::new(Point::default(), Point::default())
    }
}

mod tests {

    #[test]
    fn test_dummy() {
        assert!(true);
    }
}
