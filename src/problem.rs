use std::fs::File;
use std::io::Write;

use crate::boundaries::Boundaries;
use crate::collision_checker::CollisionChecker;
use crate::core::Metric2D;
use crate::optimizer::Optimizer;
use crate::planner::base_planner::Planner;
use crate::space::Point;

/// The Problem Definition serves as a collector for various planners and problems.
pub struct ProblemDefinition<T: Metric2D> {
    planner: Box<dyn Planner<T>>,
}

impl<T: Metric2D> ProblemDefinition<T> {
    pub fn new(
        start: Point<T>,
        goal: Point<T>,
        bounds: Boundaries<T>,
        optimizer: Box<dyn Optimizer<T>>,
        collision_checker: Box<dyn CollisionChecker<T>>,
        planner: Box<dyn Planner<T>>,
    ) -> Self {
        ProblemDefinition { planner: planner }
    }

    pub fn solve(&mut self) {
        self.planner.init();
        self.planner.run();
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

mod tests {

    #[test]
    fn test_dummy() {
        assert!(true);
    }
}
