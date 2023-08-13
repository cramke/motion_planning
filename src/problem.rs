use num::{Bounded, Float, Signed};
use rand::distributions::uniform::SampleUniform;
use std::fmt::Display;
use std::fs::File;
use std::io::Write;
use std::ops::{Add, Mul, Sub};

use crate::boundaries::Boundaries;
use crate::collision_checker::CollisionChecker;
use crate::core::Metric2D;
use crate::optimizer::Optimizer;
use crate::planner::base_planner::Planner;
use crate::planner::prm_star::PRMstar;
use crate::space::Point;

#[derive(Clone, Copy)]
pub struct Parameter {
    pub max_size: usize,
    pub k_nearest_neighbors: usize,
}

impl Parameter {
    pub fn new(param1: usize, param2: usize) -> Self {
        Parameter {
            max_size: param1,
            k_nearest_neighbors: param2,
        }
    }
}

impl Default for Parameter {
    fn default() -> Self {
        Parameter {
            max_size: 64,
            k_nearest_neighbors: 10,
        }
    }
}

/// The Problem Definition serves as a collector for various planners and problems.
pub struct ProblemDefinition<T> {
    planner: Box<dyn Planner<T>>,
}

impl<
        T: PartialOrd
            + SampleUniform
            + Sub
            + Add
            + Mul
            + Bounded
            + Float
            + Signed
            + std::fmt::Debug
            + Default
            + Display
            + ToString
            + Metric2D
            + 'static,
    > ProblemDefinition<T>
{
    pub fn new(
        start: Point<T>,
        goal: Point<T>,
        bounds: Boundaries<T>,
        optimizer: Box<dyn Optimizer<T>>,
        params: Parameter,
        collision_checker: Box<dyn CollisionChecker<T>>,
    ) -> Self {
        let planner: Box<dyn Planner<T>> = Box::new(PRMstar::new(
            start,
            goal,
            bounds,
            optimizer,
            params,
            collision_checker,
        ));
        ProblemDefinition { planner }
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
    fn test_parameter_new() {
        use super::Parameter;
        let params = Parameter::new(21, 19);
        assert_eq!(params.max_size, 21usize);
        assert_eq!(params.k_nearest_neighbors, 19);
    }

    #[test]
    fn test_parameter_default() {
        use super::Parameter;
        let params = Parameter::default();
        assert_eq!(params.max_size, 64usize);
        assert_eq!(params.k_nearest_neighbors, 10);
    }
}
