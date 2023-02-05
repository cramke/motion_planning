use std::fs::File;
use std::io::Write;
use geo_types::Point;

use crate::collision_checker::CollisionChecker;
use crate::boundaries::Boundaries;
use crate::optimizer::Optimizer;
use crate::planner::planner::Planner;
use crate::planner::prm::PRM;

#[derive(Clone, Copy)]
pub struct Parameter {
    pub max_size: usize,
    pub k_nearest_neighbors: usize,
}

impl Parameter {
    pub fn new(param1: usize, param2: usize) -> Self {
        let params: Parameter = Parameter {max_size: param1, k_nearest_neighbors: param2};
        return params;
    }

    pub fn default() -> Self {
        Parameter { max_size: 40, k_nearest_neighbors: 8 }
    }
}

/// The Problem Definition serves as a collector for various planners and problems. 
pub struct ProblemDefinition {
    planner: Box<dyn Planner>,
    params: Parameter
}

impl ProblemDefinition {
    pub fn new(start: Point, goal: Point, bounds: Boundaries, optimizer: Box<dyn Optimizer>, params: Parameter, collision_checker: Box<dyn CollisionChecker>) -> Self {
        let planner: Box<dyn Planner> = Box::new(PRM::new( start, goal, bounds, optimizer, params.clone(), collision_checker));
        let pdef = ProblemDefinition {planner, params};
        return pdef;
    }

    pub fn solve(&mut self) {
        self.planner.set_params(&self.params);
        self.planner.init();
        self.planner.run();
    }

    pub fn print_statistics(&self, path:&str) {
        self.planner.print_statistics(path);

        if self.planner.is_solved() {
            let path = self.get_solution_path();
            println!("Solution cost -{}- with {} nodes", self.get_solution_cost(), path.len());
            for el in path {
                print!("{el:?}, ");
            }
            println!();
        } else {
            println!("No solution was found");
        }
    }

    pub fn get_solution_cost(&self) -> f64 {
        return self.planner.get_solution_cost();
    }

    pub fn get_solution_path(&self) -> Vec<Point> {
        return self.planner.get_solution_path();
    }

    pub fn write_solution_path(&self, path:&str) {
        let mut file = match File::create(path) {
            Ok(file) => file,
            Err(_) => {
                println!("Could not open file {path}");
                return;
            }
        };

        match write!(file, "[") {
            Err(_) => {
                println!("Could not write the solution path to file! -> {path:?}"); 
                return; }
            Ok(_) => {},
        };

        for node in self.get_solution_path() {
            match write!(file, "{node:?}, "){
                Err(_) => {
                    println!("Could not write the solution path to file! -> {path}"); 
                    return; }
                Ok(_) => {},
            };
        }

        match write!(file, "]") {
            Err(_) => println!("Could not write the solution path to file! -> {path}"),
            Ok(_) => println!("Write solution path to file: {path}"),
        };
    }
}