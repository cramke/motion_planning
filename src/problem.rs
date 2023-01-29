use std::fs::File;
use std::io::Write;

use crate::node::Node2D;
use crate::boundaries::Boundaries;
use crate::optimizer::Optimizer;
use crate::planner::planner::Planner;
use crate::planner::prm::PRM;

pub struct Parameter {
    MAX_SIZE: usize,
}

impl Parameter {
    pub fn new(param1: usize) -> Self {
        let params: Parameter = Parameter {MAX_SIZE: param1};
        return params;
    }
}

pub struct ProblemDefinition {
    planner: Box<dyn Planner>,
    params: Parameter,
}

impl ProblemDefinition {
    pub fn new(start: Node2D, goal: Node2D, bounds: Boundaries, is_collision: fn(&Node2D) -> bool, 
    is_edge_in_collision: fn() -> bool, optimizer: Box<dyn Optimizer>, params: Parameter) -> Self {
        let planner: Box<dyn Planner> = Box::new(PRM::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer, params.MAX_SIZE));
        let params: Parameter = params;
        let pdef = ProblemDefinition {planner: planner, params: params};
        return pdef;
    }

    pub fn solve(&mut self) {
        self.planner.init();
        self.planner.run();
    }

    pub fn print_statistics(&self, path:&str) {
        self.planner.print_statistics(path);

        if self.planner.is_solved() {
            let path = self.get_solution_path();
            println!("Solution cost -{}- with {} nodes", self.get_solution_cost(), path.len());
            for el in path {
                print!("{:?}, ", el);
            }
            print!("\n");
        } else {
            println!("No solution was found");
        }
    }

    pub fn get_solution_cost(&self) -> f64 {
        return self.planner.get_solution_cost();
    }

    pub fn get_solution_path(&self) -> Vec<Node2D> {
        return self.planner.get_solution_path().clone();
    }

    pub fn write_solution_path(&self, path:&str) {
        let mut f = File::create(path).unwrap();
        write!(f, "[");
        for node in self.get_solution_path() {
            write!(f, "{}, ",node);
        }
        write!(f, "]");
    }
}