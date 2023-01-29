use petgraph::dot::{Dot, Config};

use crate::node::Node2D;
use crate::boundaries::Boundaries;
use crate::optimizer::Optimizer;
use crate::prm::PRM;

pub struct ProblemDefinition {
    planner: PRM,
}

impl ProblemDefinition {
    pub fn new(start: Node2D, goal: Node2D, bounds: Boundaries, is_collision: fn(&Node2D) -> bool, 
    is_edge_in_collision: fn() -> bool, optimizer: Box<dyn Optimizer>) -> Self {
        let setup = PRM::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer); 
        let pdef = ProblemDefinition {planner: setup};
        return pdef;
    }

    pub fn solve(&mut self) {
        self.planner.init();
        self.planner.run();
    }

    pub fn print_statistics(&self) {
        let nodes: usize = self.planner.graph.node_count();
        println!("Graph contains {} nodes", nodes);

        let edges: usize = self.planner.graph.edge_count();
        println!("Graph contains {} edges", edges);

        if self.planner.is_solved {
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
        return self.planner.solution_path.clone();
    }

    pub fn print_graph(&self) {
        println!("{:?}", Dot::with_config(self.planner.get_graph(), &[Config::EdgeNoLabel]));
    }
}