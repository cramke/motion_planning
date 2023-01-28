use petgraph::dot::{Dot, Config};
use petgraph::graph::{NodeIndex};

use crate::node::Node2D;
use crate::boundaries::Boundaries;
use crate::optimizer::{self, Optimizer};
use crate::prm::PRM;

pub struct ProblemDefinition {
    setup: PRM,
}

impl ProblemDefinition {
    pub fn new(start: Node2D, goal: Node2D, bounds: Boundaries, is_collision: fn(&Node2D) -> bool, 
    is_edge_in_collision: fn() -> bool, optimizer: Box<dyn Optimizer>) -> Self {
        let setup = PRM::new( start, goal, bounds, is_collision, is_edge_in_collision, optimizer); 
        let pdef = ProblemDefinition {setup};
        return pdef;
    }

    pub fn solve(&mut self) {
        self.setup.init();
        self.setup.run();
    }

    pub fn print_statistics(&self) {
        let nodes: usize = self.setup.graph.node_count();
        println!("Graph contains {} nodes", nodes);

        let edges: usize = self.setup.graph.edge_count();
        println!("Graph contains {} edges", edges);

        let path: &Vec<NodeIndex>;
        let cost: f64;
        match &self.setup.solution {
            None => println!("No solution was found"),
            Some(a) => {
                cost = a.0;
                path = &a.1;
                println!("Solution cost -{}- with {} nodes", cost, path.len());
            }
        }
    }

    pub fn get_solution_cost(&self) -> f64 {
        match &self.setup.solution {
            None => return f64::MAX,
            Some(a) => {
                return a.0;
            }
        }
    }

    pub fn print_graph(&self) {
        println!("{:?}", Dot::with_config(self.setup.get_graph(), &[Config::EdgeNoLabel]));
    }
}