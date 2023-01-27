use crate::node::Node2D;

pub trait Optimizer {
    fn get_edge_weight(&self, begin: &Node2D, end: &Node2D) -> f64;
}

#[derive(Debug, Copy, Clone)]
pub struct DefaultOptimizer;
impl Optimizer for DefaultOptimizer {
    fn get_edge_weight(&self, begin: &Node2D, end: &Node2D) -> f64 {
        let a = (begin.x - end.x).powi(2);
        let b: f64 = (begin.y - end.y).powi(2);
        let cost: f64 = (a+b).sqrt();
        println!("Cost is: {}", &cost);
        return cost;
    }
}