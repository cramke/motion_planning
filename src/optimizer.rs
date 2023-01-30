use crate::node::Node2D;

pub trait Optimizer {
    fn get_edge_weight(&self, batch_edges: Vec<(Node2D, Node2D)>) -> Vec<f64>;
    fn init(&mut self) -> bool;
}

#[derive(Debug, Copy, Clone)]
pub struct DefaultOptimizer;
impl Optimizer for DefaultOptimizer {

    fn get_edge_weight(&self, batch_edges: Vec<(Node2D, Node2D)>) -> Vec<f64> {
        let mut costs: Vec<f64> = Vec::new();
        for edge in batch_edges {
            let (begin, end) = edge;
            let a = (begin.x - end.x).powi(2);
            let b: f64 = (begin.y - end.y).powi(2);
            let cost: f64 = (a+b).sqrt();
            costs.push(cost);
        }
        return costs;
    }

    fn init(&mut self) -> bool {
        return true;
    }
}

#[cfg(test)]
mod tests {
    use crate::node::Node2D;
    use super::{Optimizer, DefaultOptimizer};

    #[test]
    fn test_default_init() {
        let mut optimizer = DefaultOptimizer;
        assert_eq!(true, optimizer.init());
    }
/*
    #[test]
    fn test_default_edge_weight_x() {
        let optimizer = DefaultOptimizer;
        let a = Node2D::new(0f64, 0f64);
        let b = Node2D::new(1f64, 0f64);

        assert_eq!(1f64, optimizer.get_edge_weight(&a, &b));
    }

    #[test]
    fn test_default_edge_weight_y() {
        let optimizer = DefaultOptimizer;
        let a = Node2D::new(0f64, 0f64);
        let b = Node2D::new(0f64, 1f64);

        assert_eq!(1f64, optimizer.get_edge_weight(&a, &b));
    }
*/
  
}