use crate::node::Node2D;

/// Every Custom Optimizer needs to be based on this trait.
pub trait Optimizer {
    /// Returns a vector of triplets. Every consists of a start-node, end-node, and the calculated edge weight. Batch-wise weight calculation allows the Optimizer to use parallelism. 
    /// 
    /// ## Arguments
    ///     A batch of edges on which he cost needs to be returned. A single edge is presented a pair of start-node and end-node. The batch is represented as a vector of pairs / edges.
    fn get_edge_weight(&self, batch_edges: Vec<(Node2D, Node2D)>) -> Vec<(Node2D, Node2D, f64)>;

    /// The init function allows the Optimizer to execute code before running. This function is called only once and before all the other functions are called. This allows setup function like reading a file or connecting to a Database. 
    fn init(&mut self) -> bool;
}


/// Simple Optimizer used for examples and testing.
#[derive(Debug, Copy, Clone)]
pub struct DefaultOptimizer;
impl Optimizer for DefaultOptimizer {

    // Cost is based on the distance in 2D. Which is basically just Pythagoras.
    fn get_edge_weight(&self, batch_edges: Vec<(Node2D, Node2D)>) -> Vec<(Node2D, Node2D, f64)> {
        let mut costs: Vec<(Node2D, Node2D, f64)> = Vec::new();
        for edge in batch_edges {
            let (begin, end) = edge;
            let a = (begin.x - end.x).powi(2);
            let b: f64 = (begin.y - end.y).powi(2);
            let cost: f64 = (a+b).sqrt();
            costs.push((begin, end, cost));
        }
        return costs;
    }

    /// Does not do anything. Returns always true without any condition. 
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

    #[test]
    fn test_default_edge_weight_x() {
        let optimizer = DefaultOptimizer;
        let a = Node2D::new(0f64, 0f64);
        let b = Node2D::new(1f64, 0f64);

        let cost: f64 = optimizer.get_edge_weight(vec![(a, b)]).first().unwrap().2;
        assert_eq!(1f64, cost);
    }

    #[test]
    fn test_default_edge_weight_y() {
        let optimizer = DefaultOptimizer;
        let a = Node2D::new(0f64, 0f64);
        let b = Node2D::new(0f64, 1f64);

        let cost: f64 = optimizer.get_edge_weight(vec![(a, b)]).first().unwrap().2;
        assert_eq!(1f64, cost);
    }
  
}