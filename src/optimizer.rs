use geo_types::Point;
use geo::algorithm::euclidean_distance::EuclideanDistance;

/// Every Custom Optimizer needs to be based on this trait.
pub trait Optimizer {
    /// Returns a vector of triplets. Every consists of a start-node, end-node, and the calculated edge weight. Batch-wise weight calculation allows the Optimizer to use parallelism. 
    /// 
    /// ## Arguments
    /// A batch of edges on which he cost needs to be returned. A single edge is presented a pair of start-node and end-node. The batch is represented as a vector of pairs / edges.
    fn get_edge_weight(&self, begin: Point, end: Point) -> (Point, Point, f64);

    /// The init function allows the Optimizer to execute code before running. This function is called only once and before all the other functions are called. This allows setup function like reading a file or connecting to a Database. 
    fn init(&mut self) -> bool;
}


/// Simple Optimizer used for examples and testing.
#[derive(Debug, Copy, Clone)]
pub struct DefaultOptimizer;

impl DefaultOptimizer {
    pub fn new_box() -> Box<dyn Optimizer> {
        Box::new(DefaultOptimizer{})
    }
}

impl Optimizer for DefaultOptimizer {
    // Cost is based on the distance in 2D. Which is basically just Pythagoras.
    fn get_edge_weight(&self, begin: Point, end: Point) -> (Point, Point, f64) {
        let cost: f64 = begin.euclidean_distance(&end);
        return (begin, end, cost);
    }

    /// Does not do anything. Returns always true without any condition. 
    fn init(&mut self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::{Optimizer, DefaultOptimizer};

    #[test]
    fn test_default_init() {
        let mut optimizer = DefaultOptimizer;
        assert_eq!(true, optimizer.init());
    }

    #[test]
    fn test_default_edge_weight_x() {
        use geo_types::Point;

        let optimizer = DefaultOptimizer;
        let a = Point::new(0f64, 0f64);
        let b = Point::new(1f64, 0f64);

        let cost: f64 = optimizer.get_edge_weight(a, b).2;
        assert_eq!(1f64, cost);
    }

    #[test]
    fn test_default_edge_weight_y() {
        use geo_types::Point;

        let optimizer = DefaultOptimizer;
        let a = Point::new(0f64, 0f64);
        let b = Point::new(0f64, 1f64);

        let cost: f64 = optimizer.get_edge_weight(a, b).2;
        assert_eq!(1f64, cost);
    }
  
}