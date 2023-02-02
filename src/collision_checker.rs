use petgraph::graph::NodeIndex;
use crate::node::Node2D;

/// CollisionChecker to implement custom Collision checkers.
pub trait CollisionChecker {
    /// Is run only once and before any checks are done. Can be used to read a file or database. 
    fn init(&self) -> bool;

    /// Returns a Vector of pairs collision_free edges. Colliding edges are not returned. 
    /// - Node2D: Node that was checked
    fn check_nodes(&self, nodes: Vec<Node2D>) -> Vec<Node2D>;

    /// Returns a Vector of collision free nodes. Colliding nodes are not returned. 
    /// - Node2D: Start-Node of edge that was checked.
    /// - Node2D: End-Node of edge that was checked.
    fn check_edge(&self, node: Node2D, end: NodeIndex) -> bool;
}

#[derive(Copy, Clone, Debug)]
pub struct NaiveCollisionChecker {}

impl NaiveCollisionChecker {
    fn new() -> Box<dyn CollisionChecker> {
        return Box::new(NaiveCollisionChecker{});
    }
}

impl CollisionChecker for NaiveCollisionChecker {
    fn init(&self) -> bool {
        return true;
    }
    
    fn check_edge(&self, _node: Node2D, _end: NodeIndex) -> bool {
        return true;
}

    fn check_nodes(&self, nodes: Vec<Node2D>) -> Vec<Node2D> {
        let mut result: Vec<Node2D> = Vec::new();
        for node in nodes {
            if true {
                result.push(node);
            }
        }
        return result;
    }
}