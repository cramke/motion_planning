use geo_types::Point;

/// CollisionChecker to implement custom Collision checkers.
pub trait CollisionChecker {
    /// Is run only once and before any checks are done. Can be used to read a file or database. 
    fn init(&self) -> bool;

    /// Returns a Vector of pairs collision_free edges. Colliding edges are not returned. 
    /// - Node2D: Node that was checked
    fn is_node_colliding(&self, node: &Point) -> bool;

    /// Returns a Vector of collision free nodes. Colliding nodes are not returned. 
    /// - Node2D: Start-Node of edge that was checked.
    /// - Node2D: End-Node of edge that was checked.
    fn is_edge_colliding(&self, node: &Point, end: &Point) -> bool;
}

#[derive(Copy, Clone, Debug)]
pub struct NaiveCollisionChecker {}

impl NaiveCollisionChecker {
    pub fn new() -> Box<dyn CollisionChecker> {
        return Box::new(NaiveCollisionChecker{});
    }
}

impl CollisionChecker for NaiveCollisionChecker {
    fn init(&self) -> bool {
        return true;
    }
    
    fn is_edge_colliding(&self, _node: &Point, _end: &Point) -> bool {
        return false;
}

    fn is_node_colliding(&self, _node: &Point) -> bool {
        return false;
    }
}