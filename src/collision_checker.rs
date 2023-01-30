use crate::node::Node2D;

/// CollisionChecker to implement custom Collision checkers.
pub trait CollisionChecker {
    /// Is run only once and before any checks are done. Can be used to read a file or database. 
    fn init(&self) -> bool;

    /// Returns a Vector of pairs with the node and the check result
    /// - Node2D: Node that was checked
    /// - bool: 
    ///     - true: If it is collision-free
    ///     - false: If there is a collision
    fn check_node(&self, nodes: Vec<Node2D>) -> Vec<(Node2D, bool)>;

    /// Returns a Vector of pairs with the node and the check result
    /// - Node2D: Start-Node of edge that was checked.
    /// - Node2D: End-Node of edge that was checked.
    /// - bool: 
    ///     - true: If it is collision-free
    ///     - false: If there is a collision
    fn check_edge(&self, edges: Vec<(Node2D, Node2D)>) -> Vec<(Node2D, Node2D, bool)>;
}