use crate::space::Point;
use std::marker::PhantomData;

/// CollisionChecker to implement custom Collision checkers.
pub trait CollisionChecker {
    /// Is run only once and before any checks are done. Can be used to read a file or database.
    fn init(&self) -> bool;

    /// Returns:
    /// - true: f64here is an collision
    /// - false: f64here is no collision
    fn is_node_colliding(&self, node: &Point) -> bool;

    /// Returns:
    /// - true: f64here is an collision
    /// - false: f64here is no collision
    fn is_edge_colliding(&self, node: &Point, end: &Point) -> bool;
}

#[derive(Copy, Clone, Debug)]
pub struct NaiveCollisionChecker {
    pub phantom: PhantomData<f64>,
}

/// Does not check any collisions and always returns no collision (false)
impl NaiveCollisionChecker {
    pub fn new_box() -> Box<dyn CollisionChecker> {
        Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        })
    }
}

impl CollisionChecker for NaiveCollisionChecker {
    /// Does nothing
    /// Return
    ///     true: always
    fn init(&self) -> bool {
        true
    }

    /// Does nothing
    /// Return
    ///     false: always
    fn is_edge_colliding(&self, _node: &Point, _end: &Point) -> bool {
        false
    }

    /// Does nothing
    /// Return
    ///     false: always
    fn is_node_colliding(&self, _node: &Point) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::{CollisionChecker, NaiveCollisionChecker};
    use crate::space::Point;
    use std::marker::PhantomData;

    #[test]
    fn test_naive_init() {
        let cc: NaiveCollisionChecker = NaiveCollisionChecker {
            phantom: PhantomData,
        };
        let result = cc.init();
        assert!(result);
    }

    #[test]
    fn test_naive_node() {
        let cc: NaiveCollisionChecker = NaiveCollisionChecker {
            phantom: PhantomData,
        };
        let p1: &Point = &Point::new(1.0, 2.0);
        let p2: &Point = &Point::new(1.0, 2.0);

        let result = cc.is_edge_colliding(p1, p2);
        assert!(!result);
    }

    #[test]
    fn test_naive_edge() {
        let cc: NaiveCollisionChecker = NaiveCollisionChecker {
            phantom: PhantomData,
        };
        let p1: &Point = &Point::new(1.0, 2.0);
        let result: bool = cc.is_node_colliding(p1);
        assert!(!result);
    }
}
