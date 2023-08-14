use crate::{space::Point, types::Metric2D};
use std::marker::PhantomData;

/// CollisionChecker to implement custom Collision checkers.
pub trait CollisionChecker<T: Metric2D> {
    /// Is run only once and before any checks are done. Can be used to read a file or database.
    fn init(&self) -> bool;

    /// Returns:
    /// - true: there is an collision
    /// - false: there is no collision
    fn is_node_colliding(&self, node: &Point<T>) -> bool;

    /// Returns:
    /// - true: there is an collision
    /// - false: there is no collision
    fn is_edge_colliding(&self, node: &Point<T>, end: &Point<T>) -> bool;
}

#[derive(Copy, Clone, Debug)]
pub struct NaiveCollisionChecker<T: Metric2D> {
    pub phantom: PhantomData<T>,
}

/// Does not check any collisions and always returns no collision (false)
impl<T: Metric2D + 'static> NaiveCollisionChecker<T> {
    pub fn new_box() -> Box<dyn CollisionChecker<T>> {
        Box::new(NaiveCollisionChecker {
            phantom: PhantomData,
        })
    }
}

impl<T: Metric2D> CollisionChecker<T> for NaiveCollisionChecker<T> {
    /// Does nothing
    /// Return
    ///     true: always
    fn init(&self) -> bool {
        true
    }

    /// Does nothing
    /// Return
    ///     false: always
    fn is_edge_colliding(&self, _node: &Point<T>, _end: &Point<T>) -> bool {
        false
    }

    /// Does nothing
    /// Return
    ///     false: always
    fn is_node_colliding(&self, _node: &Point<T>) -> bool {
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
        let cc: NaiveCollisionChecker<f64> = NaiveCollisionChecker {
            phantom: PhantomData,
        };
        let result = cc.init();
        assert!(result);
    }

    #[test]
    fn test_naive_node() {
        let cc: NaiveCollisionChecker<f64> = NaiveCollisionChecker {
            phantom: PhantomData,
        };
        let p1: &Point<f64> = &Point { x: 1.0, y: 2.0 };
        let p2: &Point<f64> = &Point { x: 1.0, y: 2.0 };

        let result = cc.is_edge_colliding(p1, p2);
        assert!(!result);
    }

    #[test]
    fn test_naive_edge() {
        let cc: NaiveCollisionChecker<f64> = NaiveCollisionChecker {
            phantom: PhantomData,
        };
        let p1: &Point<f64> = &Point { x: 1.0, y: 2.0 };
        let result: bool = cc.is_node_colliding(p1);
        assert!(!result);
    }
}
