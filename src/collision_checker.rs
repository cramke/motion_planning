use std::{ops::{Sub, Add, Mul}, marker::PhantomData};

use num::{Bounded, Signed};
use rand::distributions::uniform::SampleUniform;

use crate::space::Point;


/// CollisionChecker to implement custom Collision checkers.
pub trait CollisionChecker<T: PartialOrd + SampleUniform + Sub + Add + Mul + Bounded + Copy + Signed + std::fmt::Debug> {
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
#[allow(unused)]
pub struct NaiveCollisionChecker<T: PartialOrd + SampleUniform + Sub + Add + Mul + Bounded + Copy + Signed + std::fmt::Debug> {
    pub phantom:PhantomData<T>
}

/// Does not check any collisions and always returns no collision (false)
impl<T: PartialOrd + SampleUniform + Sub + Add + Mul + Bounded + Copy + Signed + std::fmt::Debug + 'static> NaiveCollisionChecker<T> {
    pub fn new_box() -> Box<dyn CollisionChecker<T>> {
        Box::new(NaiveCollisionChecker{phantom: PhantomData})
    }
}


impl<T: PartialOrd + SampleUniform + Sub + Add + Mul + Bounded + Copy + Signed + std::fmt::Debug> CollisionChecker<T> for NaiveCollisionChecker<T> {
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
    use std::marker::PhantomData;

    use crate::space::Point;


    use super::{NaiveCollisionChecker, CollisionChecker};

    #[test]
    fn test_naive_init() {
        let cc: NaiveCollisionChecker<f64> = NaiveCollisionChecker{phantom: PhantomData};
        let result = cc.init();
        assert!(result);
    }

    #[test]
    fn test_naive_node() {
        let cc = NaiveCollisionChecker{phantom: PhantomData};
        let p1: &Point<f64> = &Point{x:1.0, y:2.0};
        let p2: &Point<f64> = &Point{x:1.0, y:2.0};


        let result = cc.is_edge_colliding(p1, p2);
        assert!(!result);
    }

    #[test]
    fn test_naive_edge() {
        let cc = NaiveCollisionChecker{phantom: PhantomData};
        let p1: &Point<f64> = &Point{x:1.0, y:2.0};
        let result: bool = cc.is_node_colliding(p1);
        assert!(!result);
    }
}