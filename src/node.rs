use std::fmt;

use petgraph::graph::NodeIndex;
use geo_types::{coord, Line, Point};
use geo_types::*;
use wkt::ToWkt;

/// Node in 2D with an X-Coordinate, Y-Coordinate and an index. The index corresponds with the index inside the planner's graph. 
#[derive(Debug, Copy)]
pub struct Node2D {
    pub x: f64,
    pub y: f64,
    pub idx: usize,
}

impl Clone for Node2D {
    fn clone(&self) -> Self {
        return Node2D{x:self.x, y:self.y, idx:self.idx};
    }
}

impl fmt::Display for Node2D {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // WKT: POINT(X Y)
        write!(f, "POINT({} {})", self.x, self.y)
    }
}

impl Node2D {
    pub fn new(x: f64, y: f64) -> Self {
        let idx: usize = 0;
        return Node2D { x: x, y: y, idx: idx }
    }

    pub fn new_index(x: f64, y: f64, idx: usize) -> Self {
        return Node2D { x: x, y: y, idx: idx };
    }

    pub fn from_graph_node(index: NodeIndex) -> Self {
        return Node2D { x: 0f64, y: 0f64, idx: index.index() };
    }

    pub fn get_index_type(&self) -> NodeIndex {
        let index: NodeIndex = NodeIndex::new(self.idx);
        return index;
    }

    pub fn get_wkt(&self) -> String {
        let point = Point::new(self.x, self.y);
        return  point.wkt_string();
    }

    pub fn get_line_wkt(&self, other: &Node2D) -> String {
        let line = Line::new(coord! { x: self.x, y: self.y }, coord! { x: other.x, y: other.y });
        let l = Geometry::Line(line);
        return l.wkt_string();

    }
}

#[cfg(test)]
mod tests {
    use super::Node2D;
    use petgraph::graph::{NodeIndex};

    #[test]
    fn test_new() {
        let node = Node2D::new(2f64, 3f64);
        assert_eq!(2f64, node.x);
        assert_eq!(3f64, node.y);
        assert_eq!(0usize, node.idx);

    }

    #[test]
    fn test_new_index() {
        let node: Node2D = Node2D::new_index(1f64, 2f64, 3usize);
        assert_eq!(1f64, node.x);
        assert_eq!(2f64, node.y);
        assert_eq!(3usize, node.idx);
    }

    #[test]
    fn test_from_graph_node() {
        let node_index: NodeIndex = NodeIndex::new(1usize);
        let node: Node2D = Node2D::from_graph_node(node_index);
        assert_eq!(0f64, node.x);
        assert_eq!(0f64, node.y);
        assert_eq!(1usize, node.idx);
    }

    #[test]
    fn test_get_index_type() {
        let node: Node2D = Node2D { x: 0f64, y: 1f64, idx: 2usize };
        let index: NodeIndex = node.get_index_type();
        assert_eq!(NodeIndex::new(2usize), index);
    }

    #[test]
    fn test_get_wkt() {
        let node: Node2D = Node2D { x: 0f64, y: 1f64, idx: 2usize };
        let expected: &str = "POINT(0 1)";
        assert_eq!(expected, node.get_wkt());
    }

    #[test]
    fn test_get_line_wkt () {
        let node: Node2D = Node2D { x: 0f64, y: 1f64, idx: 2usize };
        let other: Node2D = Node2D { x: 2f64, y: 3f64, idx: 7usize };
        let expected: &str = "LINESTRING(0 1,2 3)";
        assert_eq!(expected, node.get_line_wkt(&other));
    }
}