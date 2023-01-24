use petgraph::graph::NodeIndex;

#[derive(Debug, Copy, Clone)]
pub struct Node2D {
    pub x: f64,
    pub y: f64,
    pub idx: usize,
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
}

#[cfg(test)]
mod tests {
    use super::Node2D;
    use petgraph::graph::NodeIndex;

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
}