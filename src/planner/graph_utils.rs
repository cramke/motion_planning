use petgraph::{Undirected};
use petgraph::graph::{Graph, NodeIndex};

use crate::node::Node2D;

pub fn insert_node_in_graph(graph: &mut Graph<Node2D, f64, Undirected>, node: &mut Node2D) {
    let index: usize = graph.add_node(*node).index();
    node.idx = index;
}

pub fn insert_edge_in_graph(graph: &mut Graph<Node2D, f64, Undirected>, begin: &Node2D, end: NodeIndex, weight:f64) {
    let a: NodeIndex<u32> = NodeIndex::new(begin.idx);
    if a == end { // do not insert edge from a node to itself
        return;
    }
    graph.add_edge(a, end, weight);

}

pub fn is_edge_already_in_graph(graph:&Graph<Node2D, f64, Undirected>, begin:&Node2D, end:NodeIndex) -> bool {
    match graph.find_edge(begin.get_index_type(), end) {
        None => return false,
        Some(_) => return true,
    }
}

pub fn is_node_already_in_graph(graph:&Graph<Node2D, f64, Undirected>, node:&Node2D) -> bool {
    // TODO: Not implemented. Cannot find nodes by their weight.
    return false;
}
