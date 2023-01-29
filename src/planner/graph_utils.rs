use std::fs::File;
use std::io::Write;

use petgraph::dot::{Dot, Config};
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

pub fn write_graph_to_file(graph:&Graph<Node2D, f64, Undirected>) {
    let mut f = File::create("./examples/example1/example1.dot").unwrap();
    let output = format!("{}", Dot::with_config(&graph, &[]));
    f.write_all(&output.as_bytes());
}

pub fn print_graph(graph:&Graph<Node2D, f64, Undirected>) {
    println!("{:?}", Dot::with_config(graph, &[]));
}
