use std::fs::File;
use std::io::Write;
use geo_types::Point;

use petgraph::dot::{Dot};
use petgraph::{Undirected};
use petgraph::graph::{Graph, NodeIndex};

use crate::node::Node2D;

pub fn insert_node_in_graph(graph: &mut Graph<Node2D, f64, Undirected>, node: &mut Node2D) {
    let index: usize = graph.add_node(*node).index();
    node.idx = index;
}

pub fn insert_edge_in_graph(graph: &mut Graph<Node2D, f64, Undirected>, begin: &Node2D, end_index: NodeIndex, weight:f64) {
    let begin_index: NodeIndex<u32> = NodeIndex::new(begin.idx);
    if begin_index == end_index { // do not insert edge from a node to itself
        return;
    }
    graph.add_edge(begin_index, end_index, weight);

}

pub fn is_edge_already_in_graph(graph:&Graph<Node2D, f64, Undirected>, begin:&Node2D, end:NodeIndex) -> bool {
    match graph.find_edge(begin.get_index_type(), end) {
        None => return false,
        Some(_) => return true,
    }
}

pub fn is_node_already_in_graph(_graph:&Graph<Node2D, f64, Undirected>, _node:&Node2D) -> bool {
    // TODO: Not implemented. Cannot find nodes by their weight.
    return false;
}

pub fn write_graph_to_file(graph:&Graph<Point, f64, Undirected>, path:&str) {
    let mut f = File::create(path).unwrap();
    let output = format!("{:?}", Dot::with_config(&graph, &[]));
    let res = f.write_all(&output.as_bytes());
    match res {
        Err(_) => println!("Could not write the graph to file!"),
        Ok(_) => println!("Write graph to file: {}", path),
    }
}

pub fn print_graph(graph:&Graph<Point, f64, Undirected>) {
    println!("{:?}", Dot::with_config(graph, &[]));
}
