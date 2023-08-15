use crate::space::Point;
use crate::types::SpaceContinuous;
use std::fs::File;
use std::io::Write;

use num::Signed;
use petgraph::dot::Dot;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;

pub fn is_edge_already_in_graph<T: Copy + Clone + Signed + std::fmt::Debug>(
    graph: &Graph<NodeIndex, T, Undirected>,
    begin: NodeIndex,
    end: NodeIndex,
) -> bool {
    graph.find_edge(begin, end).is_some()
}

pub fn write_graph_to_file<T: SpaceContinuous>(graph: &Graph<Point<T>, T, Undirected>, path: &str) {
    let output = format!("{:?}", Dot::with_config(&graph, &[]));
    let mut file = match File::create(path) {
        Ok(file) => file,
        Err(_) => {
            println!("Could not open file at: {path}");
            return;
        }
    };

    match file.write_all(output.as_bytes()) {
        Err(_) => println!("Could not write the solution path to file! -> {path}"),
        Ok(_) => println!("Write graph to file: {path}"),
    };
}

pub fn print_graph<T: SpaceContinuous>(graph: &Graph<Point<T>, T, Undirected>) {
    println!("{:?}", Dot::with_config(graph, &[]));
}
