use crate::space::Point;
use std::fs::File;
use std::io::Write;

use num::Signed;
use petgraph::dot::Dot;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;

/// Returns true if there is an edge between the given nodes in the graph, false otherwise.
///
/// # Arguments
///
/// * `graph` - A reference to the graph to search for the edge.
/// * `begin` - The index of the beginning node of the edge.
/// * `end` - The index of the end node of the edge.
///
/// # Type parameters
///
/// * `T` - The weight type of the graph.
pub fn is_edge_already_in_graph<T: Copy + Clone + Signed + std::fmt::Debug>(
    graph: &Graph<NodeIndex, f64, Undirected>,
    begin: NodeIndex,
    end: NodeIndex,
) -> bool {
    graph.find_edge(begin, end).is_some()
}

/// Writes the given graph to a file at the specified path.
///
/// # Arguments
///
/// * `graph` - A reference to the graph to be written to file.
/// * `path` - The path to the file where the graph will be written.
pub fn write_graph_to_file(graph: &Graph<Point, f64, Undirected>, path: &str) {
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

/// Prints the graph using the `Dot` format.
///
/// # Arguments
///
/// * `graph` - A reference to the graph to be printed.
pub fn print_graph(graph: &Graph<Point, f64, Undirected>) {
    println!("{:?}", Dot::with_config(graph, &[]));
}
