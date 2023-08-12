use std::fs::File;
use std::io::Write;
use crate::space::Point;


use petgraph::dot::{Dot};
use petgraph::{Undirected};
use petgraph::graph::{Graph, NodeIndex};

pub fn is_edge_already_in_graph(graph:&Graph<NodeIndex, f64, Undirected>, begin:NodeIndex, end:NodeIndex) -> bool {
    graph.find_edge(begin, end).is_some()
}

pub fn write_graph_to_file(graph:&Graph<Point<f64>, f64, Undirected>, path:&str) {
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

pub fn print_graph(graph:&Graph<Point<f64>, f64, Undirected>) {
    println!("{:?}", Dot::with_config(graph, &[]));
}
