trait TerminationCritera {
    fn is_met(&self) -> bool;
}

pub struct BaseTerminationCriteria {
    max_size: usize,
}

impl BaseTerminationCriteria {
    pub fn new(max_size: usize) -> self {
        BaseTerminationCriteria { max_size }
    }
}

impl TerminationCritera for BaseTerminationCriteria {
    fn is_met(&self, graph: &Graph<Point, f64, Undirected>) -> bool{
        graph.node_count() >= self.max_size
    }
}