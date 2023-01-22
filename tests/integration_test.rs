use prm;

#[test]
fn it_adds_two() {
    assert_eq!(4, 4);
}

#[test]
fn test_prm() {
    let node = prm::node::Node2D::new(0f64, 1f64);
    assert_eq!(0f64, node.x);
}