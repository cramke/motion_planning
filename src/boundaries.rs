#[derive(Debug, Copy, Clone)]
pub struct Boundaries {
    pub x_lower: f64,
    pub x_upper: f64,
    pub y_lower: f64,
    pub y_upper: f64,
}

impl Boundaries {
    pub fn new(x_low: f64, x_up: f64, y_low: f64, y_up: f64) -> Self {
        return Boundaries { x_lower: x_low, x_upper: x_up, y_lower: y_low, y_upper: y_up };
    }
}

mod tests {

    #[test]
    fn test_boundaries_dummy() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        assert_eq!(0f64, bounds.x_lower);
        assert_eq!(1f64, bounds.x_upper);
        assert_eq!(2f64, bounds.y_lower);
        assert_eq!(3f64, bounds.y_upper);
    }
}