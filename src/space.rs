#[derive(Debug, Clone, Copy)]
pub struct Point<T> 
{
    pub x: T,
    pub y: T
}

impl PartialEq for Point<f64> {
    fn eq(&self, other: &Self) -> bool {
        let eq_x: bool = (self.x - other.x).abs() < f64::EPSILON;
        let eq_y: bool = (self.y - other.y).abs() < f64::EPSILON;
        eq_x && eq_y
    }
}

impl<T: ToString + std::ops::Sub<Output = T> + std::ops::Add<Output = T> + std::ops::Mul<Output = T> + Copy> Point<T> 
{
    pub fn to_wkt(&self) -> String {
        format!("POINT({} {})", self.x.to_string(), self.x.to_string())
    }

    pub fn euclidean_distance(&self, other: &Point<T>) -> T {
        ((self.x - other.x)*(self.x - other.x)) + ((self.y - other.y)*(self.y - other.y))
    }
}