use num::{Bounded, Signed};
use rand::distributions::uniform::SampleUniform;
use std::ops::{Add, Mul, Sub};

pub trait Metric2D:
    Default
    + Copy
    + Clone
    + std::fmt::Debug
    + ToString
    + PartialEq
    + PartialOrd
    + Signed
    + Bounded + Sub<Self, Output = Self>
    + Mul<Self, Output = Self>
    + Add<Self, Output = Self>
    + SampleUniform
{
    const MAX: Self;
    const EPSILON: Self;
}

impl Metric2D for f64 {
    const MAX: Self = f64::MAX;
    const EPSILON: Self = f64::EPSILON;
}
impl Metric2D for f32 {
    const MAX: Self = f32::MAX;
    const EPSILON: Self = f32::EPSILON;
}
