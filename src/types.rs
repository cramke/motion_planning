use num::{Bounded, Signed, Float};
use rand::distributions::uniform::SampleUniform;
use std::ops::{Add, Mul, Sub};

pub trait SpaceContinuous:
    Default
    + Copy
    + Clone
    + std::fmt::Debug
    + std::fmt::Display
    + ToString
    + PartialEq
    + PartialOrd
    + Signed
    + Bounded
    + Sub<Self, Output = Self>
    + Mul<Self, Output = Self>
    + Add<Self, Output = Self>
    + SampleUniform
    + Sized
    + Float
{
    const MAX: Self;
    const EPSILON: Self;
    const DEFAULT: Self;
}

impl SpaceContinuous for f64 {
    const MAX: Self = f64::MAX;
    const EPSILON: Self = f64::EPSILON;
    const DEFAULT: Self = 0f64;
}
impl SpaceContinuous for f32 {
    const MAX: Self = f32::MAX;
    const EPSILON: Self = f32::EPSILON;
    const DEFAULT: Self = 0f32;
}
