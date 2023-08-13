use std::ops::{Sub, Mul, Add};
use num::{Signed, Bounded, Float};
use rand::distributions::uniform::SampleUniform;

pub trait Metric2D: 
    Default + 
    Copy + Clone + 
    std::fmt::Debug + ToString + 
    PartialEq + PartialOrd + 
    Signed +
    Bounded + Float +
    Sub<Self, Output = Self> + Mul<Self, Output = Self>+ Add<Self, Output = Self> + 
    Default +
    SampleUniform
{
}

impl Metric2D for f64 {}
impl Metric2D for f32 {}