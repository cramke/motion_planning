/*!
mpl
========

**mpl** is a **mo**tion **pla**nning **l**ibrary written with
the rust programming language.

The main idea is to easily usable with various problems in different domains. It also tries to make specific goals or planners independent and easily interchangeable.

*/

pub mod boundaries;
pub mod problem;
pub mod optimizer;
pub mod planner;
pub mod collision_checker;
pub mod space;
pub mod core;