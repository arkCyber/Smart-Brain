//! `brain-middleware` — 大脑内部的“神经网”（数据总线）。
//!
//! 提供类型安全的话题发布/订阅（类似 ROS2 topic / Zenoh key-expression），
//! 以及一个按名字索引的模块注册表。各模块通过总线解耦：感知模块发布
//! 检测结果，决策模块订阅之，无需互相知道对方的存在。

pub mod bus;

pub use bus::{BusMessage, DataBus, Topic};
