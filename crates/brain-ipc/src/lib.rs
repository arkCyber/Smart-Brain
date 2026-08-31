//! `brain-ipc` — 本地极低延迟传输层。
//!
//! 对应参考架构第 3 层：用于在模块间传输百万级点云与高帧率图像。提供
//! **预分配、零内存分配**的环形缓冲（覆盖式写入），以及一个线程安全的
//! 共享封装，适合在感知/避障主循环间搬运高频传感器帧。

pub mod ring;

pub use ring::{FixedRingBuffer, RingError, SharedRing};
