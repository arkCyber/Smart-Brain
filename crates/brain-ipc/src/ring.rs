//! 环形缓冲（Ring Buffer）。
//!
//! 关键设计：底层是**预分配**的 `Vec<Option<T>>`，`push` 只把元素“移动”进
//! 已有槽位并推进写指针，**全程零内存分配**；达到容量后覆盖最旧数据
//! （适合雷达/相机这类“只保留最近 N 帧”的语义）。

use std::sync::Mutex;

/// 环形缓冲错误。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingError {
    /// 空缓冲。
    Empty,
    /// 容量非法（0）。
    InvalidCapacity,
}

/// 预分配的环形缓冲（单线程视图）。
pub struct FixedRingBuffer<T> {
    buf: Vec<Option<T>>,
    cap: usize,
    /// 写指针（下一个写入位置）。
    head: usize,
    /// 元素个数。
    len: usize,
}

impl<T> FixedRingBuffer<T> {
    /// 创建容量为 `cap` 的环形缓冲（cap 必须 > 0）。
    pub fn new(cap: usize) -> Result<Self, RingError> {
        if cap == 0 {
            return Err(RingError::InvalidCapacity);
        }
        Ok(Self {
            buf: std::iter::repeat_with(|| None).take(cap).collect(),
            cap,
            head: 0,
            len: 0,
        })
    }

    /// 写入一个元素。若已满，覆盖最旧元素并返回被覆盖的值。
    pub fn push(&mut self, item: T) -> Option<T> {
        let evicted = self.buf[self.head].take();
        self.buf[self.head] = Some(item);
        self.head = (self.head + 1) % self.cap;
        if self.len < self.cap {
            self.len += 1;
        }
        evicted
    }

    /// 弹出最旧的元素（FIFO 语义）。
    pub fn pop_oldest(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        // 最旧元素位于 `head - len`（模 cap）。
        let idx = (self.head + self.cap - self.len) % self.cap;
        self.len -= 1;
        self.buf[idx].take()
    }

    /// 读取第 `i` 个最旧的元素（`i < len`）。
    pub fn get(&self, i: usize) -> Option<&T> {
        if i >= self.len {
            return None;
        }
        let idx = (self.head + self.cap - self.len + i) % self.cap;
        self.buf[idx].as_ref()
    }

    /// 从最旧到最新依次迭代元素。
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        (0..self.len).filter_map(move |i| self.get(i))
    }

    /// 元素个数。
    pub fn len(&self) -> usize {
        self.len
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 是否已满。
    pub fn is_full(&self) -> bool {
        self.len == self.cap
    }

    /// 容量。
    pub fn capacity(&self) -> usize {
        self.cap
    }

    /// 清空。
    pub fn clear(&mut self) {
        for slot in self.buf.iter_mut() {
            *slot = None;
        }
        self.head = 0;
        self.len = 0;
    }
}

/// 线程安全共享环形缓冲：内部用 `Mutex` 保护，便于在多线程/多循环间共享。
pub struct SharedRing<T> {
    inner: Mutex<FixedRingBuffer<T>>,
}

impl<T> SharedRing<T> {
    pub fn new(cap: usize) -> Result<Self, RingError> {
        Ok(Self {
            inner: Mutex::new(FixedRingBuffer::new(cap)?),
        })
    }

    pub fn push(&self, item: T) -> Option<T> {
        self.inner.lock().ok().and_then(|mut g| g.push(item))
    }

    pub fn pop_oldest(&self) -> Option<T> {
        self.inner.lock().ok().and_then(|mut g| g.pop_oldest())
    }

    pub fn len(&self) -> usize {
        self.inner.lock().map(|g| g.len()).unwrap_or(0)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn capacity(&self) -> usize {
        self.inner.lock().map(|g| g.capacity()).unwrap_or(0)
    }

    pub fn is_full(&self) -> bool {
        self.inner.lock().map(|g| g.is_full()).unwrap_or(false)
    }

    /// 读取第 `i` 个最旧的元素（需要 `T: Clone`）。
    pub fn get(&self, i: usize) -> Option<T>
    where
        T: Clone,
    {
        self.inner.lock().ok().and_then(|g| g.get(i).cloned())
    }

    /// 从最旧到最新依次迭代元素。
    pub fn iter(&self) -> Vec<T>
    where
        T: Clone,
    {
        self.inner
            .lock()
            .map(|g| g.iter().cloned().collect())
            .unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_order() {
        let mut r = FixedRingBuffer::new(3).unwrap();
        r.push(1);
        r.push(2);
        r.push(3);
        assert_eq!(r.pop_oldest(), Some(1));
        assert_eq!(r.pop_oldest(), Some(2));
        assert_eq!(r.pop_oldest(), Some(3));
        assert!(r.is_empty());
    }

    #[test]
    fn overwrite_oldest() {
        let mut r = FixedRingBuffer::new(2).unwrap();
        r.push(1);
        r.push(2);
        // 溢出：覆盖最旧的 1。
        let evicted = r.push(3);
        assert_eq!(evicted, Some(1));
        assert_eq!(r.len(), 2);
        // 现在保留的是 2,3。
        assert_eq!(r.get(0), Some(&2));
        assert_eq!(r.get(1), Some(&3));
    }

    #[test]
    fn invalid_capacity() {
        assert!(matches!(
            FixedRingBuffer::<u8>::new(0),
            Err(RingError::InvalidCapacity)
        ));
    }

    #[test]
    fn iter_yields_oldest_to_newest() {
        let mut r = FixedRingBuffer::new(3).unwrap();
        r.push(10);
        r.push(20);
        r.push(30);
        // 覆盖最旧的 10。
        r.push(40);
        let got: Vec<i32> = r.iter().copied().collect();
        assert_eq!(got, vec![20, 30, 40]);
    }

    #[test]
    fn shared_ring_exposes_read_api() {
        let ring = SharedRing::new(3).unwrap();
        assert!(ring.is_empty());
        assert_eq!(ring.capacity(), 3);
        ring.push("a".to_string());
        ring.push("b".to_string());
        assert_eq!(ring.len(), 2);
        assert_eq!(ring.get(0), Some("a".to_string()));
        assert_eq!(ring.get(1), Some("b".to_string()));
        assert_eq!(ring.iter(), vec!["a".to_string(), "b".to_string()]);
        assert_eq!(ring.pop_oldest(), Some("a".to_string()));
    }

    #[test]
    fn shared_ring_full_and_evict() {
        let ring = SharedRing::new(2).unwrap();
        ring.push(1);
        ring.push(2);
        assert!(ring.is_full());
        // 覆盖最旧的 1。
        let evicted = ring.push(3);
        assert_eq!(evicted, Some(1));
        assert_eq!(ring.iter(), vec![2, 3]);
    }
}
