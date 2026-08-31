//! 目标检测的后处理：边界框解码与 NMS（非极大值抑制）。
//!
//! 与推理后端解耦的纯函数，便于对 ONNX 输出做标准化的 NMS，也便于单元测试。
//! 解码假设 YOLOv8 风格的输出：张量形状 `[1, 4 + nc, num_anchors]`，
//! 每个 anchor 依次为 `[cx, cy, w, h, class0, class1, ...]`（归一化坐标）。

use brain_core::Result;

/// 解码后的一个检测框（像素/归一化坐标均可，保持同一参考系即可）。
#[derive(Debug, Clone, PartialEq)]
pub struct DetectionBox {
    /// 类别索引。
    pub class: usize,
    /// 置信度 0..1。
    pub score: f32,
    /// 左上角 x。
    pub x1: f32,
    /// 左上角 y。
    pub y1: f32,
    /// 右下角 x。
    pub x2: f32,
    /// 右下角 y。
    pub y2: f32,
}

impl DetectionBox {
    /// 用中心点/宽高构造。
    pub fn from_cxcywh(class: usize, score: f32, cx: f32, cy: f32, w: f32, h: f32) -> Self {
        Self {
            class,
            score,
            x1: cx - w / 2.0,
            y1: cy - h / 2.0,
            x2: cx + w / 2.0,
            y2: cy + h / 2.0,
        }
    }

    /// 面积。
    pub fn area(&self) -> f32 {
        (self.x2 - self.x1).max(0.0) * (self.y2 - self.y1).max(0.0)
    }

    /// 与另一个框的 IoU（交并比）。
    pub fn iou(&self, o: &DetectionBox) -> f32 {
        let ix1 = self.x1.max(o.x1);
        let iy1 = self.y1.max(o.y1);
        let ix2 = self.x2.min(o.x2);
        let iy2 = self.y2.min(o.y2);
        let inter = (ix2 - ix1).max(0.0) * (iy2 - iy1).max(0.0);
        let union = self.area() + o.area() - inter;
        if union <= 0.0 {
            0.0
        } else {
            inter / union
        }
    }
}

/// 按置信度降序执行**类别感知**的贪心 NMS。
///
/// 仅抑制“同类别且 IoU 超过阈值”的框，避免不同类别的重叠目标互相误抑制。
/// 返回保留下来的框（保持 score 降序），不修改输入。
pub fn nms(boxes: Vec<DetectionBox>, iou_threshold: f32) -> Vec<DetectionBox> {
    let mut ordered: Vec<(usize, DetectionBox)> = boxes.into_iter().enumerate().collect();
    ordered.sort_by(|(_, a), (_, b)| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut keep: Vec<DetectionBox> = Vec::new();
    let mut suppressed = vec![false; ordered.len()];
    for i in 0..ordered.len() {
        if suppressed[i] {
            continue;
        }
        let (_, box_i) = &ordered[i];
        keep.push(box_i.clone());
        for j in (i + 1)..ordered.len() {
            let (_, box_j) = &ordered[j];
            // 仅抑制同类别且高重叠的框。
            if !suppressed[j] && box_i.class == box_j.class && box_i.iou(box_j) > iou_threshold {
                suppressed[j] = true;
            }
        }
    }
    keep
}

/// 从 YOLOv8 输出张量解码出候选框（按类别取最高分）。
///
/// - `data`：行优先的 `[1, 4 + nc, num_anchors]` 扁平数组。
/// - `nc`：类别数。
/// - 返回已按类别 max-score 解码、置信度 >= `conf_threshold` 的候选框。
pub fn decode_yolov8(data: &[f32], nc: usize, conf_threshold: f32) -> Result<Vec<DetectionBox>> {
    let stride = 4 + nc;
    if stride == 0 || !data.len().is_multiple_of(stride) {
        return Err(brain_core::BrainError::Inference(format!(
            "yolov8 output len {} not divisible by 4+nc={}",
            data.len(),
            stride
        )));
    }
    let num_anchors = data.len() / stride;
    let mut dets = Vec::new();
    for a in 0..num_anchors {
        let row = &data[a * stride..(a + 1) * stride];
        let cx = row[0];
        let cy = row[1];
        let w = row[2];
        let h = row[3];
        let mut best_class = 0usize;
        let mut best_score = f32::NEG_INFINITY;
        for c in 0..nc {
            let s = row[4 + c];
            if s > best_score {
                best_score = s;
                best_class = c;
            }
        }
        if best_score >= conf_threshold && w > 0.0 && h > 0.0 {
            dets.push(DetectionBox::from_cxcywh(
                best_class, best_score, cx, cy, w, h,
            ));
        }
    }
    Ok(dets)
}

/// 便捷：解码 + NMS 一步完成，返回保留下来的框。
pub fn detect_and_nms(
    data: &[f32],
    nc: usize,
    conf_threshold: f32,
    iou_threshold: f32,
) -> Result<Vec<DetectionBox>> {
    let candidates = decode_yolov8(data, nc, conf_threshold)?;
    Ok(nms(candidates, iou_threshold))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iou_identical_is_one() {
        let a = DetectionBox::from_cxcywh(0, 0.9, 0.5, 0.5, 0.2, 0.2);
        let b = DetectionBox::from_cxcywh(0, 0.5, 0.5, 0.5, 0.2, 0.2);
        assert!((a.iou(&b) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn iou_disjoint_is_zero() {
        let a = DetectionBox::from_cxcywh(0, 0.9, 0.1, 0.1, 0.2, 0.2);
        let b = DetectionBox::from_cxcywh(0, 0.5, 0.9, 0.9, 0.2, 0.2);
        assert_eq!(a.iou(&b), 0.0);
    }

    #[test]
    fn nms_keeps_one_of_overlapping() {
        let boxes = vec![
            DetectionBox::from_cxcywh(0, 0.95, 0.5, 0.5, 0.4, 0.4),
            DetectionBox::from_cxcywh(0, 0.60, 0.51, 0.5, 0.4, 0.4),
        ];
        let kept = nms(boxes, 0.45);
        assert_eq!(kept.len(), 1);
        assert!((kept[0].score - 0.95).abs() < 1e-5);
    }

    #[test]
    fn nms_keeps_separate_classes() {
        let boxes = vec![
            DetectionBox::from_cxcywh(0, 0.9, 0.5, 0.5, 0.3, 0.3),
            DetectionBox::from_cxcywh(1, 0.9, 0.5, 0.5, 0.3, 0.3),
        ];
        assert_eq!(nms(boxes, 0.45).len(), 2);
    }

    #[test]
    fn decode_yolov8_filters_by_conf() {
        let mut data = vec![0.5, 0.5, 0.2, 0.2, 0.9, 0.7, 0.7, 0.2, 0.2, 0.1];
        let dets = decode_yolov8(&data, 1, 0.5).unwrap();
        assert_eq!(dets.len(), 1);
        assert_eq!(dets[0].class, 0);
        assert!((dets[0].score - 0.9).abs() < 1e-5);

        data[4] = 0.4; // 第一个 anchor 低于阈值
        assert_eq!(decode_yolov8(&data, 1, 0.5).unwrap().len(), 0);
    }

    #[test]
    fn decode_rejects_bad_shape() {
        let data = vec![0.0f32; 7]; // 4+nc 无法整除
        assert!(decode_yolov8(&data, 2, 0.5).is_err());
    }

    #[test]
    fn detect_and_nms_end_to_end() {
        let mut data = Vec::new();
        for (cx, cy, s) in [(0.5f32, 0.5f32, 0.9f32), (0.51, 0.5, 0.6), (0.9, 0.9, 0.3)] {
            data.extend_from_slice(&[cx, cy, 0.3, 0.3, s]);
        }
        let kept = detect_and_nms(&data, 1, 0.5, 0.45).unwrap();
        assert_eq!(kept.len(), 1);
        assert!((kept[0].score - 0.9).abs() < 1e-5);
    }
}
