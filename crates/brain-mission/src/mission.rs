//! 任务模型与执行器。

use std::path::Path;

use brain_core::error::{BrainError, Result};
use brain_core::Vec3;
use brain_message::CommandTarget;
use serde::{Deserialize, Serialize};

/// 一个航点。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub sequence: u32,
    /// 北向（米）。
    pub north: f32,
    /// 东向（米）。
    pub east: f32,
    /// 高度（米，向上为正）。
    pub alt: f32,
    /// 到达容差（米）。
    pub accept_radius: f32,
}

/// 任务执行阶段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    NotStarted,
    InProgress,
    Completed,
    Aborted,
}

/// 一条航线任务。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mission {
    pub id: String,
    pub waypoints: Vec<Waypoint>,
}

impl Mission {
    /// 便捷构造：以首航点为起点。
    pub fn new(id: impl Into<String>, waypoints: Vec<Waypoint>) -> Self {
        Self {
            id: id.into(),
            waypoints,
        }
    }

    /// 校验任务：航点序号需连续且高度为正。
    pub fn validate(&self) -> Result<()> {
        if self.waypoints.is_empty() {
            return Err(BrainError::Mission("mission has no waypoints".into()));
        }
        for (i, wp) in self.waypoints.iter().enumerate() {
            if wp.sequence != i as u32 {
                return Err(BrainError::Mission(format!(
                    "waypoint sequence out of order at index {i}"
                )));
            }
            if wp.alt < 0.0 {
                return Err(BrainError::Mission(format!(
                    "waypoint {} has negative altitude",
                    wp.sequence
                )));
            }
        }
        Ok(())
    }

    /// 任务总路径长度（米，按航点顺序累计 3D 距离）。
    pub fn total_distance(&self) -> f32 {
        let mut d = 0.0f32;
        let mut prev: Option<Vec3> = None;
        for wp in &self.waypoints {
            let p = Vec3::new(wp.north, wp.east, wp.alt);
            if let Some(pr) = prev {
                d += p.sub(pr).norm();
            }
            prev = Some(p);
        }
        d
    }

    /// 序列化为 JSON 字符串。
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self)
            .map_err(|e| BrainError::Mission(format!("serialize: {e}")))
    }

    /// 从 JSON 字符串解析并校验。
    pub fn from_json(json: &str) -> Result<Self> {
        let m: Mission = serde_json::from_str(json)
            .map_err(|e| BrainError::Mission(format!("deserialize: {e}")))?;
        m.validate()?;
        Ok(m)
    }

    /// 保存到文件。
    pub fn save(&self, path: impl AsRef<Path>) -> Result<()> {
        let json = self.to_json()?;
        std::fs::write(path.as_ref(), json)
            .map_err(|e| BrainError::Mission(format!("write {}: {e}", path.as_ref().display())))
    }

    /// 从文件加载并校验。
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let json = std::fs::read_to_string(path.as_ref())
            .map_err(|e| BrainError::Mission(format!("read {}: {e}", path.as_ref().display())))?;
        Self::from_json(&json)
    }
}

/// 任务执行进度。
#[derive(Debug, Clone)]
pub struct MissionProgress {
    waypoints_completed: usize,
    total_waypoints: usize,
    distance_traveled: f32,
    total_distance: f32,
    phase: MissionPhase,
}

impl MissionProgress {
    fn new(total_waypoints: usize, total_distance: f32) -> Self {
        Self {
            waypoints_completed: 0,
            total_waypoints: total_waypoints.max(1),
            distance_traveled: 0.0,
            total_distance,
            phase: MissionPhase::NotStarted,
        }
    }

    /// 按航点计的完成百分比（0..100）。
    pub fn percent_complete(&self) -> f32 {
        (self.waypoints_completed as f32 / self.total_waypoints as f32) * 100.0
    }

    /// 已完成的航点数。
    pub fn waypoints_completed(&self) -> usize {
        self.waypoints_completed
    }

    /// 总航点数。
    pub fn total_waypoints(&self) -> usize {
        self.total_waypoints
    }

    /// 已飞距离（米）。
    pub fn distance_traveled(&self) -> f32 {
        self.distance_traveled
    }

    /// 任务总距离（米）。
    pub fn distance_total(&self) -> f32 {
        self.total_distance
    }

    /// 剩余距离（米）。
    pub fn distance_remaining(&self) -> f32 {
        (self.total_distance - self.distance_traveled).max(0.0)
    }

    /// 按距离计的完成百分比（0..100）。
    pub fn distance_percent(&self) -> f32 {
        if self.total_distance <= 0.0 {
            0.0
        } else {
            (self.distance_traveled / self.total_distance) * 100.0
        }
    }

    /// 当前阶段。
    pub fn phase(&self) -> MissionPhase {
        self.phase
    }

    fn add_distance(&mut self, d: f32) {
        self.distance_traveled += d.max(0.0);
    }
}

/// 任务执行器：逐个下发航点，返回当前指令，并跟踪进度。
pub struct MissionExecutor {
    mission: Mission,
    idx: usize,
    phase: MissionPhase,
    progress: MissionProgress,
    /// 上一次位置（用于累计已飞距离）。
    last_pos: Option<Vec3>,
}

impl MissionExecutor {
    /// 以任务构建执行器。
    pub fn new(mission: Mission) -> Result<Self> {
        mission.validate()?;
        let total = mission.total_distance();
        let n = mission.waypoints.len();
        Ok(Self {
            mission,
            idx: 0,
            phase: MissionPhase::NotStarted,
            progress: MissionProgress::new(n, total),
            last_pos: None,
        })
    }

    /// 开始执行（从第一个航点）。
    pub fn start(&mut self) {
        self.idx = 0;
        self.phase = MissionPhase::InProgress;
        self.progress.waypoints_completed = 0;
        self.progress.distance_traveled = 0.0;
        self.progress.phase = MissionPhase::InProgress;
        self.last_pos = None;
    }

    /// 当前执行阶段。
    pub fn phase(&self) -> MissionPhase {
        self.phase
    }

    /// 当前执行进度。
    pub fn progress(&self) -> &MissionProgress {
        &self.progress
    }

    /// 上报最新位置，累计已飞距离（供进度统计）。
    pub fn update_position(&mut self, pos: Vec3) {
        if let Some(prev) = self.last_pos {
            self.progress.add_distance(pos.sub(prev).norm());
        }
        self.last_pos = Some(pos);
    }

    /// 当前航点序号。
    pub fn index(&self) -> usize {
        self.idx
    }

    /// 获取当前航点对应的指令目标；任务完成返回 `None`。
    pub fn current_target(&self) -> Option<CommandTarget> {
        self.mission
            .waypoints
            .get(self.idx)
            .map(|wp| CommandTarget::Position {
                north: wp.north,
                east: wp.east,
                down: -wp.alt,
            })
    }

    /// 推进到下一个航点。
    pub fn advance(&mut self) {
        self.idx += 1;
        self.progress.waypoints_completed = self.idx.min(self.progress.total_waypoints);
        if self.idx >= self.mission.waypoints.len() {
            self.phase = MissionPhase::Completed;
            self.progress.phase = MissionPhase::Completed;
        }
    }

    /// 中止任务。
    pub fn abort(&mut self) {
        self.phase = MissionPhase::Aborted;
        self.progress.phase = MissionPhase::Aborted;
    }

    /// 访问内部任务。
    pub fn mission(&self) -> &Mission {
        &self.mission
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_mission() -> Mission {
        Mission::new(
            "survey-01",
            vec![
                Waypoint {
                    sequence: 0,
                    north: 0.0,
                    east: 0.0,
                    alt: 30.0,
                    accept_radius: 2.0,
                },
                Waypoint {
                    sequence: 1,
                    north: 100.0,
                    east: 0.0,
                    alt: 30.0,
                    accept_radius: 2.0,
                },
                Waypoint {
                    sequence: 2,
                    north: 100.0,
                    east: 100.0,
                    alt: 30.0,
                    accept_radius: 2.0,
                },
            ],
        )
    }

    #[test]
    fn executor_walks_waypoints() {
        let mut ex = MissionExecutor::new(sample_mission()).unwrap();
        ex.start();
        assert!(ex.current_target().is_some());
        ex.advance();
        ex.advance();
        ex.advance();
        assert_eq!(ex.phase(), MissionPhase::Completed);
        assert!(ex.current_target().is_none());
    }

    #[test]
    fn invalid_sequence_rejected() {
        let mut m = sample_mission();
        m.waypoints[1].sequence = 9;
        assert!(MissionExecutor::new(m).is_err());
    }
    #[test]
    fn mission_json_roundtrip() {
        let m = sample_mission();
        let json = m.to_json().unwrap();
        let back = Mission::from_json(&json).unwrap();
        assert_eq!(back.id, "survey-01");
        assert_eq!(back.waypoints.len(), 3);
    }

    #[test]
    fn mission_file_save_load() {
        let m = sample_mission();
        let path = std::env::temp_dir().join("smart_brain_test_mission.json");
        m.save(&path).unwrap();
        let loaded = Mission::load(&path).unwrap();
        assert_eq!(loaded.id, m.id);
        assert_eq!(loaded.waypoints.len(), m.waypoints.len());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn total_distance_is_correct() {
        let m = sample_mission();
        let d = m.total_distance();
        // (0,0)->(100,0)->(100,100) 同一高度，距离 200。
        assert!((d - 200.0).abs() < 0.01, "d={d}");
    }

    #[test]
    fn progress_tracks_waypoints_and_distance() {
        let mut ex = MissionExecutor::new(sample_mission()).unwrap();
        ex.start();
        assert_eq!(ex.progress().percent_complete(), 0.0);

        // 飞行中上报位置，累计已飞距离。
        ex.update_position(Vec3::new(0.0, 0.0, 0.0));
        ex.update_position(Vec3::new(50.0, 0.0, 0.0));
        assert!(ex.progress().distance_traveled() > 0.0);

        ex.advance();
        assert_eq!(ex.progress().waypoints_completed(), 1);
        let third = 100.0 / 3.0;
        assert!((ex.progress().percent_complete() - third).abs() < 0.01);

        ex.advance();
        ex.advance();
        assert_eq!(ex.progress().waypoints_completed(), 3);
        assert_eq!(ex.progress().percent_complete(), 100.0);
        assert_eq!(ex.phase(), MissionPhase::Completed);
    }

    #[test]
    fn validate_rejects_empty_and_bad_waypoints() {
        let empty = Mission::new("x", vec![]);
        assert!(empty.validate().is_err());
        assert!(MissionExecutor::new(empty).is_err());

        let mut bad_alt = sample_mission();
        bad_alt.waypoints[0].alt = -5.0;
        assert!(bad_alt.validate().is_err());
        assert!(MissionExecutor::new(bad_alt).is_err());
    }

    #[test]
    fn from_json_rejects_invalid() {
        // 非法 JSON / 序号乱序 / 缺航点都应报错。
        assert!(Mission::from_json("not json").is_err());
        let mut m = sample_mission();
        m.waypoints[1].sequence = 9;
        let json = m.to_json().unwrap();
        assert!(Mission::from_json(&json).is_err());
        let empty = Mission::new("x", vec![]);
        assert!(Mission::from_json(&empty.to_json().unwrap()).is_err());
    }

    #[test]
    fn save_load_error_paths() {
        let m = sample_mission();
        // 保存到非法路径 -> 错误。
        assert!(m.save("/nonexistent_dir_xyz/file.json").is_err());
        // 加载不存在的文件 -> 错误。
        assert!(Mission::load("/nonexistent_dir_xyz/file.json").is_err());
    }

    #[test]
    fn single_waypoint_has_zero_total_distance() {
        let m = Mission::new(
            "single",
            vec![Waypoint {
                sequence: 0,
                north: 1.0,
                east: 2.0,
                alt: 10.0,
                accept_radius: 2.0,
            }],
        );
        assert!(m.total_distance() < 1e-6);
    }

    #[test]
    fn progress_distance_metrics() {
        let mut ex = MissionExecutor::new(sample_mission()).unwrap();
        ex.start();
        let p = ex.progress();
        assert_eq!(p.total_waypoints(), 3);
        assert!((p.distance_total() - 200.0).abs() < 0.01);
        assert_eq!(p.distance_remaining(), p.distance_total());
        assert_eq!(p.distance_percent(), 0.0);
        ex.update_position(Vec3::new(0.0, 0.0, 0.0));
        ex.update_position(Vec3::new(100.0, 0.0, 0.0));
        let p2 = ex.progress();
        assert!(p2.distance_traveled() > 0.0);
        // 已飞一段，剩余减少、距离百分比上升。
        assert!(p2.distance_remaining() < p2.distance_total());
        assert!(p2.distance_percent() > 0.0 && p2.distance_percent() <= 100.0);
    }

    #[test]
    fn distance_percent_zero_when_no_total() {
        let mut ex = MissionExecutor::new(Mission::new(
            "s",
            vec![Waypoint {
                sequence: 0,
                north: 0.0,
                east: 0.0,
                alt: 5.0,
                accept_radius: 2.0,
            }],
        ))
        .unwrap();
        ex.start();
        assert_eq!(ex.progress().distance_percent(), 0.0);
    }

    #[test]
    fn executor_index_mission_and_abort() {
        let mut ex = MissionExecutor::new(sample_mission()).unwrap();
        ex.start();
        assert_eq!(ex.index(), 0);
        assert_eq!(ex.mission().id, "survey-01");
        // current_target 把高度映射为 down = -alt。
        if let Some(CommandTarget::Position { down, .. }) = ex.current_target() {
            assert!((down + 30.0).abs() < 1e-4, "down={down}");
        } else {
            panic!("expected a position target");
        }
        ex.advance();
        assert_eq!(ex.index(), 1);
        ex.abort();
        assert_eq!(ex.phase(), MissionPhase::Aborted);
        assert_eq!(ex.progress().phase(), MissionPhase::Aborted);
        // 中止后不应再产生目标。
        ex.advance();
        assert_eq!(ex.phase(), MissionPhase::Aborted);
    }
}
