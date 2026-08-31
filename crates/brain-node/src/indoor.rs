//! 室内无 GPS 自主避障流水线演示。
//!
//! 串联本工作区新增的室内栈：
//!   VIO（视觉惯性里程计定位）→ 3D 占据网格（避障建图）→ A*/DWA（局部避障）
//!   → 前沿探索 / 面包屑回溯（任务层）
//! 全程使用合成数据，演示各模块如何协同，替代 GPS 完成室内自定位与避障。

use brain_core::Vec3;
use brain_mapping::{GridConfig, Index3, OccupancyGrid3D, RaycastUpdater};
use brain_nav::{Backtracker, Explorer};
use brain_odometry::{IcpConfig, ImuSample, VisualInertialOdometry};
use brain_planning::{AStar2D, DwaConfig, DwaPlanner, GridPoint};

/// 演示 VIO：用合成 IMU + RGB-D 帧得到无 GPS 的局部位姿。
fn demonstrate_vio() {
    println!("\n=== 1. 视觉惯性里程计（VIO）：无 GPS 自定位 ===");
    let mut vio = VisualInertialOdometry::new(IcpConfig::default());

    // 合成一个立方体点云（世界系下）。
    let cube = |off: Vec3| -> Vec<Vec3> {
        (0..8)
            .map(|i| {
                Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32).add(off)
            })
            .collect()
    };

    let mut t = 0u64;
    for step in 0..6 {
        // 相机位姿：世界系下从 -x 到 +x 移动，看到的是点云相对左移。
        let cam_pos = Vec3::new(step as f32 * 0.05, 0.0, 0.0);
        let cloud: Vec<Vec3> = cube(Vec3::ZERO).iter().map(|&p| p.sub(cam_pos)).collect();

        // 高频 IMU（静止：加速度=重力，无角速度）。
        for _ in 0..10 {
            let s = ImuSample::new(t, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);
            vio.update_imu(&s, t);
            t += 10;
        }
        if let Some(est) = vio.update_frame(&cloud, t) {
            let fv = vio.filtered_velocity();
            println!(
                "  frame {step}: vo_ready={} est.pos=({:+.2},{:+.2},{:+.2}) kalman_vel=({:+.2},{:+.2}) src={:?}",
                vio.is_visual_ready(),
                est.pose.position.x,
                est.pose.position.y,
                est.pose.position.z,
                fv.x,
                fv.z,
                est.source
            );
        }
    }
}

/// 演示占据网格 + 光线投射建图 + A*/DWA 避障。
fn demonstrate_mapping_and_planning() {
    println!("\n=== 2. 3D 占据网格 + 光线投射 + A*/DWA 避障 ===");
    let cfg = GridConfig::from_world_size(0.5, 20.0, 20.0, 20.0);
    let mut grid = OccupancyGrid3D::new(cfg);
    let updater = RaycastUpdater::default();
    let sensor_origin = Vec3::new(0.5, 0.5, 1.0);

    // 合成一堵“实体墙面”（最近表面点）：位于 x=1.0 的竖直平面。
    let mut cloud = Vec::new();
    for i in 0..6 {
        for j in 0..4 {
            cloud.push(Vec3::new(1.0, 0.25 + i as f32 * 0.5, 0.75 + j as f32 * 0.5));
        }
    }
    updater.update_point_cloud(&mut grid, sensor_origin, &cloud, 8.0);
    let (free, occ, unk) = grid.counts();
    println!("  grid counts: free={free} occupied={occ} unknown={unk}");
    println!(
        "  is_occupied(front wall)={} is_free(origin)={}",
        grid.is_occupied(Vec3::new(1.0, 0.5, 1.0)),
        grid.is_free(sensor_origin)
    );

    // A* 全局路径（在 z 层 2 上）。
    let astar = AStar2D::new(2, 0.2);
    if let Some(path) = astar.plan(&grid, GridPoint { x: 1, y: 1 }, GridPoint { x: 15, y: 15 }) {
        println!(
            "  A* path: {} points, start={:?} goal={:?}",
            path.len(),
            path.first(),
            path.last()
        );
    } else {
        println!("  A*: no path (blocked)");
    }

    // DWA 局部避障：目标在前方但有一堵墙 → 应拒绝撞墙。
    let dwa = DwaPlanner::new(DwaConfig::default());
    let cmd = dwa.plan(
        &grid,
        sensor_origin,
        0.0,
        Vec3::new(8.0, 0.5, 1.0),
        (0.3, 0.0),
    );
    println!("  DWA cmd (wall ahead): {cmd:?} (None = 拒绝前进，安全兜底)");
}

/// 演示前沿探索 + 面包屑回溯。
fn demonstrate_explore_and_backtrack() {
    println!("\n=== 3. 前沿探索 + 面包屑原路返回 ===");
    let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 8.0, 8.0, 8.0));
    // 左侧厚块已探索（空闲），右侧未知 → 前沿在 x=3 处。
    for x in 0..3 {
        for y in 0..8 {
            for z in 0..4 {
                grid.set_log_odds(Index3::new(x, y, z), -2.0);
            }
        }
    }
    let ex = Explorer::new(0);
    if let Some(t) = ex.next_target(&grid, Vec3::new(1.0, 1.0, 0.5)) {
        println!(
            "  explore next target: ({:.1},{:.1},{:.1}) coverage={}",
            t.position.x, t.position.y, t.position.z, t.coverage
        );
    }

    let mut bt = Backtracker::new(100, 1.0);
    for i in 0..5 {
        bt.record(0, Vec3::new(i as f32, 0.0, 0.0));
    }
    println!("  breadcrumbs: {}", bt.len());
    let rewind = bt.start_rewind();
    println!(
        "  rewind path: {} steps, first={:?}",
        rewind.len(),
        rewind.first()
    );
}

/// 顶层入口。
pub fn run() {
    println!("=== 室内无 GPS 自主避障（视觉定位）演示 ===");
    demonstrate_vio();
    demonstrate_mapping_and_planning();
    demonstrate_explore_and_backtrack();
}
