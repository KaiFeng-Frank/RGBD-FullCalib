# MID-360S 标定通用参考协议

本页是给他人、其他设备和不同部署条件的方法参考。GUI 的
「标定参考」共 7 项：4 项 D435i 参考与 3 项 LiDAR/rig 参考。
参考项不是当前文档 rig 的 pending/rework，不计入它的结果总数，
也不表示版本迭代。通用采集、求解、导入和查看入口见
[`LIDAR_CAMERA_EXTRINSIC.md`](LIDAR_CAMERA_EXTRINSIC.md)。

## 当前文档 rig 的已固结结果

| `task_id` | 结果产物 | 当前 rig 状态 |
|---|---|---|
| `mid360s_imu` | `results/mid360s_imu.json` | operational；17 姿态（14 fit + 3 holdout），fit/holdout RMS 0.00962/0.01194 m/s²，rank 9/9 |
| `mid360s_d435i_ext` | `results/mid360s_d435i_extrinsic.local.json` | operational；scene06–scene10 五场景，已用于同画布雷视融合 |
| `mid360s_lidar_imu` | `results/mid360s_lidar_imu.json` | operational；轴向同基，IMU 位置 `[+11.00,+23.29,−44.12] mm` |
| `mid360s_d435i_td` | `results/mid360s_d435i_timesync.json` | operational；dual-gyro 时偏 −1.940 ms，合成深度时钟方程 `t_depth = t_livox − 5.989 ms` |

MID-360S IMU 结果固结 accel bias
`[−0.0174527,+0.0276977,−0.0155995] m/s²`、scale
`[0.9996646,1.0068007,1.0070296]`、非正交
`[−0.00223534,+0.00176965,−0.00316793] rad` 与 gyro bias
`[−0.00640217,+0.000822435,−0.00831762] rad/s`。短窗白噪声密度基于
196.156 Hz、0.495805 s 窗；`allan_characterization = not_performed`，不把它写成
长时 Allan bias instability/random walk。gyro scale/非正交仅保留在已知角速率
转台的通用参考协议中，不属于该 operational 结果字段。

LiDAR–IMU 结果同时驱动 Livox 扫描末 rotation-only deskew：逐点
`offset_time` + 内置 IMU + 官方杠杆臂。同一正式协议下，高旋转主平面
P95 为 70.04→20.39 mm，70/70 帧改善；低旋转对照 17.255→17.262 mm。
同点杠杆臂 ablation 为 20.21→19.48 mm；`offset_time` 有效率 100%，
IMU 完整覆盖 646/647 帧。该范围不声称平台平移或完整 6DoF deskew。

## GUI 中的 3 项 LiDAR/rig 参考

| 参考项 | 适用情况 | 参考输出 |
|---|---|---|
| MID-360S 测距与点云健康验收 | 新设备验收、特殊材质环境或疑似测距/覆盖异常 | 测距、覆盖率与冷热稳态健康报告 |
| LiDAR/相机到 `base_link` 安装外参 | 把雷视组件安装到机器人并需要 `base_link` 位姿 | `T_base_lidar` / `T_base_camera` |
| 雷视–IMU 独立留出数据闭环验收 | 其他设备或复现者需要完整复现性验收 | 独立留出闭环报告与输入哈希 |

**下文的 VALIDATED 数值门是可选的通用严格验收模板，不会反向把上表
operational 结果定义为未完成。** 只有选择该模板的复现者，才使用其
`pending` / `rework` / `done` 生命周期。

## 可选严格 VALIDATED 结果封装

选择该严格层时，数值门由 `viewer/lidar_calib.py` 固定，不由结果 JSON
自己声明：

- `task_id`、设备角色/型号/序列号、`rig_id`、UTC 时间、方法和坐标方向完整；
- 求解集与独立验收集分别记录路径和不同的 SHA-256；
- `result` 符合对应字段、类型和刚体矩阵约束；
- `validation.checks` 恰好覆盖预注册 gate ID，check 值与 result 值一致；
- 程序重算每个数值阈值，不仅依赖产物自报的 `passed`；
- 同一严格验收包内的 rig/传感器序列号一致；
- 留出数据不与求解数据哈希重合，并引用对应上游 JSON 的实际 SHA-256。

## 可选严格 VALIDATED 预注册数值门

下列阈值仅对选择严格参考协议的复现者生效。应在对应数据采集前冻结，
不在看完结果后迁移阈值。

| 任务 | VALIDATED 完成门 |
|---|---|
| MID 测距健康 | A/B 各 10 个逐因素条件，覆盖 1/3/5/10 m、`≥4` 种材质、`≥3` 档入射角和 `≥10°C` 核心温差；测距偏差/测距 P95/重复偏差差的 95% 保守上界分别 `≤20/40/10 mm`；固定时空角格覆盖率 `≥0.90`；外部真值 `U95≤5 mm`；掉帧率 `≤0.01`；设备诊断故障样本数为 0 |
| MID IMU | 明确 `g → m/s²` 且转换误差 `≤ 0.01 m/s²/g`；`allan_duration_h ≥ 2`；`pose_count ≥ 12`；静止/留出重力误差均 `≤ 0.15 m/s²`；静止陀螺零偏范数 `≤ 0.02 rad/s` |
| LiDAR–IMU | 必须给出带符号约定的 `time_offset_lidar_to_imu_ms`；留出墙厚 P95 `≤ 30 mm`；deskew 改善 `≥ 0.20`；重复解旋转/平移/时移差 `≤ 0.20° / 10 mm / 0.20 ms`；逐点时间有效率 `≥ 0.99` |
| LiDAR–D435i 外参 | 有效静态联合录包 `≥5`；留出投影 P95 `≤ 3 px`；重复解旋转/平移差 `≤ 0.20° / 10 mm` |
| LiDAR–D435i 时间 | 固定时钟方程并由 `clock_model_a` 重算 ppm；长记录 `≥ 20 min`；留出残差 P95 `≤ 1 ms`；重复 offset 差 `≤ 1 ms`；时钟尺度偏差绝对值 `≤ 200 ppm` |
| rig–base_link | 闭环平移/旋转 `≤ 5 mm / 0.20°`；装拆复测 `≤ 10 mm / 0.50°` |
| 最终留出验收 | 投影 P95 `≤ 3 px`；墙厚 P95 `≤ 30 mm`；时间残差 P95 `≤ 1 ms`；frame 闭环 `≤ 5 mm / 0.20°`；重复外参 `≤ 10 mm / 0.20°` |

几个不能省略的非标量字段也在 schema 内：IMU 结果必须包含 bias/scale/非正交与噪声密度；
LiDAR–IMU 和雷视外参必须分别给出合法的 `T_imu_lidar`、`T_camera_lidar` 刚体矩阵；
时间标定必须固定代码中登记的时钟方程。`rig_base` 有两条变换，因此不能用一个含糊的
`from/to`，必须写成：

```json
"frame_convention": {
  "T_base_lidar": {"from": "livox_frame", "to": "base_link"},
  "T_base_camera": {"from": "camera_color_optical_frame", "to": "base_link"}
}
```

## MID 测距健康 v1 冻结协议

这项是**健康验收，不是重新标定束角或距离修正模型**。正式 A/B 数据前固定以下口径：

- A、B 是不同 `session_uuid`、不同数据树哈希的完整独立轮次；B 前重新上电、重装目标或雷达并重新量取真值，不能把同一 bag 切成两半；
- 每轮 10 个逐因素条件：哑光参考面 0° 的 1/3/5/10 m（4 个），3 m 的 30°/60°（2 个），3 种其他平整材质的 3 m/0°（3 个），以及哑光 3 m/0° 的第二个核心温度稳态（1 个）；这不是全因子温漂实验；
- 每个条件先预滚动，再保留 10 个不重叠的 1 s 窗；全局固定 `0.5°×0.5°` 方位/俯仰格，不得看完数据后改变格宽、窗口、ROI、tag mask 或残差阈值；
- 外部真值平面必须给出 `n·x=D`、`livox_frame` 中的有限 polygon、量具/原始读数和 `U95≤5 mm`。1/3/5/10 m 只是名义摆位，不是真值；
- 对每个非零 ROI 点，以其射线和外部真值平面的交距计算主误差。RANSAC/TLS 只可用于找靶和诊断，不能把 LiDAR 自己拟合的平面当真值，也不能只在内点上计算 P95；
- `spatiotemporal_coverage_ratio` 是命中真值 polygon 的 `(1 s 窗, 0.5°角格)` 中，被至少一个有限非零且真值残差在 ±150 mm 内的回波占据的比例。它只是固定时空分辨率下的覆盖代理，**不是 per-ray detection probability**；
- bias、P95 和 A/B bias 差均按 1 s block bootstrap 给 95% 上界，再加外部真值 U95；最终取 validation 各条件最差值，不能池化掩盖坏材质；
- 温度完成门只认 `/livox/device_status` 的 SDK `core_temp/100`，环境温度只作旁证。稳态建议为 `|dT/dt|<0.2°C/min` 连续 5 min；
- 主误差使用 polygon 内全部有限非零回波；tag 与 reflectivity 分层另报，不得删掉低反射或低置信返回来美化主指标。

允许在正式 A 前做一次不入结果的 pilot，验证真实扫描密度下 `0.5°/1 s/0.90` 是否可执行。若需要改协议，应先改注册表，再从 A 重新采，不能在看过 B 后调门。

## 最小示例

下面只演示 `mid360s_health` 的结构，数值和哈希是占位符，**不得复制为真实标定结果**：

```json
{
  "schema_version": 1,
  "task_id": "mid360s_health",
  "status": "validated",
  "devices": [
    {"role": "lidar", "model": "Livox Mid-360S", "serial": "REAL_SERIAL"}
  ],
  "rig_id": "mid360s-d435i-01",
  "created_utc": "2026-08-31T12:00:00Z",
  "method": "solver name, version and command",
  "source_data": [
    {
      "role": "calibration",
      "path": "data/health-fit.bag",
      "sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
    },
    {
      "role": "validation",
      "path": "data/health-holdout.bag",
      "sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
    }
  ],
  "frame_convention": {"from": "livox_frame", "to": "range_target_frame"},
  "result": {
    "distance_targets_m": [1.0, 3.0, 5.0, 10.0],
    "metric_definition_version": "mid360s_health/v1",
    "truth_plane_convention": "n dot x = D in livox_frame; beam error = measured range - external ray-plane intersection range",
    "coverage_definition": "10 usable 1 s windows after pre-roll; fixed 0.5 deg angular grid; occupied by >=1 finite nonzero return within 150 mm of external truth plane",
    "primary_return_policy": "all finite nonzero returns inside the external truth polygon; tag and reflectivity are reported but never used to hide primary range errors",
    "core_temperature_source": "SDK core_temp / 100 degC",
    "range_bias_observed_max_mm": 0.0,
    "range_bias_ucb95_mm": 0.0,
    "range_p95_observed_max_mm": 0.0,
    "range_p95_ucb95_mm": 0.0,
    "spatiotemporal_coverage_ratio": 1.0,
    "repeat_bias_delta_observed_max_mm": 0.0,
    "repeat_bias_delta_ucb95_mm": 0.0,
    "truth_range_u95_max_mm": 0.0,
    "material_count": 4,
    "incidence_bin_count": 3,
    "condition_count": 10,
    "core_temperature_span_C": 10.0,
    "frame_drop_ratio": 0.0,
    "status_fault_count": 0
  },
  "summary": {"已知距离复核": "用真实测量替换"},
  "validation": {
    "status": "passed",
    "checks": [
      {"id": "range_bias_ucb95_mm", "value": 0.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "range_p95_ucb95_mm", "value": 0.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "spatiotemporal_coverage_ratio", "value": 1.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "repeat_bias_delta_ucb95_mm", "value": 0.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "truth_range_u95_max_mm", "value": 0.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "material_count", "value": 4, "status": "passed", "detail": "独立复测说明"},
      {"id": "incidence_bin_count", "value": 3, "status": "passed", "detail": "独立复测说明"},
      {"id": "condition_count", "value": 10, "status": "passed", "detail": "独立复测说明"},
      {"id": "core_temperature_span_C", "value": 10.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "frame_drop_ratio", "value": 0.0, "status": "passed", "detail": "独立复测说明"},
      {"id": "status_fault_count", "value": 0, "status": "passed", "detail": "独立复测说明"}
    ]
  }
}
```

权威任务注册表和验证器在 `viewer/lidar_calib.py`，生命周期回归测试在
`viewer/test_calib_state.py`。
