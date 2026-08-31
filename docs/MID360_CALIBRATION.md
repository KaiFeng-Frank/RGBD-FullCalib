# 作者 MID-360S rig：独立定量验收层级

本页定义作者这一套 MID-360S + D435i rig 从 LOCAL operational 进入
VALIDATED 的独立定量验收层级。通用采集、求解、导入和查看入口见
[`LIDAR_CAMERA_EXTRINSIC.md`](LIDAR_CAMERA_EXTRINSIC.md)。

当前 device/mount 已生效的结果为
`results/mid360s_d435i_extrinsic.local.json`。它汇总 scene06–scene10 五场景稠密配准、
10.019/16.873 mm 优化器内点 median/P90 与 0.00720° / 0.120 mm 多起点一致性，
以 **LOCAL operational** 层级绑定记录的设备和安装会话，并已用于同画布雷视融合。
VALIDATED 层级在此基础上增加独立数据验收。LiDAR–IMU 与时间同步按独立工程链管理。

| `task_id` | 结果产物 | 当前状态 |
|---|---|---|
| `mid360s_d435i_ext` | `results/mid360s_d435i_extrinsic.local.json` | LOCAL operational · 当前 device/mount 已生效 |

直接雷视流程已实现；官方 `direct_visual_lidar_calibration` targetless
流程的安全默认输出是 `results/mid360s_d435i_extrinsic.draft.json`。
对某次连续、未拆动的安装，方向和投影审查后可使用明确标为
`operational` 的本地结果。下面的完成门定义作者 rig 从 LOCAL 进入 VALIDATED 的审计层。

### 作者 rig 的 VALIDATED 完成门

进入 VALIDATED 时，完成门由 `viewer/lidar_calib.py` 固定，不能由结果 JSON 自己声明：

- `task_id`、设备角色/型号/序列号、`rig_id`、UTC 时间、方法和坐标方向必须完整；
- 求解集与独立验收集必须分别记录路径和不同的 SHA-256；
- `result` 必须符合该任务的字段、类型和刚体矩阵约束；
- `validation.checks` 必须恰好覆盖预注册 gate ID，且 check 值必须与 result 值一致；
- 程序重新计算每个数值阈值，不相信产物自报的 `passed`；
- 已完成产物之间的 rig/传感器序列号必须一致；
- 最终留出数据的哈希不得与任何上游求解/验收数据重合；最终产物还必须引用当前 6 个
  上游 JSON 的实际 SHA-256。任一上游结果变化，最终验收自动回到「需重做」。

结构不完整时状态为 `pending`；结构完整但阈值、独立核查、身份一致性或上游哈希失败时为
`rework`；全部通过才是 `done`。若启用作者验收状态页，它每 5 秒重读一次，
无需重启服务。

## 扩展工程目录

健康、IMU、时间同步与 rig 闭环规则按应用需求分别启用；当前雷视外参保持
LOCAL operational 已生效状态。

## 作者 rig 的 VALIDATED 预注册数值门

这些阈值定义作者 rig 从 LOCAL 进入 VALIDATED 的定量口径。应在对应数据采集前冻结；
若要改变，应先改注册表并留下
原因，不能看完结果后迁移阈值。

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
