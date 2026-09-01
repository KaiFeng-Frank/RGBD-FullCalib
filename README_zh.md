# RGBD-FullCalib

**RGBD-FullCalib 是一套开源的一站式多传感器标定工作台，覆盖相机、
IMU、LiDAR 与跨传感器外参，把标定输出变成可视化、机器可读且带明确
部署范围的结果。v0.1 交付一台 D435i 端到端标定；v0.2.0 交付 24 条
声明式判决规则；v0.3 已收口直接雷视链、MID-360S IMU 标定、LiDAR–IMU 几何、
跨设备时间对齐、扫描 deskew、MID-360S 原生查看和
D435i/MID-360S 单画布融合。**

[English README](README.md) · 完整实录 [CALIBRATION.md](CALIBRATION.md) ·
误差影响分析 [IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md) ·
[已完成范围](ROADMAP.md) · [CHANGELOG](CHANGELOG.md)

![D435i 与 MID-360S 在彩色相机坐标系中的单画布实时融合点云](results/lidar_rgbd_fused_viewer.gif)

<p align="center">
  <img src="docs/img/rig_mid360s_d435i_ready.jpg" alt="已准备好的 MID-360S 与 D435i 刚性雷视一体支架" width="47%">
  <img src="docs/img/rig_mid360s_d435i_wiring.jpg" alt="MID-360S 与 D435i 雷视一体 rig 通电采集中" width="47%">
</p>
<p align="center"><sub>MID-360S + D435i 刚性雷视一体支架：干净的准备状态（左）与通电采集状态（右）</sub></p>

标定这行最怕的不是误差大，是**看起来漂亮但错得一致**。RGBD-FullCalib
把拟合指标与物理参照、重复采集、device/mount 身份一起固结，并把适用的
证据层级直接写在结果旁边。CLI、报告、GUI 与实时融合查看器共用同一份事实。

## 这套 rig

<p align="center">
  <img src="docs/img/rig_d435i.jpg" alt="D435i 标定设备" width="61%">
  <img src="docs/img/target_aprilgrid.jpg" alt="AprilGrid 刚性标定板" width="34%">
</p>
<p align="center"><sub>Intel RealSense D435i（fw 5.12.7.100，USB 3.2）· DFOPTIX <code>Tag6-320-35.2mm</code> 刚性 AprilGrid</sub></p>

靶标是成品板不是自己打印的 —— 这点很重要:排除了打印缩放这个误差源,
残差就能归到检测和相机身上,而不是尺子身上。`tagSpacing = 0.3` 经四方确认
(尺子实测白缝 10.6 mm、型号名反推、Kalibr 默认、重投影误差扫描在 0.30 处有明确极小)。

## 现已落地

**v0.1(已交付)**:一台相机端到端(RGB 内参 → IR 双目 → RGB↔IR 外参 → 深度噪声
模型 → cam-IMU → 加速度计内参 → 温漂模型),全套工具、写在散文/GUI 代码里的手写
判决、GUI、15+ 条真实时间换来的坑。

**v0.2.0(已交付)**:上述核查已变成 24 条声明式规则,CLI、`REPORT.md`、GUI 卡片
共用一个引擎和一个事实源;加设备的判决项变成写规则,不是 fork 判决代码。

**v0.3（已交付）**：viewer 可直接渲染任意 ROS 2
`sensor_msgs/msg/PointCloud2` 与 Livox `CustomMsg`；直接雷视流程从采集、
求解到导入全链打通；D435i/MID-360S 实时点云在同一彩色相机坐标系
画布中融合。文档 rig 已固结 MID-360S IMU 标定、五场景雷视外参、
LiDAR–IMU 几何和雷达到 D435i 的常数时偏；逐点 `offset_time` 与 200 Hz
内置陀螺仪驱动带已知 IMU 杠杆臂的扫描末旋转 deskew。

## MID-360S IMU 端到端 pipeline 与 ROS 2 部署

一个 fail-closed 入口已连通：device/mount 预检→自动多姿态静置采集
（或复用 rosbag2/NPZ）→明确的 `g × 9.80665` 单位转换→解算→
独立 holdout/可观测性验收→正式结果 promotion→GUI 目录回归：

```bash
# 在线：从 /livox/imu 自动收集 12 fit + 3 holdout 姿态。
./calibrate_mid360s_imu.sh --project-root /path/to/your_rig_workspace

# 复现并核验本文档 rig，不覆盖已固结结果。
./calibrate_mid360s_imu.sh --verify-existing --inputs \
  data/mid360s_imu_intrinsic_20260901_run1 \
  data/mid360s_imu_intrinsic_20260901_run2 \
  data/mid360s_imu_intrinsic_20260901_run3
```

仓库同时是可安装的 ROS 2 包。source ROS 2 Jazzy 后：

```bash
colcon build --symlink-install --packages-select rgbd_fullcalib
source install/setup.bash
ros2 launch rgbd_fullcalib mid360s_imu_calibration.launch.py \
  project_root:=/absolute/path/to/artifact-root \
  config_file:=/absolute/path/to/mid360s_imu_calibration.yaml
ros2 launch rgbd_fullcalib mid360s_imu_runtime.launch.py \
  project_root:=/absolute/path/to/artifact-root \
  config_file:=/absolute/path/to/mid360s_imu_calibration.yaml
```

运行时节点订阅 raw `/livox/imu`，应用已 promotion 的模型，在
`/livox/imu_calibrated` 发布 SI 观测。它只应用有证据的 accel
bias/scale/非正交与 gyro 静态 bias，不虚构 gyro scale。topic、frame、
身份、路径与输出均在
[`config/mid360s_imu_calibration.yaml`](config/mid360s_imu_calibration.yaml)
中参数化。

本 rig 实际打印的两件式支架也已原样收入：
[`3MF 文件`](hardware/MID360S_D435i_RK3588S_BATTERY_REV6_A1_PLA_4W15.3mf)
与[几何/安装方向说明](hardware/README.md)。ROS 安装后位于
`share/rgbd_fullcalib/hardware`。

## 结果速览

| 结果卡 | 关键数字 | 判决 |
|---|---|---|
| RGB 内参 | fx/fy 884.8/883.9，重投影 σ 0.56 px | ✅ 冻结 |
| IR 双目 | 基线 50.148 mm，与出厂值差 0.16% | ✅ 冻结 |
| 深度噪声 | 1/2/4 m 处 σ = 4.1/16.3/65 mm | ✅ 用作权重 |
| cam–IMU | 时移 4.0494 ms，旋转约 1° 量级 | ✅ 冻结旋转与时移 |
| RGB↔IR 外参 | 与出厂值的 \|t\| 差 0.023 mm | ✅ 冻结 |
| 加速度计内参 | 最大非正交 2.83°，12 姿态残差 3.76 mm/s² | ✅ 使用 |
| 温漂模型 | 深度尺度 −372 ppm/°C | ✅ 补偿深度尺度 |
| MID-360S IMU | accel bias `[−0.01745,+0.02770,−0.01560]` m/s²；scale `[0.999665,1.006801,1.007030]`；非正交 `[−0.002235,+0.001770,−0.003168]` rad | ✅ 当前 rig operational |
| MID-360S→D435i 外参 | 五场景，inlier median/P90 10.0/16.9 mm | ✅ 当前 rig operational |
| MID-360S LiDAR–IMU + deskew | 轴向同基，IMU `[+11.00,+23.29,−44.12]` mm；高旋转平面 P95 70.04→20.39 mm | ✅ 当前 rig operational |
| MID-360S→D435i 时间 | dual-gyro 时偏 −1.940 ms；`t_depth = t_livox − 5.989 ms` | ✅ 当前 rig 常数时偏 operational |

GUI 与结果 API 当前显示 **11 项本 rig 结果**。另外的 **7 项「标定参考」**
是给他人、其他设备或不同部署条件的方法目录，不是本 rig 的
pending/rework，也不计入结果状态。

机器可读结果在 [`data/*.yaml`](data) 与 [`results/*.json`](results)；
雷视时偏、MID-360S IMU、LiDAR–IMU 与 deskew 验收分别固结于
[`results/mid360s_d435i_timesync.json`](results/mid360s_d435i_timesync.json)、
[`results/mid360s_imu.json`](results/mid360s_imu.json)、
[`results/mid360s_lidar_imu.json`](results/mid360s_lidar_imu.json) 和
[`results/mid360s_deskew_validation.json`](results/mid360s_deskew_validation.json)。
实扫审计的逐点时间有效率为 100%，IMU 完整覆盖 646/647 帧；同点
ablation 加入已标定杠杆臂分量后 P95 从 20.21 降到 19.48 mm（组中位约
3.6%，paired median 约 2.8%）。

MID-360S IMU 结果还固结 gyro bias
`[−0.006402,+0.000822,−0.008318] rad/s`，以及 196.156 Hz、0.496 s 窗的逐轴
短窗白噪声密度：accel `[0.004974,0.005405,0.006853] m/s²/√Hz`，gyro
`[0.001421,0.001216,0.001749] rad/s/√Hz`。这一口径不冒充长时 Allan
bias instability/random walk；gyro scale/非正交属于已知角速率转台的
通用参考方法，不在本 operational 结果口径内。
雷视外参属于文档中的刚性安装；其他 rig 或重新拆装后通过同一
流程生成自己的外参。

## 特意发表的负结果

- **本机 cam-IMU 平移标不出来**:三次独立解散布 24.6~31.7 mm,量本身才 ~26 mm。
  杠杆臂(IMU 距光心 2~3 cm)太短,手持激励喂不出可观测性。旋转和时移可冻结,
  平移只配当 VIO 初值。
- **六面静置法修不了动态残差**:同 bag 严格对照,只改加速度前处理,Kalibr 归一化
  残差基本不动 —— 静置内参是真的,但动态残差的主因是振动/模糊/同步抖动。
- **IMU 内参校正会耦合进外参旋转**(~0.9°):用哪套轴修正标的,部署就得用哪套。
- **主点温漂在本机是伪命题**(13°C 扫温 R²=0.07)—— 测了,证伪了,从担心清单划掉。
- **单姿态分离不了 bias/scale/非正交**:我试了,得到一个自信且错误的标度结论;
  12 姿态之后真凶是非正交。作为可辨识性陷阱的完整案例留档。

## 坑库

15+ 条完整故事在 [CALIBRATION.md](CALIBRATION.md) 的「采集踩过的坑」章,
一行版摘要见 [English README](README.md#pitfalls-that-cost-real-hours)。

## v0.2.0 已交付:判决层变成数据

![带在线标定与 SLAM 影响徽章的最新深度模型判决卡片](docs/img/verdict_depth_badges.png)

```bash
python -m verdicts                      # 终端判决
python -m verdicts --md REPORT.md       # Markdown 报告(已入库)
```

原来以散文形式写在文档和 GUI 代码里的每一条核查,现在都在
[`verdicts/rules_d435i.yaml`](verdicts/rules_d435i.yaml) —— 现有 26 条规则
(原 24 条 + 2 条非阻塞的原样 IR 立体校正验收),每条 =
{取值表达式, 外部参照, 容差, 行动指令},由 ~200 行引擎执行,CLI 报告、
REPORT.md、GUI 卡片同一事实源。已发布产物缺失时记为完整性问题,不登记成未来工作。
加设备的判决项 = 写规则,不是 fork 判决代码。

机器检验散文第一天就有收获:旧文案"五次独立测量基线"被抓出**假独立** ——
Kalibr 的 imu-camera 阶段逐位继承相机链,五个数里三个是复制品。真实证据是
"两次独立标定(不同 bag)",差值 0.005 mm;yaml 里留了注释讲原因。

## GUI

WebGL2 点云查看器,三页签:实时点云(16-bit 深度图在 shader 内反投影,
或原生 xyz/intensity/RGB 点流走 VBO)、
标定结果(判决卡片,直接从 yaml/json 生成;每张卡带两枚 SLAM 徽章:**能否在线标定**
(外参旋转/时移/IMU 零偏是 VINS 类系统的标准在线状态,内参与深度链不是)与
**对 SLAM 影响定级**,每一级背后是实测传播数字)、标定参考(七项可复用方法：
四项 D435i + 三项 LiDAR/rig 参考，含适用场景/流程/输出)。第三页供其他设备
或不同部署条件查用,不计入文档 rig 的 11 项结果，不是
pending/rework，也不是版本迭代计划。温漂补偿一个勾选框直接作用到实时点云。

![最新标定结果与在线标定、SLAM 影响徽章](results/gui_slam_badges.png)

```bash
./view_pointcloud.sh fused                     # 同一坐标系、同一画布
./view_pointcloud.sh mid360                    # 复用已有驱动,或自动启动 MID-360S
./view_pointcloud.sh ros2 /your/points         # 任意 PointCloud2 话题
./view_pointcloud.sh ros2 auto                 # 只有一个候选时自动选择
./view_pointcloud.sh d435i --alt-emitter       # D435i 直连
./view_pointcloud.sh synthetic-points          # 无硬件原生点流回归
```

`fused` 把两路实时输入变换到同一个
`camera_color_optical_frame` 画布。文档 rig 的
`results/mid360s_d435i_extrinsic.local.json`、
`results/mid360s_d435i_timesync.json` 与
`results/mid360s_lidar_imu.json` 自动载入；其他 rig 用
`--extrinsic PATH` 指定自己的相机外参。Livox `CustomMsg` 使用每点
`offset_time`、内置 IMU 与已知 IMU 杠杆臂补偿到扫描末姿态。该范围是
rotation-only deskew，不声称平台平移补偿或完整 6DoF deskew。
[通用雷视外参流程](docs/LIDAR_CAMERA_EXTRINSIC.md)已覆盖采集、求解、
结果方向、导入与融合查看。

统一入口打开 `http://localhost:8080`，隔离 ROS Jazzy 与 Conda，
任一实时流断开时会同步停止两个后端。ROS 2 点云按消息 frame
渲染当前帧。

## 关键文件

- `calibrate_mid360s_imu.sh`：MID-360S IMU 采集→解算→holdout→promotion
  端到端入口。
- `launch/mid360s_imu_calibration.launch.py`：ROS 2 标定 pipeline 入口。
- `launch/mid360s_imu_runtime.launch.py`：ROS 2 校正 IMU 运行时部署。
- `tools/record_mid360s_imu_poses.py`：自动多姿态静置采集。
- `tools/calibrate_mid360s_imu_intrinsics.py`：rosbag2/NPZ 解算与可观测性验收。
- `tools/promote_mid360s_imu.py`：严格绑定 current-rig 的正式结果边界。
- `tools/mid360s_imu_runtime.py`：raw g 输入到校正 SI IMU 话题。
- `tools/calibrate_lidar_camera_timesync.py`：dual-gyro 常数时偏拟合。
- `viewer/livox_deskew.py`：逐点扫描末旋转 deskew 与 IMU 杠杆臂。
- `results/mid360s_d435i_timesync.json`：雷视时间方程与常数时偏。
- `results/mid360s_imu.json`：MID-360S IMU operational 内参与短窗白噪声。
- `results/mid360s_lidar_imu.json`：`T_lidar_imu` 与 deskew 结果。
- `results/mid360s_deskew_validation.json`：实扫 deskew A/B 验收。
- `hardware/MID360S_D435i_RK3588S_BATTERY_REV6_A1_PLA_4W15.3mf`：实际
  两件式打印支架。

## 在你自己的 D435i 上复现

1. `pip install -r requirements.txt`(Python ≥3.10,**不需要装 ROS**)
2. 构建 Kalibr 镜像:先把 [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr)
   clone 到 `kalibr/`,然后 `docker build -t kalibr:noetic -f kalibr/Dockerfile_ros1_20_04 kalibr/`
3. 打印 AprilGrid,**用尺子实测** tag 尺寸和间距,改 yaml
4. 采集 → 标定 → **先看判决行再决定信不信** —— 每个阶段的确切命令在
   [CALIBRATION.md](CALIBRATION.md) 末尾「复现」章

注意:本仓库数字来自一台 fw 5.12.7.100 的 D435i(USB 3.2)。你的机器数字**一定**
不同 —— 这正是要标定的原因。

## 为什么连"影响"也发

[IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md) 把每项实测误差传播到建图/定位量级
(含 Mid-360 雷视对照列),组织原则只有一条:把误差分成 零均值随机 / 恒定系统性 /
**状态依赖系统性** —— 第三类毁地图(温漂深度尺度拼出双层墙),第二类静默卡死
图内定位。它还按实测影响排定标定优先级:只标你的应用真正感觉得到的量,能省几天。

## License

MIT — 见 [LICENSE](LICENSE)。
