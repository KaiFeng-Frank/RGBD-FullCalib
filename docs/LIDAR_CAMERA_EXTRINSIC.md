# MID-360S ↔ D435i 雷视外参标定

这条链不在本仓库重写 solver，只负责把硬件发布、静态录包和本机已标 RGB
内参接到官方
[`koide3/direct_visual_lidar_calibration:jazzy`](https://hub.docker.com/r/koide3/direct_visual_lidar_calibration)
镜像。上游是无靶标标定，支持 Livox 这类非重复扫描 LiDAR。

## 当前实现状态

直接雷视流程已实现：`preflight` / 静态联合录包 / 预处理 / 初值 /
求解 / 投影审查 / 导入，以及把自己 rig 的外参交给同一画布查看器。
当前 device/mount 的四项 MID-360S 结果均已完成，融合所需的三项
跨传感器结果由同画布查看器载入：

| 项目 | 结果产物 | 已固结结果 |
|---|---|---|
| MID-360S IMU 内参 | `results/mid360s_imu.json` | 17 姿态（14 fit + 3 holdout）；fit/holdout RMS 0.00962/0.01194 m/s²；accel bias/scale/非正交 + gyro bias + 短窗白噪声 |
| LiDAR–D435i 6DoF 外参 | `results/mid360s_d435i_extrinsic.local.json` | scene06–scene10 五组静态稠密 RGB-D↔LiDAR 配准，inlier median/P90 10.019/16.873 mm，多起点收敛 0.00720° / 0.120 mm |
| LiDAR–内置 IMU | `results/mid360s_lidar_imu.json` | `p_lidar = T_lidar_imu p_imu`；官方定义轴向同基，平移 `[+11.00,+23.29,−44.12] mm` |
| LiDAR–D435i 时间 | `results/mid360s_d435i_timesync.json` | dual-gyro 时偏 −1.940 ms；最终深度时钟方程 `t_depth = t_livox − 5.989 ms` |

MID-360S IMU 噪声数字是 196.156 Hz、0.495805 s 窗的短窗 white-noise
density，明确不声称长时 Allan bias instability/random walk；gyro
scale/非正交仅属于已知角速率转台的通用参考方法。

Livox 点云同时启用扫描末 rotation-only deskew，使用逐点
`offset_time`、内置 IMU 和已知杠杆臂的旋转分量。实扫高旋转主平面
P95 为 70.04→20.39 mm，70/70 帧改善；低旋转对照 17.255→17.262 mm；
同点杠杆臂 ablation 20.21→19.48 mm；`offset_time` 有效率 100%，
IMU 完整覆盖 646/647 帧。该实现不声称平台平移或完整 6DoF deskew。

[`MID360_CALIBRATION.md`](MID360_CALIBRATION.md) 保留给其他设备/部署的通用
严格验收参考；该参考不反向定义上表当前 rig operational 结果为未完成。

## 开源使用口径

每套 D435i + MID-360S 支架都使用绑定自身 device/mount 的外参。D435i 身份从
`results/factory_params.json` 读取；MID-360S 身份从
`/livox/device_info` 读取，驱动不发布该话题时才设 `MID360S_SERIAL`。这些
检查把每份结果精确绑定到它的采集设备与连续安装会话。

通用点云查看器可直接读取用户自己的 operational JSON：

```bash
./view_pointcloud.sh fused --extrinsic /path/to/my_rig_extrinsic.json
```

文件需声明 `status: "operational"`、
`local_schema: "d435i_calib/lidar_camera_extrinsic_local/v1"`、设备 serial、坐标系方向和
`result.T_camera_lidar`。查看器校验这些可移植的几何与身份条件，并将匹配
当前 device/mount 的结果标记为 LOCAL operational。

## 硬件和采集约束

1. MID-360S 和 D435i 必须锁紧在同一刚性支架上。标定期间不得重装、拧螺丝或
   改变两者相对位姿。
2. 采 5–10 个结构和强度纹理丰富的室内/室外场景。每个场景间移动的是
   **整个 rig**；每次开始录包前放稳，15 s 内完全静止。
3. 避免纯白墙、玻璃占满画面、大量运动物体和 LiDAR 近距离盲区。相机与 LiDAR
   视场重叠区内应同时有边缘、材质反射变化和三维深度变化。

首个 bag 会在工作目录生成 `capture_session.json`，固结 rig ID、两个设备
serial 与本次连续安装的 `mount_session_id`。同一工作目录中的所有 bag
必须属于这一次未中断的刚性安装；只要松动、拆卸或重装任一传感器，
必须换新的 `LIDAR_CAMERA_WORK_DIR`，不得将新旧安装的 bag 混用。

上游采集约束见 [Data collection](https://koide3.github.io/direct_visual_lidar_calibration/collection/)。

## 一次性准备

脚本不执行 `sudo`。缺包时它会打印精确安装命令；当前机器需要的完整集合是：

```bash
sudo apt install ros-jazzy-realsense2-camera \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-sqlite3
docker pull koide3/direct_visual_lidar_calibration:jazzy
```

`image_transport_plugins` 是硬前置。录包只保存
`/camera/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)，不保存
1280×720@30 的 raw RGB；这避免 raw RGB 约 83 MB/s 写入只剩有限空间的根分区。
上游 ROS 2 预处理源码会对 topic 名含 `compressed` 的消息按
`sensor_msgs/msg/CompressedImage` 反序列化，再通过 `cv_bridge` 解码，因此不需要
转换回 raw bag。

## 发布器

如果没有现成 ROS 2 发布器，在两个终端前台运行：

```bash
# 终端 1：绑定 factory_params.json 中当前 D435i 的 serial
./calibrate_lidar_camera.sh camera

# 终端 2：Livox xfer_format=0，输出 PointCloud2，10 Hz
./calibrate_lidar_camera.sh lidar-points
```

`camera` 会关闭 depth/infra/gyro/accel，避免无关 USB 与磁盘负载。如果 D435i 正被 HTML
查看器的 `pyrealsense2` 进程占用，先主动关闭那个相机流。
它同样通过受监督启动器运行：启动前按 USB serial 确认就是本机这台
D435i；运行时 serial 连续消失 3 s，或有效且时间戳前进的 RGB 帧连续
5 s 不再到达，就会退出并清理它启动的整个 ROS 进程组。已有同话题
publisher 时直接拒绝，防止旁路数据冒充该驱动的心跳。

`lidar-points` 不会抢占或停掉已有驱动。若 `/livox/lidar` 当前是
`livox_ros_driver2/msg/CustomMsg`，它会明确拒绝；先主动结束原驱动，再切换为
PointCloud2。

它通过同一个受监督启动器运行 Livox：启动后 5 s 收不到有效点云，或网卡
carrier/IP 连续丢失 3 s，就会退出并清理它启动的整个 ROS 进程组；空点云不算
有效心跳，全零/非有限 XYZ 或时间戳不前进的重放点云也不算。设备在启动前
已断开时直接拒绝启动，不留下等待硬件的驱动进程。

默认 RealSense node 为官方 `/camera/camera`（`camera_namespace=camera`，
`camera_name=camera`），因此默认话题为：

- `/camera/camera/color/image_raw/compressed` — `sensor_msgs/msg/CompressedImage`
- `/camera/camera/color/camera_info` — `sensor_msgs/msg/CameraInfo`
- `/livox/lidar` — `sensor_msgs/msg/PointCloud2`

raw `/camera/camera/color/image_raw` 只在 preflight 中读一帧核对实际 1280×720，
不进 bag。

## 录包

```bash
./calibrate_lidar_camera.sh preflight
./calibrate_lidar_camera.sh record scene01
./calibrate_lidar_camera.sh record scene02
./calibrate_lidar_camera.sh record scene03
./calibrate_lidar_camera.sh record scene04
./calibrate_lidar_camera.sh record scene05
```

`preflight` 和每次 `record` 都会检查：

- 三个入包话题的类型与实际消息；
- compressed 流确实为 JPEG transport；
- raw Image 和 CameraInfo 均为 1280×720，RealSense profile 为 1280x720x30；
- depth/infra/gyro/accel 全关；
- `/camera/camera/device_info` 实测 serial 与
  `results/factory_params.json` 中当前 D435i 的 serial 一致；
- `/livox/device_info` 的 JSON `serial_number` 被记录为 MID-360S 身份；若使用的
  PointCloud2 驱动没有该话题，必须显式设置 `MID360S_SERIAL`，绝不从 IP 猜设备身份；
- LiDAR PointCloud2 具有 `x,y,z,intensity` 字段。
- LiDAR `frame_id` 为 `livox_frame`，且在线 serial 必须与本次
  `capture_session.json` 首包冻结的设备一致。
- raw RGB、JPEG RGB、CameraInfo 和 PointCloud2 四个在线话题均只能有一个
  publisher；其 node、GID、type 会作为发布器身份见证写入 manifest。

`record` 会要求人工确认刚性支架与静止状态，固定录 15 s，且绝不覆盖同名
scene。开始前还会打印可用空间，低于 5 GiB 则拒绝录制，不自动删除任何旧数据。
非交互执行需显式加 `--rigid-mounted`。每个 bag 内附带：

- `capture_manifest.json`：两个 serial、rig/mount session、发布器见证、话题
  类型、内参文件 hash 和人工确认；
- `rosbag_info.txt`：话题计数与实际时长；
- `SHA256SUMS`：bag 内文件校验和。

录包结束不只检查“有消息”：在实际记录时长上，JPEG 必须达到至少 20 Hz、
LiDAR 必须达到至少 7 Hz，否则整包保留但判为不完整，不进入求解。

完成时命令会打印该 bag 的实际磁盘大小。数据位于
`data/lidar_camera_extrinsic/bags/<scene>/`。

## 调用官方 solver

```bash
./calibrate_lidar_camera.sh preprocess
./calibrate_lidar_camera.sh initial
./calibrate_lidar_camera.sh solve
./calibrate_lidar_camera.sh view
./calibrate_lidar_camera.sh import
```

`preprocess` **不采用 RealSense CameraInfo 的出厂零畸变**。它会从
`data/cam_rgb-camchain.yaml` 读取本机 Kalibr 自标结果：

```text
model: plumb_bob (Kalibr pinhole-radtan)
K: 884.7801714792454,883.8676458516142,652.1558719536981,373.36376307278374
D: 0.11018893566651113,-0.19834915299940875,-0.0012403776370154334,0.0014777729661525543,0
```

Kalibr radtan 解的是 `k1,k2,p1,p2`；上游 `plumb_bob` CLI 需要
`k1,k2,p1,p2,k3`，因此只把未在 Kalibr 模型中估计的 `k3` 约束为 0，前四项仍是
本机自标值。所有 topic 和参数均显式传入，不使用上游 `-a` 自动猜测。

`preprocess` 还会核对 `calib.json` 中的 bag 名单，并要求每包的 PNG、PLY、
LiDAR index/intensity 图全部非空；这用于防止上游异常返回成功码却只留下半成品。
预处理先写入独立 staging 目录，只在上述完整性检查全部通过后才原子性
更名为 `preprocessed/`。其中 `SOURCE_BAGS.json` 固结每个 bag 的文件集与
SHA-256、设备身份和 mount session；`SOURCE_SOLVER.json` 固结 Docker image ID、
repo digest 与上游 commit。initial/solve/view/import 各阶段发现原始 bag 或
solver 身份改变时会拒绝继续，防止旧派生物被冒充为当前数据的结果。

`initial` 打开上游手工 2D–3D 对应 UI；关窗后必须已保存有限、非零四元数的
`init_T_lidar_camera`。`solve` 调用上游 `calibrate`
后台精配准；`view` 用上游 viewer 审查投影。上游 `--background` 仍会初始化
GLFW，因此 `solve` 也使用同一套 X11 容器参数，并加入 `--ipc host`与
`LIBGL_ALWAYS_SOFTWARE=1`。GUI 容器会优先继承当前
`DISPLAY/XAUTHORITY`，否则自动查找本机 `:1` 和
`/run/user/<uid>/gdm/Xauthority`。默认不传 `--gpus all`；只有已确认 Docker
NVIDIA runtime 可用时才手动设 `DVLC_USE_NVIDIA=1`。
官方镜像的入口脚本从 `/root/ros2_ws` 加载 ROS，因此容器按上游默认用户启动；
每条命令结束后只对挂载的 `preprocessed/` 执行容器内 ownership 归还，宿主机
全程不调用 `sudo`。

上游结果位于 `data/lidar_camera_extrinsic/preprocessed/calib.json`。其
`T_lidar_camera` 的官方定义是把相机系点变到 LiDAR 系：

```text
p_lidar = T_lidar_camera * p_camera
```

在 `view` 投影审查通过前，该数字不应被当作最终 rig 外参。上游命令和方向
定义见 [Program details](https://koide3.github.io/direct_visual_lidar_calibration/programs/)。

## 导入与本地 operational 使用

`import` 不重写变换逻辑，而是直接调用仓库现有的
`tools/import_lidar_camera_extrinsic.py`：

```bash
# 安全默认 rig-id=mid360s-d435i-01，输出文件不存在时才写
./calibrate_lidar_camera.sh import

# 也可显式指定；只有人工确认覆盖时才加 --force
./calibrate_lidar_camera.sh import \
  --rig-id mid360s-d435i-01 \
  --lidar-serial <YOUR_MID360S_SERIAL> \
  --output results/mid360s_d435i_extrinsic.draft.json
```

相机 serial 从当前 `results/factory_params.json` 读取，且导入前再与
录包身份核对。LiDAR serial 优先从每个 bag 的
`capture_manifest.json` 汇总；多包不一致立即失败。老 bag 没有身份字段时，使用
`--lidar-serial` 或 `MID360S_SERIAL`。导入器把完整 `bags/` 目录作为
`--source-data` 做树 hash，同时 hash `capture_session.json`、`SOURCE_BAGS.json`、
`SOURCE_SOLVER.json` 和 RGB camchain，并将上游相机→LiDAR 变换审计后取逆为
项目约定的 LiDAR→相机变换。

默认产物为 `results/mid360s_d435i_extrinsic.draft.json`，不覆盖，并记录实际
Docker repo digest 与上游 source commit。导入器负责 draft/LOCAL 路径；
选择通用严格验收参考的复现者，可以另行生成带覆盖保护的
VALIDATED 产物；该可选层不改写 operational 的生效含义。

对自己未拆动的 rig，完成方向核对和几何投影审查后，可按本项目的
`d435i_calib/lidar_camera_extrinsic_local/v1` 结构保存 `status: operational`
的本地结果，并用 `./view_pointcloud.sh fused --extrinsic PATH` 载入。
`operational` 表示该外参已对记录的 device/mount 生效。

## 可选覆盖

仅在 rig 配置确实不同时使用：

```bash
D435I_CAMERA_NAMESPACE=robot1
D435I_CAMERA_NAME=d435i
LIDAR_CAMERA_IMAGE_TOPIC=/robot1/d435i/color/image_raw/compressed
LIDAR_CAMERA_INFO_TOPIC=/robot1/d435i/color/camera_info
LIDAR_CAMERA_POINTS_TOPIC=/livox/lidar
MID360S_SERIAL=<YOUR_MID360S_SERIAL>
D435I_USB_SERIAL=<OPTIONAL_USB_DESCRIPTOR_SERIAL>
LIDAR_CAMERA_RIG_ID=mid360s-d435i-01
# 仅用于明确给本次连续安装命名；首个 bag 后即固结
LIDAR_CAMERA_MOUNT_SESSION=<unique-session-id>
LIDAR_CAMERA_CAMCHAIN=/absolute/path/to/camchain.yaml
LIDAR_CAMERA_WORK_DIR=/absolute/path/to/workdir
LIDAR_CAMERA_DRAFT_OUTPUT=/absolute/path/to/draft.json
DVLC_IMAGE=koide3/direct_visual_lidar_calibration:jazzy
```

话题或分辨率修改后仍必须通过 `preflight`。因上游靠 topic 名而非消息类型
选择解码分支，自定义 `LIDAR_CAMERA_IMAGE_TOPIC` 也必须含小写 `compressed`。脚本不会静默退回 raw RGB、CustomMsg
或出厂零畸变。
