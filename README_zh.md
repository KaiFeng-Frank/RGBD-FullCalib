# RGBD-FullCalib

**RGBD-FullCalib 是一套开源的一站式多传感器标定工作台，覆盖相机、
IMU、LiDAR 与跨传感器外参，把标定输出变成可视化、机器可读且带明确
部署范围的结果。v0.1 交付一台 D435i 端到端标定；v0.2.0 交付 24 条
声明式判决规则；v0.3 现已实现直接雷视链、MID-360S 原生查看和
D435i/MID-360S 单画布融合。**

[English README](README.md) · 完整实录 [CALIBRATION.md](CALIBRATION.md) ·
误差影响分析 [IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md) ·
[ROADMAP](ROADMAP.md) · [CHANGELOG](CHANGELOG.md)

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

**v0.3（已实现）**：viewer 可直接渲染任意 ROS 2
`sensor_msgs/msg/PointCloud2` 与 Livox `CustomMsg`；直接雷视流程从采集、
求解到导入全链打通；D435i/MID-360S 实时点云在同一彩色相机坐标系
画布中融合。文档 rig 使用五场景 operational 外参。

### 扩展到你的相机

欢迎其他 RGB-D / 视惯 rig 提交移植记录。新适配器以真实设备轨迹为依据，
共用标定链与判决链保持稳定。

## 结果速览

七项标定的参数、外部核查、判决,见 [English README](README.md#results-at-a-glance)
的速览表或 [CALIBRATION.md](CALIBRATION.md) 全文。机器可读结果在
[`data/*.yaml`](data) 与 [`results/*.json`](results)。
文档 MID-360S→D435i rig 已生成五场景
[当前 rig 可用外参](results/mid360s_d435i_extrinsic.local.json)。该外参属于文档中的
刚性安装；其他 rig 或重新拆装后通过同一流程生成自己的外参。

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
REPORT.md、GUI 卡片同一事实源。缺文件显示为"待数据",标了一半也能出报告。
加设备的判决项 = 写规则,不是 fork 判决代码。

机器检验散文第一天就有收获:旧文案"五次独立测量基线"被抓出**假独立** ——
Kalibr 的 imu-camera 阶段逐位继承相机链,五个数里三个是复制品。真实证据是
"两次独立标定(不同 bag)",差值 0.005 mm;yaml 里留了注释讲原因。

## GUI

WebGL2 点云查看器,三页签:实时点云(16-bit 深度图在 shader 内反投影,
或原生 xyz/intensity/RGB 点流走 VBO)、
标定结果(判决卡片,直接从 yaml/json 生成;每张卡带两枚 SLAM 徽章:**能否在线标定**
(外参旋转/时移/IMU 零偏是 VINS 类系统的标准在线状态,内参与深度链不是)与
**对 SLAM 影响定级**,每一级背后是实测传播数字)、后续标定规划。温漂补偿一个勾选框
直接作用到实时点云。

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
`results/mid360s_d435i_extrinsic.local.json` 自动载入；其他 rig 用
`--extrinsic PATH` 指定自己的结果。
[通用雷视外参流程](docs/LIDAR_CAMERA_EXTRINSIC.md)已覆盖采集、求解、
结果方向、导入与融合查看。

统一入口打开 `http://localhost:8080`，隔离 ROS Jazzy 与 Conda，
任一实时流断开时会同步停止两个后端。ROS 2 点云按消息 frame
渲染当前帧。

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

## 路线图速览

**v0.2.0(已交付)** 声明式判决引擎 · **v0.3(当前)** Mid-360 原生查看、
直接雷视标定与单画布融合 ·
**v0.4** 更多 RGB-D 家族,rig 变描述文件 · **v0.5** 影响分析器工具化(输入你的工况,
输出你的一阶误差项) · **v1.0** 引导式采集 + 标定回归检测 + 按症状搜索的坑库 ——
工作台成形,届时仓库改名(GitHub 自动重定向)。详见 [ROADMAP.md](ROADMAP.md)。

## License

MIT — 见 [LICENSE](LICENSE)。
