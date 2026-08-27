# D435i 标定实录(Field Guide)

**一台 Intel RealSense D435i 的完整标定实录 —— 每个参数都带外部参照判决,每个坑都写了下来。**

[English README](README.md) · 完整实录 [CALIBRATION.md](CALIBRATION.md) ·
误差影响分析 [IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md)

![判决卡片](docs/img/verdict_card.png)

标定这行最怕的不是误差大,是**看起来漂亮但错得一致**。重投影误差只说明模型拟合了
喂给它的数据,仅此而已。所以这里每个结果都对照一个**优化之外**的参照:出厂参数、
当地重力、多次独立采集、一把尺子。参数过不了核查,工具就直说 —— 上图那行红的就是
本机的 cam-IMU 平移,判决是"**不要冻结这个数,交给 VIO 在线估计**"。

## 这是什么(不是什么)

**是**:一台相机从头到尾标完的 working example(RGB 内参 → IR 双目 → RGB↔IR 外参 →
深度噪声模型 → cam-IMU → 加速度计内参 → 温漂模型),带全套工具、判决层、GUI、
和 15+ 条花了真实时间踩出来的坑。

**不是**(还不是):万能标定套件。脚本为 D435i 写,只在一台机器上验证过。
代码不一定能直接迁移,但**方法**(判决层 + 外部参照)在任何传感器上都成立。

## 结果速览

七项标定的参数、外部核查、判决,见 [English README](README.md#results-at-a-glance)
的速览表或 [CALIBRATION.md](CALIBRATION.md) 全文。机器可读结果在
[`data/*.yaml`](data) 与 [`results/*.json`](results)。

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

## GUI

WebGL2 点云查看器,三页签:实时点云(16-bit 深度 WebSocket 流、shader 内反投影)、
标定结果(判决卡片,直接从 yaml/json 生成)、待办占位。温漂补偿一个勾选框
直接作用到实时点云。

```bash
cd viewer
python server.py --source d435i --alt-emitter   # 实机 + 发射器交替帧
python server.py --source synthetic             # 无相机也能跑
# 浏览器打开 http://localhost:8080    (?static=1 是可截图的静态模式)
```

![标定页](docs/img/calib_page.png)

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
图内定位。它还把待办标定按影响重排了序:只标你的应用真正感觉得到的量,能省几天。

## License

MIT — 见 [LICENSE](LICENSE)。
