# D435i 硬件立体校正验收协议

**冻结日期：2026-08-31，协议版本：v1。**

主验收产物固定为 `results/stereo_rectification_validation.json`，
由 `tools/validate_stereo_rectification.py` 生成。

本验收只回答一个问题：D435i serial `947122070908`、firmware
`5.12.7.100` 在 1280×720 Y8 模式下由 librealsense 直接交付的 IR1/IR2
图像，是否已经水平校正到可供行约束立体匹配使用。它不生成新的
传感器参数，也不改写 D435i 的硬件校正。

## 冻结输入

- 已有标定只用 `data/cam_ir-camchain.yaml` 做溯源，不用它计算
  原始交付图的主验收指标。
- 独立验证集固定为 `data/cam_trio_frames/` 的全部 16 对：
  `NNNN.png` 为 IR1，`NNNN_r.png` 为 IR2。
- `data/cam_trio-camchain.yaml` 严禁参与匹配、拟合、校正或阈值调整。
- 16 对全部进入统计；不得根据结果手工删帧或删角点。

## 冻结检测与配对

- OpenCV `DICT_APRILTAG_36h11`。
- 只接受本 6×6 AprilGrid 的 tag ID `0..35`；该范围之外的同字典
  标记不属于验收靶标。
- adaptive threshold window `3..15`, step `2`。
- `CORNER_REFINE_SUBPIX`，window `5`。
- 左右目只按 `(tag_id, canonical_corner_index)` 配对；禁止最近邻、
  画面顺序重排或人工关联。
- 主指标直接在 librealsense 交付图上计算，不做 undistort、
  `stereoRectify` 或 remap。

对每个配对角点定义带符号垂直差 `dy = y_IR1 - y_IR2`，主误差
`e_y = |dy|`。报告 signed median、`|dy|` 的 median/P95/P99/max、
`|dy| > 1 px` 与 `|dy| > 2 px` 比例，以及每帧 P95。单个 max
对角点亚像素误检过敏，只报告而不设门。
分位数统一用 NumPy 默认的 linear interpolation 定义。

## 充分性门

任一项不足都只能判 `INSUFFICIENT`，不得判 PASS 或 FAIL：

1. 至少 `12 / 16` 对图有共同 tag。
2. 至少 `600` 个配对角点。
3. 配对角点中点在 3×3 画面网格中至少覆盖 `6` 格，且左右各
   至少覆盖 2 列、上下各至少覆盖 2 行。

## PASS 门

充分性通过后，以下四项必须同时成立：

1. `median(|dy|) <= 1.0 px`
2. `P95(|dy|) <= 1.5 px`
3. `P99(|dy|) <= 2.0 px`
4. `fraction(|dy| > 2 px) <= 1%`

这是对原始交付 IR 对的验收，不是对事后软件校正结果的验收。
可以额外报告用 `cam_ir` 参数计算的 software-rectified 残差作诊断，
但该数字绝不参与 PASS。

## 作用边界

- 当前 RGB-D 主链使用 color + rectified/aligned depth，本验收不阻塞该主链。
- 只有启用 IR1+IR2 Stereo/Stereo-Inertial 前端时，FAIL 才是直接部署阻断。
- 结论只绑定上述 serial、firmware、分辨率与流模式。
