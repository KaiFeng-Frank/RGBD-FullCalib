#!/usr/bin/env python3
"""IR 下的靶标诊断:把板子举在相机前不动,跑一次,看它到底能不能被看见。

扫曝光档位,每档报告 tag 检测数、板面黑白对比、模糊程度,并存下最好的一帧。
"""
import os, sys, time
import numpy as np, cv2, pyrealsense2 as rs

W, H, FPS = 1280, 720, 15
EXPS = [4000, 8000, 16000, 33000, 0]      # 0 = 自动
_p = cv2.aruco.DetectorParameters()
_p.adaptiveThreshWinSizeMin, _p.adaptiveThreshWinSizeMax, _p.adaptiveThreshWinSizeStep = 3, 15, 2
det = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), _p)

pipe = rs.pipeline(); cfg = rs.config()
cfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
prof = pipe.start(cfg)
# D435i 的 Stereo Module 和 RGB Camera 都 supports(exposure),
# 循环到最后拿到的是 RGB —— 调它对 IR 流毫无影响。必须认准深度/立体那个。
sens = None
for s in prof.get_device().query_sensors():
    nm = s.get_info(rs.camera_info.name)
    if s.supports(rs.option.emitter_enabled):
        s.set_option(rs.option.emitter_enabled, 0)
    if 'Stereo' in nm or s.is_depth_sensor():
        sens = s
print(f"曝光控制目标: {sens.get_info(rs.camera_info.name) if sens else '未找到'}")
K = prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile().get_intrinsics()

print(f"IR {W}x{H}@{FPS}  fx={K.fx:.1f}   把板子举在相机前,保持不动\n")
print(f"{'曝光':>8} {'tag':>6} {'亮度':>6} {'对比':>6} {'清晰度':>8} {'过曝%':>7}")
best = (-1, None, None, None, None)
for e in EXPS:
    if sens:
        try:
            if e:
                sens.set_option(rs.option.enable_auto_exposure, 0)
                r = sens.get_option_range(rs.option.exposure)
                sens.set_option(rs.option.exposure, float(min(max(e, r.min), r.max)))
                if sens.supports(rs.option.gain):
                    sens.set_option(rs.option.gain, 100.0)
            else:
                sens.set_option(rs.option.enable_auto_exposure, 1)
        except Exception as ex:
            print(f"  设曝光 {e} 失败 {ex}")
    for _ in range(12):                    # 等生效
        fs = pipe.wait_for_frames(5000)
    a = np.asanyarray(fs.get_infrared_frame(1).get_data())
    c, i, _ = det.detectMarkers(a)
    n = 0 if i is None else len(i)
    lap = cv2.Laplacian(a, cv2.CV_64F).var()        # 清晰度:越大越锐
    p5, p95 = np.percentile(a, [5, 95])
    lbl = f"{e/1000:.0f}ms" if e else "自动"
    print(f"{lbl:>8} {n:>6} {np.median(a):>6.0f} {p95-p5:>6.0f} {lap:>8.0f} {(a>250).mean()*100:>6.1f}%")
    if n > best[0]:
        best = (n, a.copy(), c, i, lbl)

pipe.stop()
n, a, c, i, lbl = best
v = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
if n:
    cv2.aruco.drawDetectedMarkers(v, c, i)
    q = np.concatenate([x.reshape(4, 2) for x in c])
    side = np.median([np.linalg.norm(x.reshape(4, 2)[j] - x.reshape(4, 2)[(j+1) % 4])
                      for x in c for j in range(4)])
    print(f"\n最好一档 {lbl}: tag {n}/36  边长 {side:.1f}px  估计距离 {K.fx*0.0352/side:.2f}m")
    print(f"  板子跨度 {q[:,0].max()-q[:,0].min():.0f}x{q[:,1].max()-q[:,1].min():.0f} px  (画面 {W}x{H})")
else:
    print(f"\n所有曝光档位都是 0 tag")
# 原图也存一份 —— 画了标记的图没法再做参数实验
cv2.imwrite(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'results', 'ir_diag_raw.png'), a)
cv2.rectangle(v, (0, 0), (W, 40), (0, 0, 0), -1)
cv2.putText(v, f"IR  best={lbl}  tags {n}/36", (12, 29),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
cv2.imwrite(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'results','ir_diag.png'), v)
print('截图 -> results/ir_diag.png')
