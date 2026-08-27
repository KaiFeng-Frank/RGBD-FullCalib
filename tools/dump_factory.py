#!/usr/bin/env python3
"""把设备内的出厂标定参数固化成机器可读文件。

判决层的核心是"外部参照":每个自标定参数都要和一个优化之外的数对照。
出厂参数就是最重要的一组参照 —— 但它们活在设备 flash 里,散文引用会腐烂。
这里一次性读出存 JSON,规则文件用 cross 引用,数据有出处、可追溯。
"""
import json
import os

import numpy as np
import pyrealsense2 as rs


def main():
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'results', 'factory_params.json')
    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pr = pipe.start(cfg)
    dev = pr.get_device()

    def intr(stream, idx=0):
        p = pr.get_stream(stream, idx) if idx else pr.get_stream(stream)
        i = p.as_video_stream_profile().get_intrinsics()
        return dict(fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy,
                    w=i.width, h=i.height, model=str(i.model), coeffs=list(i.coeffs))

    ir1 = pr.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
    ir2 = pr.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
    col = pr.get_stream(rs.stream.color).as_video_stream_profile()

    def extr(a, b):
        e = a.get_extrinsics_to(b)
        return dict(t_mm=[x * 1000 for x in e.translation],
                    R=[list(e.rotation[i*3:i*3+3]) for i in range(3)])

    d = dict(
        device=dev.get_info(rs.camera_info.name),
        serial=dev.get_info(rs.camera_info.serial_number),
        fw=dev.get_info(rs.camera_info.firmware_version),
        rgb_1280x720=intr(rs.stream.color),
        ir1_1280x720=intr(rs.stream.infrared, 1),
        T_ir2_to_ir1=extr(ir2, ir1),
        T_ir1_to_rgb=extr(ir1, col),
        baseline_mm=abs(extr(ir2, ir1)['t_mm'][0]),
    )
    pipe.stop()
    # 深度流内参(848x480)单独一组:与三相机组合会 Couldn't resolve requests
    p3 = rs.pipeline(); c3 = rs.config()
    c3.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    pr3 = p3.start(c3)
    i3 = pr3.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    d['depth_848x480'] = dict(fx=i3.fx, fy=i3.fy, cx=i3.ppx, cy=i3.ppy)
    d['depth_scale_mm'] = pr3.get_device().first_depth_sensor().get_depth_scale() * 1000
    p3.stop()
    # IMU 外参(机械设计值)
    try:
        p2 = rs.pipeline(); c2 = rs.config()
        c2.enable_stream(rs.stream.accel)
        c2.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
        pr2 = p2.start(c2)
        acc = pr2.get_stream(rs.stream.accel)
        ir1b = pr2.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        d['T_ir1_to_imu'] = extr(ir1b, acc)
        p2.stop()
    except Exception as e:
        d['T_ir1_to_imu'] = f'unavailable: {e}'
    json.dump(d, open(out, 'w'), indent=2, ensure_ascii=False)
    print(f"写入 {out}")
    print(f"  基线 {d['baseline_mm']:.3f} mm   IR1->RGB |t| "
          f"{np.linalg.norm(d['T_ir1_to_rgb']['t_mm']):.3f} mm")
    if isinstance(d['T_ir1_to_imu'], dict):
        print(f"  IR1->IMU t = {[round(x,2) for x in d['T_ir1_to_imu']['t_mm']]} mm")


if __name__ == '__main__':
    main()
