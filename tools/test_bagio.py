#!/usr/bin/env python3
"""bagio 的无头自测。目的:把「写 bag」这一步的正确性从采集流程里摘出来验证,
不必靠用户举板子来暴露 bug。"""
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import BagWriter, TS

from rosbags.rosbag1 import Reader


def main():
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, 'selftest.bag')

    rng = np.random.default_rng(0)
    imgs_bgr = [rng.integers(0, 255, (48, 64, 3), dtype=np.uint8) for _ in range(5)]
    imgs_mono = [rng.integers(0, 255, (48, 64), dtype=np.uint8) for _ in range(5)]

    t0 = 1700000000.25
    with BagWriter(path) as bag:
        for i, im in enumerate(imgs_bgr):
            bag.write_image('/cam2/image_raw', t0 + i * 0.25, im, 'bgr8')
        for i, im in enumerate(imgs_mono):
            bag.write_image('/cam0/image_raw', t0 + i * 0.25, im, 'mono8')
        for i in range(20):
            t = t0 + i * 0.005
            bag.write_imu('/imu0', t, (0.01 * i, -0.02, 0.03), (0.0, 0.0, 9.81))

    print(f"写入 {os.path.getsize(path)} 字节")

    ok = True
    with Reader(path) as r:
        got = {c.topic: c.msgcount for c in r.connections}
        print("  topic 计数:", got)
        for topic, want in [('/cam2/image_raw', 5), ('/cam0/image_raw', 5), ('/imu0', 20)]:
            if got.get(topic) != want:
                print(f"  ✗ {topic} 期望 {want} 实得 {got.get(topic)}"); ok = False

        # 反序列化第一帧彩色图,逐字节比对
        for conn, ts, raw in r.messages():
            if conn.topic != '/cam2/image_raw':
                continue
            m = TS.deserialize_ros1(raw, conn.msgtype)
            back = np.asarray(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            print(f"  首帧: {m.width}x{m.height} {m.encoding} step={m.step} "
                  f"seq={m.header.seq} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d}")
            if not np.array_equal(back, imgs_bgr[0]):
                print("  ✗ 像素数据往返不一致"); ok = False
            if m.step != m.width * 3:
                print(f"  ✗ step 错误"); ok = False
            if abs((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) - t0) > 1e-6:
                print("  ✗ 时间戳往返不一致"); ok = False
            break

        for conn, ts, raw in r.messages():
            if conn.topic != '/imu0':
                continue
            m = TS.deserialize_ros1(raw, conn.msgtype)
            print(f"  首条IMU: w=({m.angular_velocity.x:.3f},{m.angular_velocity.y:.3f},"
                  f"{m.angular_velocity.z:.3f})  a=({m.linear_acceleration.x:.2f},"
                  f"{m.linear_acceleration.y:.2f},{m.linear_acceleration.z:.2f})")
            if abs(m.linear_acceleration.z - 9.81) > 1e-6:
                print("  ✗ IMU 数据错误"); ok = False
            break

    print("\n" + ("✅ bagio 自测通过" if ok else "❌ bagio 自测失败"))
    print(f"测试 bag 保留在: {path}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
