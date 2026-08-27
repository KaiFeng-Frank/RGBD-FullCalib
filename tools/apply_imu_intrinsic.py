#!/usr/bin/env python3
"""把加速度计内参应用到 bag:a_true = M · (a_meas − b)

只校正加速度计。陀螺的标度因子需要转台给已知角速度才能标,这里不动它。
图像 topic 原样复制,时间戳不变。
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import BagWriter, TS
from rosbags.rosbag1 import Reader


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_in')
    ap.add_argument('bag_out')
    ap.add_argument('--model', default=None)
    ap.add_argument('--imu-topic', default='/imu0')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    model_p = args.model or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'results', 'imu_intrinsic.json')
    if os.path.exists(args.bag_out):
        if args.force:
            os.remove(args.bag_out)
        else:
            print(f"{args.bag_out} 已存在,加 --force"); sys.exit(1)

    m = json.load(open(model_p))
    M = np.array(m['M']); b = np.array(m['bias_ms2'])
    print(f"内参模型 {model_p}")
    print(f"  scale {np.round(m['scale'],6)}   bias {np.round(b,5)}")
    print(f"  非正交 {np.round(np.degrees(m['misalign_rad']),4)}°   当地重力 {m['gravity_local']:.5f}")

    n_imu = n_img = 0
    raw_norm, cor_norm = [], []
    with Reader(args.bag_in) as r, BagWriter(args.bag_out) as w:
        for conn, ts, raw in r.messages():
            msg = TS.deserialize_ros1(raw, conn.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if conn.topic == args.imu_topic:
                a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                              msg.linear_acceleration.z])
                g_ = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
                ac = M @ (a - b)
                raw_norm.append(np.linalg.norm(a)); cor_norm.append(np.linalg.norm(ac))
                w.write_imu(conn.topic, t, g_, ac)
                n_imu += 1
            elif 'image' in conn.topic:
                arr = np.asarray(msg.data, np.uint8)
                enc = msg.encoding
                arr = arr.reshape(msg.height, msg.width, 3) if enc == 'bgr8' else \
                      arr.reshape(msg.height, msg.width)
                w.write_image(conn.topic, t, arr, enc)
                n_img += 1
    raw_norm = np.array(raw_norm); cor_norm = np.array(cor_norm)
    g = m['gravity_local']
    print(f"\n写入 {args.bag_out}")
    print(f"  IMU {n_imu} 条   图像 {n_img} 帧")
    print(f"  |a| 校正前 {raw_norm.mean():.5f} ± {raw_norm.std():.5f}")
    print(f"      校正后 {cor_norm.mean():.5f} ± {cor_norm.std():.5f}   (当地重力 {g:.5f})")
    print(f"  注:含运动段,|a| 不会恰好等于 g;看的是离散程度有没有变小")


if __name__ == '__main__':
    main()
