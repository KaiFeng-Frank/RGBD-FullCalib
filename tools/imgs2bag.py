#!/usr/bin/env python3
"""把 capture.py 落盘的 PNG 帧重建成 ROS1 bag。采集数据的第二道保险。"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import BagWriter


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames_dir')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--topic', required=True)
    args = ap.parse_args()

    idx = os.path.join(args.frames_dir, 'times.txt')
    if not os.path.exists(idx):
        print(f"找不到 {idx}"); sys.exit(1)

    rows = [l.split() for l in open(idx) if l.strip()]
    n = 0
    with BagWriter(args.out) as bag:
        for name, t in rows:
            img = cv2.imread(os.path.join(args.frames_dir, name), cv2.IMREAD_UNCHANGED)
            if img is None:
                print(f"  跳过读不出的 {name}"); continue
            enc = 'bgr8' if img.ndim == 3 else 'mono8'
            bag.write_image(args.topic, float(t), img, enc)
            n += 1
    print(f"重建完成: {args.out}  {n} 帧  topic={args.topic}")


if __name__ == '__main__':
    main()
