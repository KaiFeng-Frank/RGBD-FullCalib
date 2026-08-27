#!/usr/bin/env python3
"""浏览器 <-> 服务端的线格式。

设计取舍:深度图按 uint16 原样发,不做任何编码。
  848x480x2B = 814KB/帧,@10fps = 8MB/s —— 本机 WebSocket 毫无压力,
  而任何有损压缩都会在深度上制造假的几何(JPEG 的块效应会变成点云上的台阶)。
  真要跨网络传再上无损压缩(zlib/zstd),别上图像编码器。

彩色图走 JPEG:它只用来贴颜色,有损无所谓。

每帧 = 1 字节类型 + 定长头 + 负载,前端用 DataView 直接读,不做 JSON 解析。
"""
import json
import struct

T_META = 0        # JSON:内参、深度单位、源信息
T_DEPTH = 1       # uint16 深度图
T_COLOR = 2       # JPEG
T_POINTS = 3      # float32 xyz(+可选 intensity)—— 给 Mid-360 这类原生点云源
T_STATS = 4       # JSON:帧率等运行时统计
T_IR = 5          # JPEG:红外原图(唯一能看到散斑的视图)

# 头部布局(网络序 big-endian)。
# 关键:头长度必须让负载落在对齐边界上 —— JS 的 TypedArray 从非对齐
# offset 构造会直接抛 RangeError,而不是像 C 那样只是慢一点。
#   depth  负载 uint16  -> 头 24 字节(8 的倍数,顺带让 double 也对齐)
#   color  负载 字节流   -> 头 20 字节
#   points 负载 float32 -> 头 20 字节(4 的倍数)
_H_DEPTH = struct.Struct('>B3xHHIdf')    # 24: type,pad,w,h,seq,t,depth_scale
_H_COLOR = struct.Struct('>B3xHHId')     # 20: type,pad,w,h,seq,t (color 与 ir 共用)
_H_POINTS = struct.Struct('>BB2xIId')    # 20: type,has_intensity,pad,count,seq,t
_H_IR = struct.Struct('>BB2xHHId')       # 20: type,laser_on,pad,w,h,seq,t


def pack_meta(meta):
    return bytes([T_META]) + json.dumps(meta).encode()


def pack_stats(stats):
    return bytes([T_STATS]) + json.dumps(stats).encode()


def pack_depth(seq, t, arr, depth_scale):
    h, w = arr.shape
    return _H_DEPTH.pack(T_DEPTH, w, h, seq, t, depth_scale) + arr.tobytes()


def pack_color(seq, t, w, h, jpeg_bytes):
    return _H_COLOR.pack(T_COLOR, w, h, seq, t) + jpeg_bytes


def pack_ir(seq, t, w, h, jpeg_bytes, laser_on=1):
    """laser_on: 该帧发射器是否点亮。交替模式下用它区分散斑帧与干净帧。"""
    return _H_IR.pack(T_IR, int(laser_on), w, h, seq, t) + jpeg_bytes


def pack_points(seq, t, xyz, intensity=None):
    """xyz: (N,3) float32;intensity: (N,) float32 或 None"""
    n = len(xyz)
    head = _H_POINTS.pack(T_POINTS, 1 if intensity is not None else 0, n, seq, t)
    body = xyz.astype('<f4').tobytes()
    if intensity is not None:
        body += intensity.astype('<f4').tobytes()
    return head + body
