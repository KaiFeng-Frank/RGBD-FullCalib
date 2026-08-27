#!/usr/bin/env python3
"""阶段 0:摸清这台 D435i —— USB 链路、可用流、出厂内外参、IMU。

自适应 USB2/USB3:不写死分辨率和帧率,从设备实际支持的 profile 里挑。
"""
import glob
import os
import sys

import pyrealsense2 as rs


def sep(t):
    print(f"\n{'='*66}\n{t}\n{'='*66}")


def usb_link_report(serial):
    """从 sysfs 读真实协商速率 —— librealsense 的 usb_type_descriptor 会撒谎的场合更少,
    但 sysfs 能额外告诉你设备挂在哪条链路、串了几级 hub。"""
    for d in glob.glob('/sys/bus/usb/devices/*/'):
        try:
            if open(d + 'idVendor').read().strip() != '8086':
                continue
        except OSError:
            continue
        node = os.path.basename(d.rstrip('/'))
        speed = open(d + 'speed').read().strip()
        ver = open(d + 'version').read().strip()
        depth = node.count('.')
        print(f"  sysfs 节点     {node}   (经过 {depth} 级 hub)")
        print(f"  协商速率       {speed} Mbps      bcdUSB {ver}")
        if float(speed) < 5000:
            print("  ⚠ 跑在 USB2 —— IR2 流会被砍掉,高帧率不可用")
            print("    多半是线缆只有 USB2 芯线;换 D435i 原装 USB-C 3.x 线")
        else:
            print("  ✓ USB3 链路正常")
        return float(speed)
    return None


def profiles_of(dev):
    """{(stream, index): [(w,h,fmt,fps), ...]}"""
    out = {}
    for s in dev.query_sensors():
        for p in s.get_stream_profiles():
            st, idx = p.stream_type(), p.stream_index()
            try:
                vp = p.as_video_stream_profile()
                out.setdefault((st, idx), []).append((vp.width(), vp.height(), p.format(), p.fps()))
            except Exception:
                out.setdefault((st, idx), []).append((0, 0, p.format(), p.fps()))
    return out


def pick(profs, key, fmt, prefer_res, prefer_fps=(30, 15, 10, 6)):
    """在设备真实支持的组合里挑一个:先按分辨率偏好,再按帧率偏好。"""
    cands = [c for c in profs.get(key, []) if c[2] == fmt]
    if not cands:
        return None
    for res in prefer_res:
        for fps in prefer_fps:
            for w, h, f, r in cands:
                if (w, h) == res and r == fps:
                    return (w, h, fmt, fps)
    return max(cands, key=lambda c: (c[0] * c[1], c[3]))


def main():
    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        print("没有检测到 RealSense 设备。")
        print("  相机插了吗:      lsusb | grep 8086")
        print("  udev 装了吗:      ls /etc/udev/rules.d/99-realsense-libusb.rules")
        sys.exit(1)

    dev = devs[0]
    sep("设备")
    for k in [rs.camera_info.name, rs.camera_info.serial_number,
              rs.camera_info.firmware_version, rs.camera_info.usb_type_descriptor,
              rs.camera_info.product_line]:
        try:
            print(f"  {str(k):30s} {dev.get_info(k)}")
        except Exception:
            pass

    sep("USB 链路")
    speed = usb_link_report(dev.get_info(rs.camera_info.serial_number))

    profs = profiles_of(dev)
    sep("这条链路上实际可用的流")
    have_ir2 = (rs.stream.infrared, 2) in profs
    for (st, idx), lst in sorted(profs.items(), key=lambda x: str(x[0])):
        res = sorted({(w, h) for w, h, _, _ in lst if w})
        fps = sorted({r for _, _, _, r in lst})
        nm = f"{st}" + (f"[{idx}]" if idx else "")
        print(f"  {nm:22s} 分辨率={res if res else '(非图像)'}")
        print(f"  {'':22s} 帧率={fps}")
    if not have_ir2:
        print("\n  ⚠ 没有 Infrared[2] —— 阶段 2(双目 IR 标定)在当前链路下无法进行")

    # ---- 按实际能力组装 pipeline ----
    RES = [(848, 480), (1280, 720), (640, 480)]
    want = {}
    for key, fmt, tag in [((rs.stream.color, 0), rs.format.bgr8, 'COLOR'),
                          ((rs.stream.depth, 0), rs.format.z16, 'DEPTH'),
                          ((rs.stream.infrared, 1), rs.format.y8, 'IR-LEFT'),
                          ((rs.stream.infrared, 2), rs.format.y8, 'IR-RIGHT')]:
        c = pick(profs, key, fmt, RES)
        if c:
            want[key] = (c, tag)

    cfg = rs.config()
    cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
    for (st, idx), ((w, h, fmt, fps), tag) in want.items():
        if st == rs.stream.infrared:
            cfg.enable_stream(st, idx, w, h, fmt, fps)
        else:
            cfg.enable_stream(st, w, h, fmt, fps)

    sep("出厂内参 —— 这是我们标定结果要去对的标尺")
    print("  请求的配置: " + ",  ".join(
        f"{tag} {w}x{h}@{fps}" for (w, h, fmt, fps), tag in want.values()))
    pipe = rs.pipeline()
    try:
        prof = pipe.start(cfg)
    except RuntimeError as e:
        print(f"\n  启动失败: {e}")
        print("  这通常意味着几条流在当前 USB 带宽下无法共存,试试逐条降分辨率")
        sys.exit(1)

    got = {}
    for (st, idx), (_, tag) in want.items():
        try:
            sp = prof.get_stream(st, idx) if st == rs.stream.infrared else prof.get_stream(st)
            i = sp.as_video_stream_profile().get_intrinsics()
            got[tag] = sp
            print(f"\n[{tag}]  {i.width}x{i.height}   畸变模型={i.model}")
            print(f"   fx={i.fx:10.4f}   fy={i.fy:10.4f}")
            print(f"   cx={i.ppx:10.4f}   cy={i.ppy:10.4f}")
            print(f"   dist={[round(c, 6) for c in i.coeffs]}")
        except Exception as e:
            print(f"\n[{tag}] 取内参失败: {e}")

    sep("出厂外参")
    pairs = [('IR-LEFT', 'IR-RIGHT', 'IR左 -> IR右 (立体基线)'),
             ('DEPTH', 'COLOR', 'DEPTH -> COLOR (对齐用)'),
             ('IR-LEFT', 'COLOR', 'IR左 -> COLOR')]
    for a, b, tag in pairs:
        if a not in got or b not in got:
            print(f"\n[{tag}]  跳过(缺 {a if a not in got else b})")
            continue
        e = got[a].get_extrinsics_to(got[b])
        t, R = e.translation, e.rotation
        n = sum(v * v for v in t) ** 0.5 * 1000
        print(f"\n[{tag}]")
        print(f"   平移 (m) [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}]    |t| = {n:.2f} mm")
        print(f"   旋转     [{R[0]:+.5f} {R[3]:+.5f} {R[6]:+.5f}]")
        print(f"            [{R[1]:+.5f} {R[4]:+.5f} {R[7]:+.5f}]")
        print(f"            [{R[2]:+.5f} {R[5]:+.5f} {R[8]:+.5f}]")

    try:
        ds = prof.get_device().first_depth_sensor()
        sc = ds.get_depth_scale()
        print(f"\n[深度单位] {sc} m/LSB  (depth 图里整数 1 = {sc*1000:.3f} mm)")
    except Exception:
        pass
    pipe.stop()

    sep("IMU —— 出厂不提供 cam-IMU 外参,这是阶段 4 要标的")
    acc = profs.get((rs.stream.accel, 0), [])
    gyr = profs.get((rs.stream.gyro, 0), [])
    if acc and gyr:
        print(f"  ✓ accel  {sorted({r for *_, r in acc})} Hz")
        print(f"  ✓ gyro   {sorted({r for *_, r in gyr})} Hz")
        print("  IMU 走 HID 通道,不占视频带宽 —— USB2 也是满速")
        print("  librealsense 只给一个粗略机械外参,没有噪声模型、没有时间偏移")
    else:
        print("  ✗ 没有 IMU —— 这台可能是 D435(不带 i)")

    sep("阶段可行性")
    ok3 = speed is None or speed >= 5000
    rows = [("1  RGB 内参", True, ""),
            ("2  双目 IR 内外参", have_ir2, "" if have_ir2 else "缺 IR2 流"),
            ("3  深度对齐/偏置", True, ""),
            ("4  cam-IMU 外参", bool(acc and gyr),
             "" if have_ir2 else "只能单目 IR + IMU")]
    for name, ok, note in rows:
        print(f"  {'✅' if ok else '❌'}  {name:22s} {note}")


if __name__ == '__main__':
    main()
