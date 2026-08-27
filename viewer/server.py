#!/usr/bin/env python3
"""D435i / Mid-360 实时点云查看器 —— 服务端。

  python server.py                    # 真机
  python server.py --source synthetic # 合成场景(相机被占用时也能开发/验证)
  然后浏览器打开 http://localhost:8080

丢帧而非排队:
  源线程只把"最新一帧"写进 latest,广播协程按自己的节奏取。
  客户端慢下来时它看到的是当前画面,而不是一条越拖越长的历史队列 ——
  实时查看器里,迟到的正确帧不如准时的近似帧。
"""
import argparse
import asyncio
import functools
import http.server
import json
import os
import socketserver
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import protocol as P
import calib_summary

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def load_thermal_model():
    """读温漂补偿模型。没有就返回 None —— 界面上会显示"未标定",不影响其它功能。"""
    p = os.path.join(ROOT, 'results', 'thermal_model.json')
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            return json.load(f)
    except Exception:
        return None


def thermal_correction(model, T):
    """给定当前 ASIC 温度,算出各项补偿量。

    模型存的是"测量值随温度怎么变",补偿要反向施加:
    深度实测随温度变小 -> 补偿时要加回去。
    """
    if not model or T is None or not np.isfinite(T):
        return None
    dT = float(T) - float(model.get('T_ref_C', 25.0))
    out = {'T': float(T), 'T_ref': float(model.get('T_ref_C', 25.0)), 'dT': dT}
    if 'depth_ppm_per_C' in model:
        out['depth_ppm'] = -model['depth_ppm_per_C'] * dT
        out['depth_mm_at_ref'] = -model['depth_m_per_C'] * dT * 1000.0
    pp = model.get('principal_px_per_C')
    if pp:
        out['dcx_px'] = -pp['cx'] * dT
        out['dcy_px'] = -pp['cy'] * dT
    if 'focal_ppm_per_C' in model:
        out['focal_ppm'] = -model['focal_ppm_per_C'] * dT
    gb = model.get('gyro_bias_deg_s_per_C')
    if gb:
        out['gyro_bias'] = {k: -v * dT for k, v in gb.items()}
    ab = model.get('accel_bias_ms2_per_C')
    if ab:
        out['accel_bias'] = {k: -v * dT for k, v in ab.items()}
    return out


def load_calibration():
    """优先用我们自己标的内参 —— 出厂彩色内参畸变全 0,拿来做反投影会让点云边缘外扩。"""
    p = os.path.join(ROOT, 'data', 'cam_rgb-camchain.yaml')
    if not os.path.exists(p):
        return None
    out = {}
    for line in open(p):
        line = line.strip()
        for key in ('intrinsics', 'distortion_coeffs', 'resolution'):
            if line.startswith(key + ':'):
                out[key] = json.loads(line.split(':', 1)[1].strip())
    return out if 'intrinsics' in out else None


class Hub:
    def __init__(self):
        self.clients = set()
        self.latest = {}
        self.lock = threading.Lock()
        self.loop = None
        self.wake = None
        self.meta = None
        self.src = None
        self.thermal_model = None
        self.counts = {}
        self.t0 = time.time()

    def on_frame(self, kind, payload):
        """由源线程调用。只留最新一帧。"""
        with self.lock:
            self.latest[kind] = payload
            self.counts[kind] = self.counts.get(kind, 0) + 1
        if self.loop and self.wake:
            try:
                self.loop.call_soon_threadsafe(self.wake.set)
            except RuntimeError:
                pass

    def take(self):
        with self.lock:
            got, self.latest = self.latest, {}
            return got

    def stats(self):
        el = max(time.time() - self.t0, 1e-9)
        with self.lock:
            out = {'streams': {k: dict(count=v, fps=round(v / el, 2))
                               for k, v in self.counts.items()}}
        temps = {}
        if self.src is not None and hasattr(self.src, 'temperatures'):
            try:
                temps = self.src.temperatures()
            except Exception:
                temps = {}
        if temps:
            out['temps'] = temps
            corr = thermal_correction(self.thermal_model, temps.get('asic'))
            if corr:
                out['thermal'] = corr
        out['has_thermal_model'] = self.thermal_model is not None
        return out


async def broadcast_loop(hub, max_fps):
    min_dt = 1.0 / max_fps if max_fps > 0 else 0.0
    last_stats = 0.0
    while True:
        await hub.wake.wait()
        hub.wake.clear()
        t = time.time()
        got = hub.take()
        msgs = []
        if 'depth' in got:
            d = got['depth']
            msgs.append(P.pack_depth(d['seq'], d['t'], d['arr'], d['scale']))
        if 'color' in got:
            c = got['color']
            msgs.append(P.pack_color(c['seq'], c['t'], c['w'], c['h'], c['jpeg']))
        for key, lz in (('ir', 1), ('ir_clean', 0)):
            if key in got:
                i = got[key]
                msgs.append(P.pack_ir(i['seq'], i['t'], i['w'], i['h'], i['jpeg'],
                                      i.get('laser', lz)))
        if 'points' in got:
            p = got['points']
            msgs.append(P.pack_points(p['seq'], p['t'], p['xyz'], p.get('intensity')))
        if t - last_stats > 1.0:
            msgs.append(P.pack_stats(hub.stats()))
            last_stats = t
        if msgs and hub.clients:
            dead = []
            for ws in list(hub.clients):
                try:
                    for m in msgs:
                        await ws.send(m)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                hub.clients.discard(ws)
        if min_dt:
            await asyncio.sleep(min_dt)


async def handler(ws, hub):
    hub.clients.add(ws)
    peer = getattr(ws, 'remote_address', None)
    print(f'[ws] 客户端接入 {peer}  (共 {len(hub.clients)})')
    try:
        if hub.meta:
            m = dict(hub.meta)
            if getattr(hub, 'meta_calib', None):
                m['calib'] = hub.meta_calib
            await ws.send(P.pack_meta(m))
        async for _ in ws:
            pass
    except Exception:
        pass
    finally:
        hub.clients.discard(ws)
        print(f'[ws] 客户端断开 {peer}  (共 {len(hub.clients)})')


def serve_http(port):
    class H(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=HERE, **kw)

        def do_GET(self):
            if self.path.startswith('/calib.json'):
                try:
                    import calib_summary
                    body = json.dumps(calib_summary.collect(), ensure_ascii=False).encode()
                except Exception as e:
                    body = json.dumps({'error': str(e)}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers(); self.wfile.write(body); return
            if self.path.split('?')[0] in ('/', ''):
                self.path = '/viewer.html'
            return super().do_GET()

        def log_message(self, *a):
            pass

    socketserver.TCPServer.allow_reuse_address = True
    srv = socketserver.TCPServer(('0.0.0.0', port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


async def main_async(args):
    hub = Hub()
    hub.loop = asyncio.get_running_loop()
    hub.wake = asyncio.Event()

    if args.source == 'd435i':
        from sources.d435i import D435i
        calib = load_calibration()
        if calib:
            print(f"[calib] 使用自标定彩色内参 fx={calib['intrinsics'][0]:.2f} "
                  f"cx={calib['intrinsics'][2]:.2f}  (出厂畸变全 0,不可用于反投影)")
        src = D435i(hub.on_frame, jpeg_quality=args.jpeg,
                    align_to_color=args.align, calib_intrinsics=calib,
                    emitter_alternate=args.alt_emitter)
    elif args.source == 'synthetic':
        from sources.synthetic import Synthetic
        src = Synthetic(hub.on_frame, fps=args.fps)
    else:
        print(f'未知源 {args.source}'); sys.exit(1)

    hub.src = src
    hub.thermal_model = load_thermal_model()
    if hub.thermal_model:
        m = hub.thermal_model
        print(f"[温漂] 已载入模型 (T_ref={m.get('T_ref_C')}C, "
              f"来自 {m.get('source')}, {m.get('duration_h', 0):.1f}h)")
    else:
        print("[温漂] 未找到 results/thermal_model.json —— 界面显示'未标定'")
    src.start()
    for _ in range(100):                    # 等源把 meta 准备好
        if src.meta():
            break
        await asyncio.sleep(0.05)
    try:
        hub.meta_calib = calib_summary.collect()
        nd = sum(1 for x in hub.meta_calib['stages'] if x['status'] == 'done')
        print(f"[标定] 载入 {nd}/{len(hub.meta_calib['stages'])} 项已完成结果,"
              f"{len(hub.meta_calib['pending'])} 项待办")
    except Exception as e:
        hub.meta_calib = None
        print(f"[标定] 汇总失败: {e}")
    hub.meta = src.meta()
    if not hub.meta:
        print('源未能初始化'); sys.exit(1)

    d = hub.meta['depth']
    print(f"[源] {hub.meta['source']}  depth {d['width']}x{d['height']}  "
          f"fx={d['fx']:.2f} fy={d['fy']:.2f} cx={d['cx']:.2f} cy={d['cy']:.2f}")
    if hub.meta.get('usb'):
        print(f"[源] USB {hub.meta['usb']}"
              + ("   ⚠ USB2 限制了帧率" if hub.meta['usb'].startswith('2') else ""))

    serve_http(args.http)
    print(f'\n  打开浏览器:  http://localhost:{args.http}\n')

    import websockets
    async with websockets.serve(functools.partial(handler, hub=hub),
                                '0.0.0.0', args.ws, max_size=None,
                                compression=None):
        await broadcast_loop(hub, args.max_fps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='d435i', choices=['d435i', 'synthetic'])
    ap.add_argument('--http', type=int, default=8080)
    ap.add_argument('--ws', type=int, default=9002)
    ap.add_argument('--jpeg', type=int, default=75)
    ap.add_argument('--fps', type=int, default=10, help='合成源的帧率')
    ap.add_argument('--max-fps', type=float, default=30, help='广播上限')
    ap.add_argument('--align', action='store_true', help='把深度对齐到彩色视角')
    ap.add_argument('--alt-emitter', action='store_true',
                    help='发射器交替帧:散斑帧供深度,干净帧供 VIO/靶标')
    args = ap.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print('\n退出')


if __name__ == '__main__':
    main()
