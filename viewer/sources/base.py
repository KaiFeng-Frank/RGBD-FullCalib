#!/usr/bin/env python3
"""数据源抽象。

存在的理由:D435i 出的是稠密 2D 深度图(靠内参反投影成点云),
Mid-360 出的是稀疏 3D 点流(本身就是点,没有像平面)。
两者在渲染端是完全不同的路径 —— 一个走深度纹理 + shader 反投影,
一个走顶点缓冲直传。所以源必须在这一层就分开,而不是硬塞进同一个数据结构。
"""
import threading


class Source:
    """所有数据源的基类。子类在自己的线程里跑,通过 on_frame 回调推数据。"""

    name = 'base'

    def __init__(self, on_frame):
        self.on_frame = on_frame        # on_frame(kind, payload_dict)
        self._stop = threading.Event()
        self._thread = None

    def meta(self):
        """返回该源的静态信息(内参、单位、可用通道等),开流前调用。"""
        raise NotImplementedError

    def _run(self):
        raise NotImplementedError

    def start(self):
        self._thread = threading.Thread(target=self._guarded, daemon=True)
        self._thread.start()

    def _guarded(self):
        try:
            self._run()
        except Exception as e:
            import traceback
            print(f'[{self.name}] 源线程异常: {e}')
            traceback.print_exc()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
