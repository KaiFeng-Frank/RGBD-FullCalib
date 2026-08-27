#!/usr/bin/env python3
"""ROS1 bag 写入。集中放这里,因为 ROS1 和 ROS2 的消息定义有坑:

  std_msgs/Header 在 ROS1 里有 seq 字段,ROS2 里没有。
  用 ROS1 typestore 却按 ROS2 写,会在写第一条消息时才炸 ——
  也就是采集全部结束之后,数据全丢。所以这块必须能被单独测试。
"""
import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS1_NOETIC)
ImageMsg = TS.types['sensor_msgs/msg/Image']
ImuMsg = TS.types['sensor_msgs/msg/Imu']
Header = TS.types['std_msgs/msg/Header']
TimeMsg = TS.types['builtin_interfaces/msg/Time']
Vector3 = TS.types['geometry_msgs/msg/Vector3']
Quaternion = TS.types['geometry_msgs/msg/Quaternion']


def _stamp(t):
    sec = int(t)
    return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def image_msg(seq, t, frame_id, arr, encoding):
    h, w = arr.shape[:2]
    ch = arr.shape[2] if arr.ndim == 3 else 1
    return ImageMsg(
        header=Header(seq=seq, stamp=_stamp(t), frame_id=frame_id),
        height=h, width=w, encoding=encoding, is_bigendian=0,
        step=w * ch, data=np.ascontiguousarray(arr).reshape(-1),
    )


def imu_msg(seq, t, gyro, accel, frame_id='imu4'):
    cov_unknown = np.array([-1.0] + [0.0] * 8, dtype=np.float64)
    cov_zero = np.zeros(9, dtype=np.float64)
    return ImuMsg(
        header=Header(seq=seq, stamp=_stamp(t), frame_id=frame_id),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=cov_unknown,
        angular_velocity=Vector3(x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2])),
        angular_velocity_covariance=cov_zero,
        linear_acceleration=Vector3(x=float(accel[0]), y=float(accel[1]), z=float(accel[2])),
        linear_acceleration_covariance=cov_zero,
    )


class BagWriter:
    """边采边写。崩溃时已写入的部分依然是合法 bag。"""

    def __init__(self, path):
        self.path = str(path)
        self._w = None
        self._conns = {}
        self._seq = {}

    def __enter__(self):
        self._w = Writer(self.path)
        self._w.open()
        return self

    def __exit__(self, *a):
        try:
            self._w.close()
        except Exception:
            pass
        return False

    def _conn(self, topic, msgtype):
        if topic not in self._conns:
            self._conns[topic] = self._w.add_connection(topic, msgtype, typestore=TS)
            self._seq[topic] = 0
        return self._conns[topic]

    def write_image(self, topic, t, arr, encoding, frame_id='cam'):
        c = self._conn(topic, 'sensor_msgs/msg/Image')
        m = image_msg(self._seq[topic], t, frame_id, arr, encoding)
        self._w.write(c, int(t * 1e9), TS.serialize_ros1(m, 'sensor_msgs/msg/Image'))
        self._seq[topic] += 1

    def write_imu(self, topic, t, gyro, accel, frame_id='imu4'):
        c = self._conn(topic, 'sensor_msgs/msg/Imu')
        m = imu_msg(self._seq[topic], t, gyro, accel, frame_id)
        self._w.write(c, int(t * 1e9), TS.serialize_ros1(m, 'sensor_msgs/msg/Imu'))
        self._seq[topic] += 1
