#!/usr/bin/env python3
"""ROS2 bag -> ROS1 bag with /cam0/image_raw and/or /imu0, for Kalibr.

Kalibr only reads ROS1 bags, so every recording has to be converted first.

  mk_imucam_bag.py SRC_ROS2_DIR DST.bag [CAM_STEP] [--cam-only]

CAM_STEP decimates the CAMERA only (IMU is never decimated). Kalibr wants ~4 Hz
for intrinsics (kalibr_calibrate_cameras) and ~20 Hz for cam-IMU, so use a step
that lands near those; the recordings here are already 20 Hz, so CAM_STEP=1 for
cam-IMU and CAM_STEP=5 for intrinsics.
"""
import sys
import numpy as np
from rosbags.rosbag2 import Reader as R2
from rosbags.rosbag1 import Writer as W1
from rosbags.typesys import Stores, get_typestore

SRC, DST = sys.argv[1], sys.argv[2]
CAM_STEP = int(sys.argv[3]) if len(sys.argv) > 3 and not sys.argv[3].startswith('-') else 1
CAM_ONLY = '--cam-only' in sys.argv

ts2 = get_typestore(Stores.ROS2_HUMBLE)
ts1 = get_typestore(Stores.ROS1_NOETIC)
Image = ts1.types['sensor_msgs/msg/Image']
Imu = ts1.types['sensor_msgs/msg/Imu']
Header = ts1.types['std_msgs/msg/Header']
Time = ts1.types['builtin_interfaces/msg/Time']
Quat = ts1.types['geometry_msgs/msg/Quaternion']
Vec3 = ts1.types['geometry_msgs/msg/Vector3']

ZERO9 = np.zeros(9, dtype=np.float64)
n_cam_in = n_cam = n_imu = 0

with R2(SRC) as r, W1(DST) as w:
    c_cam = w.add_connection('/cam0/image_raw', Image.__msgtype__, typestore=ts1)
    c_imu = None if CAM_ONLY else w.add_connection('/imu0', Imu.__msgtype__, typestore=ts1)
    for c, ts, raw in r.messages():
        m = ts2.deserialize_cdr(raw, c.msgtype)
        # Keep the recorder's CLOCK_MONOTONIC header stamp and use the identical
        # value as the bag time, so the two can never disagree.
        stamp_ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        hdr = Header(seq=0,
                     stamp=Time(sec=m.header.stamp.sec, nanosec=m.header.stamp.nanosec),
                     frame_id=m.header.frame_id or '')
        if c.topic == '/cam0/image_raw':
            n_cam_in += 1
            if (n_cam_in - 1) % CAM_STEP:
                continue
            hdr.seq = n_cam
            msg = Image(header=hdr, height=m.height, width=m.width,
                        encoding=m.encoding, is_bigendian=m.is_bigendian,
                        step=m.step, data=np.frombuffer(m.data, dtype=np.uint8))
            w.write(c_cam, stamp_ns, ts1.serialize_ros1(msg, Image.__msgtype__))
            n_cam += 1
        elif c.topic == '/imu0' and not CAM_ONLY:
            hdr.seq = n_imu
            # Kalibr reads only angular_velocity and linear_acceleration.
            msg = Imu(
                header=hdr,
                orientation=Quat(x=m.orientation.x, y=m.orientation.y,
                                 z=m.orientation.z, w=m.orientation.w),
                orientation_covariance=ZERO9.copy(),
                angular_velocity=Vec3(x=m.angular_velocity.x,
                                      y=m.angular_velocity.y,
                                      z=m.angular_velocity.z),
                angular_velocity_covariance=ZERO9.copy(),
                linear_acceleration=Vec3(x=m.linear_acceleration.x,
                                         y=m.linear_acceleration.y,
                                         z=m.linear_acceleration.z),
                linear_acceleration_covariance=ZERO9.copy(),
            )
            w.write(c_imu, stamp_ns, ts1.serialize_ros1(msg, Imu.__msgtype__))
            n_imu += 1

print(f"wrote {n_cam} images (of {n_cam_in}) + {n_imu} imu -> {DST}")
