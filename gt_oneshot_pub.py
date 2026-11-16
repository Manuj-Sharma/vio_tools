#!/usr/bin/env python3
"""Live GPS ground truth in the VIO 'global' frame on /ov_msckf/pathgt.
Alignment at VIO init (first /ov_msckf/odomimu): theta = (pi/2 - EKF yaw) - VIO yaw0 + mounting yaw, anchored at the GPS fix at that time. Needs /fc/gps1 and /fc/ekf_yaw.
Mounting yaw (OFF by default): EKF yaw is the heading of the Cube nose, VIO yaw is of the IMX frame (Cube-IMU bags are rotated into it too). The IMX
x-axis points 2.59 deg to the right of the Cube nose (saved Cube->IMX rotation, v_imx = R v_cube, Cube frame forward-right-down) -> mounting yaw -2.59 deg.
Tested 2026-11-16 on the 200 m bag: it made the GPS line fit WORSE (the one-shot yaw is already several deg too small), so the default is 0.
Env GT_MOUNT_YAW_DEG=<deg> applies a fixed value; GT_MOUNT_YAW_DEG=file takes it from the saved rotation."""
import math, os, rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

R_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'cube_imu_bag', 'R_cube_to_imx_gravity_gyro.npy')
def mount_yaw():
    v = os.environ.get('GT_MOUNT_YAW_DEG', '')
    if v == '': return 0.0, 'off'
    if v != 'file': return math.radians(float(v)), 'env GT_MOUNT_YAW_DEG'
    import numpy as np
    R = np.load(R_FILE); return -math.atan2(R[0, 1], R[0, 0]), R_FILE   # first row of R = IMX x-axis in the Cube frame
MOUNT, MOUNT_SRC = mount_yaw()

class GT(Node):
    def __init__(s):
        super().__init__('gt_oneshot_pub'); s.fix = []; s.yaw = None; s.al = None; s.ll0 = None
        s.path = Path(); s.path.header.frame_id = 'global'
        s.pub = s.create_publisher(Path, '/ov_msckf/pathgt', 10)
        s.create_subscription(NavSatFix, '/fc/gps1', s.gps, 100)
        s.create_subscription(Float32, '/fc/ekf_yaw', lambda m: setattr(s, 'yaw', m.data), 50)
        s.create_subscription(Odometry, '/ov_msckf/odomimu', s.odom, 10)
    def enu(s, m):
        if s.ll0 is None: s.ll0 = (m.latitude, m.longitude, m.altitude)
        r = 6378137.
        return (math.radians(m.longitude - s.ll0[1]) * r * math.cos(math.radians(s.ll0[0])), math.radians(m.latitude - s.ll0[0]) * r, m.altitude - s.ll0[2])
    def odom(s, m):
        if s.al is not None or s.yaw is None or not s.fix: return
        q = m.pose.pose.orientation; vy0 = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        th = (math.pi/2 - s.yaw) - vy0 + MOUNT; p = m.pose.pose.position; e = s.fix[-1][1]
        ct, st = math.cos(th), math.sin(th)
        tr = (e[0]-(ct*p.x-st*p.y), e[1]-(st*p.x+ct*p.y), e[2]-p.z)
        s.al = (th, tr); s.get_logger().info('aligned: ekf yaw %.1f deg, vio yaw0 %.1f deg, mount yaw %.2f deg (%s), theta %.1f deg' % (math.degrees(s.yaw), math.degrees(vy0), math.degrees(MOUNT), MOUNT_SRC, math.degrees(th)))
        for st_, e_ in s.fix: s.add(st_, e_)
    def add(s, stamp, e):
        th, tr = s.al; ct, st = math.cos(th), math.sin(th); x, y, z = e[0]-tr[0], e[1]-tr[1], e[2]-tr[2]
        p = PoseStamped(); p.header.frame_id = 'global'; p.header.stamp = stamp
        p.pose.position.x = ct*x + st*y; p.pose.position.y = -st*x + ct*y; p.pose.position.z = z; p.pose.orientation.w = 1.0
        s.path.poses.append(p); s.path.header.stamp = stamp; s.pub.publish(s.path)
    def gps(s, m):
        e = s.enu(m); s.fix.append((m.header.stamp, e))
        if s.al: s.add(m.header.stamp, e)
rclpy.init(); rclpy.spin(GT())
