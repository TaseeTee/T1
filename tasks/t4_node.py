from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

from t4_scan import ScanOdometry, scan_points


def nanos(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class OdometryNode(Node):
    def __init__(self):
        super().__init__("go1_t4_odometry")
        self.filter = ScanOdometry()
        self.imu = deque(maxlen=200)
        self.yaw_origin = None
        self.last_publish = None
        self.pending_scan = None
        self.last_scan = None
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.get_logger().info("T4 ready")

    def on_scan(self, msg):
        self.pending_scan = msg
        self.match_scan()

    def match_scan(self):
        if self.pending_scan is None or not self.imu:
            return
        msg = self.pending_scan
        time = nanos(msg.header.stamp)
        if time > self.imu[-1][0]:
            return
        self.pending_scan = None
        if self.last_scan is not None and time <= self.last_scan:
            return
        imu_time, attitude = min(self.imu, key=lambda entry: abs(entry[0] - time))
        if abs(imu_time - time) > 25_000_000:
            return
        points, indices = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                      msg.range_min, msg.range_max, attitude.as_matrix())
        matrix = attitude.as_matrix()
        self.filter.update(points, indices, np.arctan2(matrix[1, 0], matrix[0, 0]))
        self.last_scan = time

    def on_imu(self, msg):
        stamp = msg.header.stamp
        time = nanos(stamp)
        if self.imu and time <= self.imu[-1][0]:
            return
        q = msg.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-6:
            return
        attitude = Rotation.from_quat(quat)
        if self.yaw_origin is None:
            matrix = attitude.as_matrix()
            yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
            self.yaw_origin = Rotation.from_euler("z", -yaw)
        attitude = self.yaw_origin * attitude
        self.imu.append((time, attitude))
        self.match_scan()
        if self.last_publish is None or time - self.last_publish >= 20_000_000:
            self.publish(stamp, attitude)
            self.last_publish = time

    def publish(self, stamp, attitude):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "trunk"
        pose = odom.pose.pose
        pose.position.x, pose.position.y = map(float, self.filter.position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            map(float, attitude.as_quat())
        )
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.rotation = pose.orientation
        self.odom.publish(odom)
        self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
