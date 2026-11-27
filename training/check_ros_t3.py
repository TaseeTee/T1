import argparse
from math import cos, radians, sin
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data

from t3_policy import angles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=.22)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("t3_local_check")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    state = None

    def receive(msg):
        nonlocal state
        state = msg

    node.create_subscription(Odometry, "/odom/ground_truth", receive, qos_profile_sensor_data)
    command = Twist()
    command.linear.x = args.speed
    start = None
    bad_since = [None, None, None]
    previous_stamp = None
    initial = None
    progress = 0.
    deadline = time.monotonic() + 240
    next_send = 0.
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                pub.publish(command)
                next_send = now + .05
            rclpy.spin_once(node, timeout_sec=.05)
            if state is None:
                continue
            stamp = state.header.stamp.sec + state.header.stamp.nanosec / 1e9
            if stamp == previous_stamp:
                continue
            previous_stamp = stamp
            p, q = state.pose.pose.position, state.pose.pose.orientation
            roll, pitch, yaw = angles([q.w, q.x, q.y, q.z])
            if start is None:
                start = stamp
                heading = yaw
            elapsed = stamp - start
            for i, bad in enumerate((p.z < .16, abs(roll) > radians(50), abs(pitch) > radians(50))):
                if bad:
                    if bad_since[i] is None:
                        bad_since[i] = stamp
                    if stamp - bad_since[i] > .5:
                        raise SystemExit(f"FAIL: падение, признак {i + 1}, пройдено {progress:.3f} м")
                else:
                    bad_since[i] = None
            if elapsed < 1.5:
                continue
            if initial is None:
                initial = p.x, p.y
            progress = (p.x - initial[0]) * cos(heading) + (p.y - initial[1]) * sin(heading)
            if progress >= 3.:
                print(f"OK: {progress:.3f} м за {elapsed - 1.5:.2f} с симуляции")
                return
            if elapsed >= 91.5:
                raise SystemExit(f"FAIL: за 90 с пройдено {progress:.3f} м")
        raise SystemExit("FAIL: истёк лимит")
    finally:
        pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
