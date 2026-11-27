import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowCmd, LowState

from t2_node import SafetyNode
from t2_safety import SERVO
from t3_policy import KD, KP, PERIOD, Policy


class WalkingNode(Node):
    def __init__(self):
        super().__init__("go1_t3_walk")
        config = json.loads(Path(__file__).with_name("t3_weights.json").read_text())
        self.policy = Policy(config["weights"])
        self.speed = 0.
        self.start_tick = None
        self.last_tick = None
        self.publisher = self.create_publisher(LowCmd, "/lowcmd_bare", 1)
        self.create_subscription(Twist, "/cmd_vel", self.receive_speed, qos_profile_sensor_data)
        self.create_subscription(LowState, "/lowstate", self.receive_state, qos_profile_sensor_data)
        self.get_logger().info("T3 ready")

    def receive_speed(self, msg):
        if np.isfinite(msg.linear.x):
            self.speed = float(np.clip(msg.linear.x, 0., .6))

    def receive_state(self, msg):
        tick = int(msg.tick)
        if self.last_tick is not None and tick - self.last_tick < round(PERIOD * 1000):
            return
        quat = np.asarray(msg.imu_state.quaternion, dtype=float)
        gyro = np.asarray(msg.imu_state.gyroscope, dtype=float)
        norm = np.linalg.norm(quat)
        if not np.isfinite(norm) or norm < 1e-6 or not np.isfinite(gyro).all():
            return
        if self.start_tick is None:
            self.start_tick = tick
        dt = PERIOD if self.last_tick is None else min(.06, (tick - self.last_tick) / 1000.)
        self.last_tick = tick
        speed = self.speed if tick - self.start_tick >= 1500 else 0.
        target = self.policy.step(speed, quat / norm, gyro, dt)
        command = LowCmd()
        for i, motor in enumerate(command.motor_cmd[:12]):
            motor.mode = SERVO
            motor.q = float(target[i])
            motor.kp = KP
            motor.kd = KD
        self.publisher.publish(command)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    safety = SafetyNode()
    walking = WalkingNode()
    executor.add_node(safety)
    executor.add_node(walking)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        walking.destroy_node()
        safety.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
