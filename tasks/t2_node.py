from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from unitree_go.msg import LowCmd, LowState

from t2_safety import FIELDS, SERVO, Safety

OUTPUT_PERIOD_MS = 8
TIMEOUT_MS = 400


class SafetyNode(Node):
    def __init__(self):
        super().__init__("go1_safety")
        self.safety = Safety(Path(__file__).resolve().parents[1] / "vendor" / "go1-model")
        self.now_ms = None
        self.pending = None
        self.last_input_ms = None
        self.last_output_ms = None
        self.output = LowCmd()
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(LowCmd, "/lowcmd", 1)
        self.create_subscription(LowCmd, "/lowcmd_bare", self.receive, input_qos)
        self.create_subscription(LowState, "/lowstate", self.update_state, input_qos)
        self.create_subscription(Clock, "/clock", self.update_clock, input_qos)
        self.get_logger().info("T2 ready")

    def receive(self, msg):
        self.last_input_ms = self.now_ms
        self.pending = msg #свежак

    def update_state(self, msg):
        self.advance(int(msg.tick))

    def update_clock(self, msg):
        self.advance(msg.clock.sec * 1000 + msg.clock.nanosec // 1_000_000)

    def advance(self, time_ms):
        if self.now_ms is not None and time_ms < self.now_ms:
            return
        self.now_ms = time_ms
        if self.last_output_ms is not None and time_ms - self.last_output_ms < OUTPUT_PERIOD_MS:
            return
        if self.pending is not None:
            if self.last_input_ms is None:
                self.last_input_ms = time_ms
            self.safety.accept(self.pending.motor_cmd)
            self.pending = None

        if self.last_input_ms is None or time_ms - self.last_input_ms >= TIMEOUT_MS:
            self.safety.command = self.safety.hold()
        for i, row in enumerate(self.safety.command):
            motor = self.output.motor_cmd[i]
            motor.mode = SERVO
            for field, value in zip(FIELDS, row):
                setattr(motor, field, float(value))
        self.publisher.publish(self.output)
        self.last_output_ms = time_ms


def main():
    rclpy.init()
    node = None
    try:
        node = SafetyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
