#!/usr/bin/env python3
"""Print DDSM115 wheel encoder counts received on a JointState topic."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


COUNTS_PER_REVOLUTION = 32767.0  #4096 သွားဖတ်တော့
WHEEL_COLORS = ("\033[31m", "\033[33m", "\033[32m", "\033[36m")
ANSI_RESET = "\033[0m"


class EncoderCountPrinter(Node):
    """Formats wheel positions as continuous encoder counts."""

    def __init__(self) -> None:
        super().__init__("ddsm115_encoder_count_printer")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("print_period", 0.25)

        topic = self.get_parameter("joint_states_topic").value
        print_period = float(self.get_parameter("print_period").value)
        if print_period <= 0.0:
            raise ValueError("print_period must be greater than zero")

        self._latest_state: JointState | None = None
        self.create_subscription(JointState, topic, self._joint_state_callback, 10)
        self.create_timer(print_period, self._print_counts)
        self.get_logger().info(
            f"Listening on {topic}; using {COUNTS_PER_REVOLUTION:.0f} counts per revolution")

    def _joint_state_callback(self, message: JointState) -> None:
        self._latest_state = message

    def _print_counts(self) -> None:
        if self._latest_state is None:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=5.0)
            return

        state = self._latest_state
        count_text = []
        for wheel_index, (name, position) in enumerate(zip(state.name, state.position)):
            counts = round(position / (2.0 * math.pi) * COUNTS_PER_REVOLUTION)
            color = WHEEL_COLORS[wheel_index % len(WHEEL_COLORS)]
            count_text.append(
                f"{color}{name}: {counts} counts ({position:.4f} rad){ANSI_RESET}")

        if not count_text:
            self.get_logger().warn("Received a JointState without positions")
            return

        self.get_logger().info("\n" + "\n".join(count_text))
        
def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderCountPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
