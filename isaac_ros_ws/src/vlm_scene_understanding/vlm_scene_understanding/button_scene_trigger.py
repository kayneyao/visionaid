#!/usr/bin/env python3
"""
button.py
Publishes a scene-query request every time a physical (GPIO) or keyboard
button is pressed.

• If the script is run on a Jetson/Raspberry Pi with a push-button wired to
  BCM pin 17 it uses `gpiozero`.
• When GPIO is not available it falls back to a simple *press <ENTER>* prompt
  so the same code also works on a laptop during development.

Topic published
────────────────
/scene_query     (std_msgs/msg/String)

Example payload
───────────────
data: "Describe this scene for navigation"
"""
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TRY_GPIO = os.getenv("USE_GPIO", "1") == "1"
if TRY_GPIO:
    try:
        from gpiozero import Button
    except ImportError:
        TRY_GPIO = False


class SceneButton(Node):
    def __init__(self):
        super().__init__("scene_button")
        self.pub = self.create_publisher(String, "/scene_query", 10)

        if TRY_GPIO:
            self.get_logger().info("Using GPIO-based button on BCM 17")
            self.button = Button(17, pull_up=True, bounce_time=0.05)
            self.button.when_pressed = self._handle_press
        else:
            self.get_logger().info(
                "GPIO unavailable – press <ENTER> to trigger a scene query"
            )
            # timer checks stdin every 0.2 s
            self.create_timer(0.2, self._check_keyboard)

    # ───────────────────────────────────────── internal helpers ──
    def _handle_press(self):
        self._publish_query("Describe this scene for navigation")

    def _check_keyboard(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            _ = sys.stdin.readline()
            self._handle_press()

    def _publish_query(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"Triggered scene query -> \"{text}\"")


def main():
    rclpy.init()
    node = SceneButton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
