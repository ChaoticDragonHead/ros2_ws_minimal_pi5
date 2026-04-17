#!/usr/bin/env python3
import time
import getpass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

#RPi.GPIO is outdated
#try:
#    import RPi.GPIO as GPIO
#    GPIO_AVAILABLE = True
#except ImportError:
#    GPIO_AVAILABLE = False

#Use gpiozero instead, which is more modern and easier to use
try:
    from gpiozero import Motor
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # Robot naming
        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = self.get_parameter("robot_name").value.strip() or getpass.getuser()

        # Parameters
        self.declare_parameter("wheel_separation", 0.18)   # meters
        self.declare_parameter("max_linear_speed", 0.4)    # m/s
        self.declare_parameter("max_angular_speed", 2.0)   # rad/s
        self.declare_parameter("max_pwm", 100)             # percent

        # Topic this robot listens to
        self.declare_parameter("cmd_vel_topic", f"/{self.robot_name}/cmd_vel")
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value.strip() or f"/{self.robot_name}/cmd_vel"

        # GPIO pins (BCM)
        self.EN_A = 12
        self.IN1 = 17
        self.IN2 = 27
        self.IN3 = 22
        self.IN4 = 23
        self.EN_B = 13

        self.left_motor = None
        self.right_motor = None

        if GPIO_AVAILABLE:
            self._setup_gpio()
        else:
            self.get_logger().warn("gpiozero not available. Motors will NOT move.")

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog
        self.last_cmd_time = time.time()
        self.timeout_sec = 0.5
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(f"[{self.robot_name}] Motor driver listening on {self.cmd_vel_topic}")

    # --------------------------------------------------

    def _setup_gpio(self):
        self.left_motor = Motor(forward=self.IN1, backward=self.IN2, enable=self.EN_A, pwm=True)
        self.right_motor = Motor(forward=self.IN3, backward=self.IN4, enable=self.EN_B, pwm=True)

        self._set_motor_outputs(0.0, 0.0)
        self.get_logger().info("GPIO initialized for motor control")

    # --------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        wheel_sep = float(self.get_parameter("wheel_separation").value)
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_ang = float(self.get_parameter("max_angular_speed").value)

        v = max(-max_lin, min(max_lin, msg.linear.x))
        w = max(-max_ang, min(max_ang, msg.angular.z))

        # ===============================
        # TANK-SPIN OVERRIDE
        # ===============================
        if abs(v) < 1e-3 and abs(w) > 1e-3:
            spin_speed = 0.7 * max_lin
            direction = 1.0 if w > 0.0 else -1.0
            v_left = -direction * spin_speed
            v_right = +direction * spin_speed
            self._set_motor_outputs(v_left, v_right)
            return

        # ===============================
        # NORMAL DIFF-DRIVE MODE
        # ===============================
        v_left = v - (w * wheel_sep / 2.0)
        v_right = v + (w * wheel_sep / 2.0)
        self._set_motor_outputs(v_left, v_right)

    # --------------------------------------------------

    def _watchdog(self):
        if time.time() - self.last_cmd_time > self.timeout_sec:
            self._set_motor_outputs(0.0, 0.0)

    # --------------------------------------------------

    def _set_motor_outputs(self, v_left: float, v_right: float):
        if not GPIO_AVAILABLE:
            return

        max_lin = float(self.get_parameter("max_linear_speed").value)

        def speed_to_ratio(v):
            return max(-1.0, min(1.0, v / max_lin))

        left_ratio = speed_to_ratio(v_left)
        right_ratio = speed_to_ratio(v_right)

        self.left_motor.value = left_ratio
        self.right_motor.value = right_ratio

    # --------------------------------------------------

    def destroy_node(self):
        self._set_motor_outputs(0.0, 0.0)
        if GPIO_AVAILABLE:
            self.left_motor.close()
            self.right_motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
