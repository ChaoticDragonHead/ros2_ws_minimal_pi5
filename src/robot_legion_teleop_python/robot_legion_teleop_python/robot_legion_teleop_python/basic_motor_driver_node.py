import time
import getpass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from gpiozero import Motor
    # Test if we can actually create a motor (checks for GPIO hardware access)
    test_motor = Motor(17, 27)
    test_motor.close()
    GPIO_AVAILABLE = True
    print("gpiozero is available for motor control.", flush=True)
except (ImportError, Exception) as e:
    GPIO_AVAILABLE = False
    print("gpio init failed", flush=True)
    print(f"GPIO not available: {e}")

class MotorDriverNode(Node):
    def __init__(self):
        self.get_logger().info("super init started...")
        super().__init__("motor_driver_node")
        
        # Robot naming
        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = self.get_parameter("robot_name").value.strip() or getpass.getuser()
