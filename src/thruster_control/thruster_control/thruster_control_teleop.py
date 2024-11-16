

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
from select import select
import termios
import tty

# Key bindings for thruster control
moveBindings = {
    '\033[A': 1.0,   # Forward
    ',': -1.0,  # Backward
    'k': 0.0    # Stop
}

msg = """
Control Thruster:
---------------------------
up : Increase speed (forward)
, : Decrease speed (backward)
k : Stop thruster

CTRL-C to quit
"""

def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\033':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)

def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

class ThrusterControlTeleop(Node):
    def __init__(self):
        super().__init__('thruster_control_teleop')
        self.publisher_left = self.create_publisher(Float64, 'thruster_left/speed', 10)
        self.publisher_right = self.create_publisher(Float64, 'thruster_right/speed', 10)
        self.speed = 0.5  # Initial speed

    def publish_speed(self, speed):
        msg = Float64()
        msg.data = speed
        self.publisher_left.publish(msg)
        self.publisher_right.publish(msg)
        self.get_logger().info(f"Published speed: {speed}")

def main(args=None):
    settings = save_terminal_settings()

    rclpy.init(args=args)
    node = ThrusterControlTeleop()
    print(msg)

    try:
        while True:
            key = get_key(settings, timeout=0.1)
            if key in moveBindings:
                node.speed = moveBindings[key]
                node.publish_speed(node.speed)
            elif key == '\x03':  # CTRL-C
                break
    except Exception as e:
        print(e)
    finally:
        restore_terminal_settings(settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
