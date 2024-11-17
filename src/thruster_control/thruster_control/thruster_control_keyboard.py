import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard


class ThrusterControlKeyboard(Node):
    def __init__(self):
        super().__init__('thruster_control_keyboard')

        self.publisher_left_ = self.create_publisher(Float64, 'thruster_left/speed', 10)
        self.publisher_right_ = self.create_publisher(Float64, 'thruster_right/speed', 10)
        self.active_speed = Float64()
        self.active_speed.data = 0.5
        self.stop_speed = Float64()
        self.stop_speed.data = 0.0

        self.pressed_keys = set()


        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
    def publish_both_speeds(self, speed):
        self.publisher_right_.publish(self.active_speed)
        self.publisher_left_.publish(self.active_speed)
    def on_press(self, key):
        if key in {keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right}:
            self.pressed_keys.add(key)
            if key == keyboard.Key.left:
                self.get_logger().info("Left arrow pressed")
                self.publisher_left_.publish(self.active_speed)
            elif key == keyboard.Key.right:
                self.get_logger().info("Right arrow pressed")
                self.publisher_right_.publish(self.active_speed)
            elif key == keyboard.Key.up:
                self.get_logger().info("up arrow pressed")
                self.active_speed.data += 0.1  # Modify the `data` attribute
                self.publish_both_speeds(self.active_speed)
            elif key == keyboard.Key.down:
                self.get_logger().info("down arrow pressed")
                self.active_speed.data -= 0.1  # Modify the `data` attribute
                self.publish_both_speeds(self.active_speed)



    def on_release(self, key):

            if key in {keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right}:
                self.pressed_keys.remove(key)
                if key == keyboard.Key.left:
                     self.get_logger().info("Left arrow released")
                     self.publisher_left_.publish(self.stop_speed)
                elif key == keyboard.Key.right:
                     self.get_logger().info("Right arrow released")
                     self.publisher_right_.publish(self.stop_speed)

    



def main(args=None):
    rclpy.init(args=args)
    thruster_control_keyboard = ThrusterControlKeyboard()
    try:
        rclpy.spin(thruster_control_keyboard)
    except KeyboardInterrupt:
        pass
    finally:
      thruster_control_keyboard.destroy_node()
      rclpy.shutdown()
     

if __name__ == '__main__':
    main()