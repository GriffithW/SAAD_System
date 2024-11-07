#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time


class ThrusterControlRight(Node):
    def __init__(self):
        super().__init__('thruster_control_Right')
        self.declare_parameter('pwm_pin', 18)
        self.declare_parameter('min_pwm', 1000)
        self.declare_parameter('max_pwm', 2000)



        self.pwm_pin = self.get_parameter('pwm_pin').get_parameter_value().integer_value
        self.min_pwm = self.get_parameter('min_pwm').get_parameter_value().integer_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value

        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 50)


        self.pwm.start(0)

        self.subscription = self.create_subscription(
            Float64,
            'thruster_right/speed',
            self.control_thruster_right_callback,
            10

        )


    def control_thruster_right_callback(self, msg):
        pwm_value = self.map_speed_pwm(msg.data)
        self.pwm.ChangeDutyCycle(pwm_value)

    def map_speed_to_pwm(self, speed):
        return (self.max_pwm - self.min_pwm) * speed + self.min_pwm
    
    def stop_pwm(self):
        self.pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControlRight()
    try:
        rclpy.spin(thruster_control)
    except KeyboardInterrupt:
        pass
    finally: 
        thruster_control.stop_pwm()
        rclpy.shutdown()


if __name__ == '__main__':
    main()