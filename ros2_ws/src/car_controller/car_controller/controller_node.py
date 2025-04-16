# Copyright 2025 Max Domitrovic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Car Controller Node.

Subscribe to the 'vehicle_command' topic and convert VehicleCommand messages into
vehicle control actions.^
"""

from custom_msgs.msg import VehicleCommand
try:
    import RPi.GPIO as GPIO  # type: ignore
except (ImportError, RuntimeError):
    # Fallback für Docker / PC-Umgebungen
    class GPIO: # noqa
        BCM = OUT = HIGH = LOW = None

        @staticmethod
        def setmode(mode): pass # noqa

        @staticmethod
        def setwarnings(flag): pass # noqa

        @staticmethod
        def setup(pin, mode): pass # noqa

        @staticmethod
        def output(pin, state): pass # noqa

        @staticmethod
        def PWM(pin, frequency): return None  # noqa

        @staticmethod
        def cleanup(): pass # noqa

import rclpy
from rclpy.node import Node


class CarController(Node): # noqa
    def __init__(self):  # noqa
        super().__init__('car_controller')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pin configuration
        self.motor_forward_pin = 24
        self.motor_backward_pin = 25
        self.motor_steering_pin = 23

        # Setup pins
        GPIO.setup(self.motor_forward_pin, GPIO.OUT)
        GPIO.setup(self.motor_backward_pin, GPIO.OUT)
        GPIO.setup(self.motor_steering_pin, GPIO.OUT)

        # Initialize PWM at 50Hz
        self.motor_forward_pwm = GPIO.PWM(self.motor_forward_pin, 50)
        self.motor_backward_pwm = GPIO.PWM(self.motor_backward_pin, 50)
        self.steering_motor_pwm = GPIO.PWM(self.motor_steering_pin, 50)

        # Start PWM with 0% duty cycle
        self.motor_forward_pwm.start(0)
        self.motor_backward_pwm.start(0)
        self.steering_motor_pwm.start(0)

        self.subscription = self.create_subscription(
            VehicleCommand,
            'vehicle_command',
            self.command_callback,
            10
        )
        self.get_logger().info('CarController node started.')

    def command_callback(self, msg): # noqa
        self.get_logger().info(
            f'Received command: speed={msg.speed}, angel={msg.angle}'
        )
        if msg.speed > 0:
            self.drive_forward(msg.speed)
        elif msg.speed < 0:
            self.drive_backward(abs(msg.speed))
        else:
            self.get_logger().info('No movement command received (speed is zero).')

        # Benutze das angle-Feld für die Lenkung
        self.set_steering(msg.angle)

    def drive_forward(self, speed):  # noqa
        self.motor_forward_pwm.ChangeDutyCycle(min(max(speed, 0), 100))
        self.motor_backward_pwm.ChangeDutyCycle(0)
        self.get_logger().info(f'PWM: Driving forward at speed {speed}')

    def drive_backward(self, speed):  # noqa
        self.motor_forward_pwm.ChangeDutyCycle(0)
        self.motor_backward_pwm.ChangeDutyCycle(min(max(speed, 0), 100))
        self.get_logger().info(f'PWM: Driving backward at speed {speed}')

    def set_steering(self, angle):  # noqa
        duty_cycle = 7.5  # Default center
        if angle < 0:
            duty_cycle = max(5.0, 7.5 + (angle / 90) * 2.5)
        elif angle > 0:
            duty_cycle = min(10.0, 7.5 + (angle / 90) * 2.5)

        self.steering_motor_pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f'PWM: Steering with angle {angle}, duty_cycle {duty_cycle}')

def main(args=None): # noqa
    rclpy.init(args=args)
    node = CarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
