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
vehicle control actions.
"""


import RPi.GPIO as GPIO

from custom_msgs.msg import VehicleCommand

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CarController(Node):
    """
    CarController Node.

    This class implements a ROS2 node to control a car’s movement and steering through GPIO-based PWM signals.
    It configures GPIO pins for forward and backward motor control, as well as for steering, and sets a mode LED
    to indicate a 'wait' state. Vehicle commands received via a ROS subscription are processed to adjust movement
    and steering accordingly.
    Attributes:
        motor_forward_pin (int): BCM pin number for forward motor control.
        motor_backward_pin (int): BCM pin number for backward motor control.
        motor_steering_pin (int): BCM pin number for steering control.
        motor_forward (GPIO.PWM): PWM object used for driving forward.
        motor_backward (GPIO.PWM): PWM object used for driving backward (and steering in this implementation).
        subscription: ROS subscription for receiving VehicleCommand messages.
    Methods:
        __init__():
            Initializes the node by setting up GPIO pins, configuring GPIO warnings,
            establishing PWM channels for each motor output, initializing a mode LED, and
            subscribing to the 'vehicle_command' topic for incoming commands.
        command_callback(msg: VehicleCommand):
            Callback function for processing incoming vehicle command messages.
            It logs the received command and drives the car forward or backward based on the speed value,
            and adjusts the steering according to the angle provided in the message.
        drive_forward(speed):
            Activates the forward motor by setting its PWM duty cycle based on the given speed (0-100%).
            It ensures the backward motor's PWM is set to 0 to prevent conflict.
        drive_backward(speed):
            Activates the backward motor by setting its PWM duty cycle based on the given speed (0-100%).
            It ensures the forward motor's PWM is set to 0 to prevent conflict.
        set_steering(angle):
            Adjusts the PWM duty cycle for steering based on the provided angle.
            The default (center) duty cycle is 7.5%; it is adjusted within a range (5.0% to 10.0%)
            proportionally to the angle to steer left or right.
    """

    def __init__(self):
        """
        Initialize the CarController node.

        This method performs the following operations:
        - Calls the parent class initializer with the node name 'car_controller'.
        - Configures the GPIO settings:
            - Sets the pin numbering mode to BCM.
            - Disables warnings.
        - Defines and sets up GPIO pins for:
            - Motor control (forward, backward, steering).
            - A LED indicator on pin 20 to indicate 'wait' mode, which is set to HIGH.
        - Initializes PWM for the motor control pins with a frequency of 50Hz.
        - Starts the PWM channels with a 0% duty cycle.
        - Creates a subscription to the 'vehicle_command' topic to handle incoming commands.
        - Logs that the CarController node has started.
        """
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

        # Set mode LED on pin 20 to indicate 'wait' mode
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.HIGH)

        # Initialize PWM at 50Hz
        self.motor_forward = GPIO.PWM(self.motor_forward_pin, 50)
        self.motor_backward = GPIO.PWM(self.motor_backward_pin, 50)
        self.motor_steering = GPIO.PWM(self.motor_steering_pin, 50)

        # Start PWM with 0% duty cycle
        self.motor_forward.start(0)
        self.motor_backward.start(0)
        self.motor_steering.start(7.5)

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            VehicleCommand,
            'vehicle_command',
            self.command_callback,
            qos_profile,
            10
        )
        self.get_logger().info('CarController node started.')

    def command_callback(self, msg: VehicleCommand):
        """
        Process an incoming VehicleCommand message to control vehicle movement and steering.

        Parameters:
            msg (VehicleCommand): The message containing vehicle command data, including:
                - command: an identifier for the type of command.
                - speed: the desired speed; a positive value drives forward, a negative value drives backward.
                - angle: the steering angle to be set for the vehicle.
        Behavior:
            - Logs the received command details.
            - If msg.speed is greater than zero, invokes drive_forward with the given speed.
            - If msg.speed is less than zero, invokes drive_backward with the absolute value of the speed.
            - If msg.speed equals zero, logs that no movement command was received.
            - Uses the angle value from msg to set the vehicle's steering via set_steering.
        """
        self.get_logger().info(
            f'Received command: command={msg.command}, speed={msg.speed}, angle={msg.angle}'
        )
        if msg.speed > 0:
            self.drive_forward(msg.speed)
        elif msg.speed < 0:
            self.drive_backward(abs(msg.speed))
        else:
            self.motor_forward.ChangeDutyCycle(0)
            self.motor_backward.ChangeDutyCycle(0)
            self.get_logger().info('No movement command received (speed is zero).')

        # Use the angle field for steering
        self.set_steering(msg.angle)

    def drive_forward(self, speed):
        """
        Drive the vehicle forward at the specified speed.

        This method updates the PWM duty cycle for the motor controlling forward movement,
        ensuring that the speed value is clamped between 0 and 100 percent. It sets the backward
        motor's duty cycle to 0 to ensure only forward movement occurs.
        Parameters:
            speed (float or int): The desired speed as a percentage (0 to 100) used to set the
                                  PWM duty cycle for forward motion.
        Side Effects:
            - Modifies the state of motor_forward and motor_backward PWM channels.
            - Logs the driving action with the specified speed.
        """
        pwm_speed = min(max(speed * 100, 0), 100)
        self.motor_forward.ChangeDutyCycle(pwm_speed)
        self.motor_backward.ChangeDutyCycle(0)
        self.get_logger().info(f'PWM: Driving forward at speed {pwm_speed}%')

    def drive_backward(self, speed):
        """
        Drives the vehicle backward at the specified speed.

        This method stops any forward motion by setting its duty cycle to 0 and then activates
        the backward motor. The provided speed is clamped between 0 and 100 to ensure that
        the duty cycle remains within valid PWM bounds. A log message is generated to reflect
        the backward drive operation and the speed used.
        Parameters:
            speed (int or float): The target speed for driving backward. Values outside
                                    the 0-100 range will be clamped accordingly.
        Returns:
            None
        """
        pwm_speed = min(max(speed * 100, 0), 100)
        self.motor_forward.ChangeDutyCycle(0)
        self.motor_backward.ChangeDutyCycle(pwm_speed)
        self.get_logger().info(f'PWM: Driving backward at speed {pwm_speed}%')

    def set_steering(self, angle):
        """
        Set the steering angle by adjusting the PWM duty cycle on the motor.

        The duty cycle is calculated based on the provided angle to ensure proper steering:
            - A neutral (centered) angle results in a duty cycle of 7.5.
            - For negative angles (steering left), the duty cycle is decreased, with a lower bound of 5.0.
            - For positive angles (steering right), the duty cycle is increased, with an upper bound of 10.0.
        Parameters:
            angle (float): The desired steering angle in degrees. Negative values steer left,
                           positive values steer right.
        This method updates the PWM duty cycle accordingly and logs the resulting duty cycle along with the input angle.
        """
        neutral_duty_cycle = 7.5
        minimum_duty_cycle = 5.0
        maximum_duty_cycle = 10.0

        duty_cycle = neutral_duty_cycle + (angle * (neutral_duty_cycle - minimum_duty_cycle))

        duty_cycle = max(min(duty_cycle, maximum_duty_cycle), minimum_duty_cycle)

        self.motor_steering.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f'PWM: Steering with angle {angle}, duty_cycle {duty_cycle}')


def main(args=None):
    """
    Run the CarController node.

    This function initializes the rclpy system with any provided arguments,
    creates an instance of the CarController node, and enters a loop to process
    ROS messages using rclpy.spin(). It handles a KeyboardInterrupt gracefully,
    and on shutdown, it destroys the node, cleans up GPIO resources, and shuts down rclpy.
    Args:
        args (Optional[List[str]]): Command-line arguments to pass to rclpy initialization.
    Returns:
        None
    """
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
