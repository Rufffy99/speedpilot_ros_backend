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

"""
Ultrasonic Sensor ROS 2 Node.

This module implements a ROS 2 node (`UltrasonicSensorNode`) that interfaces with an ultrasonic distance
sensor (e.g., HC-SR04) connected via GPIO on a Raspberry Pi. The node periodically measures the distance
to nearby obstacles and publishes the result as a `std_msgs/Float32` message on the `/ultrasonic/distance` topic.

The sensor uses a trigger/echo mechanism:
- A short pulse is sent via the trigger pin.
- The echo pin listens for the reflected pulse.
- The time between send and receive is used to calculate the distance using the speed of sound.

This node is designed for integration into autonomous vehicle systems, particularly for obstacle avoidance logic.

GPIO configuration:
- Trigger pin (output): GPIO 11 (green wire)
- Echo pin (input): GPIO 9 (blue wire)
"""


import time

import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


TRIG_PIN = 11  # trigger (green cable)
ECHO_PIN = 9   # echo (blue cable)


class UltrasonicSensorNode(Node):
    """
    UltrasonicSensorNode.

    A ROS2 node for interfacing with an ultrasonic sensor to measure distances and publish them as Float32
    messages on the '/ultrasonic/distance' topic.
    Methods:
        __init__():
            Initializes the node by setting up the ROS publisher and timer for periodic distance measurements.
            Also configures the GPIO pins for the ultrasonic sensor trigger and echo operations.
        read_and_publish():
            Calls get_distance() to acquire a measurement.
            If the measurement is valid, wraps the distance in a Float32 message and publishes it.
            Logs the measured distance or a warning if the measurement is invalid.
        get_distance():
            Triggers the ultrasonic sensor by sending a pulse and waits for the echo response.
            Implements timeout mechanisms to avoid indefinite blocking while waiting for the sensor's response.
            Calculates and returns the distance based on the elapsed time and the speed of sound if the value is
            within a valid range (approximately 0.02 to 4.0 meters).
            Returns None if a timeout occurs or if the calculated distance is out of range.
        destroy_node():
            Performs cleanup of GPIO resources to prevent resource leaks.
            Calls the superclass's destroy_node() to handle additional shutdown procedures.
    """

    def __init__(self):
        """
        Initialize the ultrasonic sensor node.

        This constructor initializes the ROS2 node with a dedicated name, creates a publisher for
        Float32 messages on the '/ultrasonic/distance' topic, and sets a timer to periodically call
        the read_and_publish method every 0.2 seconds. It also configures the GPIO pins by setting the
        appropriate mode, designating TRIG_PIN as an output and ECHO_PIN as an input, and ensuring that
        TRIG_PIN starts in a low state. Finally, it logs an informational message indicating that the
        ultrasonic sensor has been initialized.
        """
        super().__init__('ultrasonic_sensor_node')
        self.publisher_ = self.create_publisher(Float32, '/ultrasonic/distance', 10)
        self.timer = self.create_timer(0.2, self.read_and_publish)

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)
        GPIO.output(TRIG_PIN, False)
        self.get_logger().info('Ultraschallsensor initialisiert (GPIO 9/11)')

    def read_and_publish(self):
        """
        Read sensor data and publish the corresponding message.

        This method retrieves a distance measurement using get_distance().
        If the measurement is valid, it wraps the distance in a Float32 message,
        publishes it, and logs the measured distance. If the measurement is invalid,
        it logs a warning indicating an invalid reading.
        """
        distance = self.get_distance()
        if distance is not None:
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)
            self.get_logger().info(f'Distanz: {distance:.2f} m')
        else:
            self.get_logger().warning('Ungültige Messung')

    def get_distance(self):
        """
        Measure the distance in meters using an ultrasonic sensor.

        This method sends a trigger pulse, waits for the echo start and end signals,
        and calculates the distance based on the elapsed time and the speed of sound.
        It returns the distance if it falls within the valid range of 0.02 to 4.0 meters;
        otherwise, it returns None. Timeout mechanisms ensure the function does not block indefinitely.
        Returns:
            float or None: The measured distance in meters if within the valid range,
            or None if a timeout occurs or the measurement is out of range.
        """
        # Trigger senden
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)

        start_time = time.time()
        stop_time = time.time()

        # Warte auf Echo Start
        timeout_start = time.time()
        while GPIO.input(ECHO_PIN) == 0:
            start_time = time.time()
            if time.time() - timeout_start > 0.02:
                return None  # Timeout

        # Warte auf Echo Ende
        timeout_start = time.time()
        while GPIO.input(ECHO_PIN) == 1:
            stop_time = time.time()
            if time.time() - timeout_start > 0.02:
                return None  # Timeout

        # Dauer und Distanz berechnen
        elapsed = stop_time - start_time
        distance = (elapsed * 343.0) / 2  # in Metern

        if 0.02 < distance < 4.0:
            return distance
        else:
            return None

    def destroy_node(self):
        """Destroys the node and cleans up GPIO resources.

        This method cleans up the GPIO to prevent resource leaks and then calls the superclass's
        destroy_node method to complete the remaining shutdown procedures.
        """
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    """
    Initialize the ROS 2 client, create an UltrasonicSensorNode instance, and spin the node.

    to process callbacks until an interruption occurs. On shutdown, the node is properly
    destroyed and ROS 2 is gracefully shut down.
    Parameters:
        args (list, optional): Command-line arguments to initialize ROS 2 with. Defaults to None.
    """
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
