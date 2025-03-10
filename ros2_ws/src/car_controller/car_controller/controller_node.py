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

import rclpy
from rclpy.node import Node

from custom_msgs.msg import VehicleCommand


class CarController(Node): # noqa
    def __init__(self): # noqa
        super().__init__('car_controller')
        self.subscription = self.create_subscription(
            VehicleCommand,
            'vehicle_command',
            self.command_callback,
            10
        )
        self.get_logger().info('CarController node started.')

    def command_callback(self, msg): # noqa
        self.get_logger().info(
            f'Received command: speed={msg.speed}, angle={msg.angle}'
        )
        # Die Richtung wird jetzt durch das Vorzeichen der Geschwindigkeit bestimmt:
        if msg.speed > 0:
            self.drive_forward(msg.speed)
        elif msg.speed < 0:
            self.drive_backward(abs(msg.speed))
        else:
            self.get_logger().info('No movement command received (speed is zero).')

        # Benutze das angle-Feld für die Lenkung
        self.set_steering(msg.angle)

    def drive_forward(self, speed): # noqa
        self.get_logger().info(f'Accelerating forward at speed {speed}.')
        # Hier den Code für Hardware oder Simulation einfügen

    def drive_backward(self, speed): # noqa
        self.get_logger().info(f'Accelerating backward at speed {speed}.')
        # Hier den Code für Hardware oder Simulation einfügen

    def set_steering(self, angle): # noqa
        self.get_logger().info(f'Setting steering angle to {angle} degrees.')
        # Hier den Code für Hardware oder Simulation einfügen

def main(args=None): # noqa
    rclpy.init(args=args)
    node = CarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
