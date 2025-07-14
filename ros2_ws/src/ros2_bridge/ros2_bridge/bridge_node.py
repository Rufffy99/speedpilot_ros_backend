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
ROSBridge Node.

Run a ROS2 node that hosts a WebSocket server to bridge commands from external clients.
"""

import json
import threading
import time

import RPi.GPIO as GPIO

from custom_msgs.msg import VehicleCommand

from geometry_msgs.msg import PoseWithCovarianceStamped

from google.protobuf.json_format import MessageToDict

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from websocket_server import WebsocketServer


def patch_websocket_server():
    """
    Override the handshake method of the WebSocketHandler class to catch all errors.

    Invalid connection attempts (e.g., HTTP requests without an 'upgrade' header)
    are ignored without crashing the server.
    """
    try:
        import websocket_server.websocket_server as wss
    except ImportError:
        return

    original_handshake = wss.WebSocketHandler.handshake

    def safe_handshake(self, *args, **kwargs):
        try:
            return original_handshake(self, *args, **kwargs)
        except Exception:
            try:
                self.server._client_left_(self)
            except Exception:
                pass
            return

    safe_handshake.__name__ = 'safe_handshake'
    wss.WebSocketHandler.handshake = safe_handshake


class ROSBridge(Node):
    """
    Bridge between a WebSocket server and ROS2 topics.

    This ROS2 node initializes a WebSocket server that listens for incoming
    connections and messages, processes the messages, and publishes them to a
    ROS2 topic.

    Attributes
    ----------
    cmd_publisher (Publisher)
        A ROS2 publisher for sending vehicle commands.
    websocket_thread (Thread)
        A thread running the WebSocket server.

    Methods
    -------
    run_websocket_server()
        Start the WebSocket server in an infinite loop.
    on_new_client(client, server)
        Handle new client connections.
    on_client_disconnect(client, server)
        Handle client disconnections safely.
    websocket_handler(client, server, message)
        Process incoming WebSocket messages.
    process_message(data)
        Process incoming WebSocket messages and send them to ROS2.

    """

    def __init__(self):
        """Initialize the ROSBridge node and start the WebSocket server."""
        super().__init__('ros_bridge')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(16, GPIO.OUT)
        GPIO.output(16, GPIO.HIGH)
        self.cmd_publisher = self.create_publisher(VehicleCommand, 'vehicle_command', 10)
        self.get_logger().info('Starting WebSocket server...')
        self.websocket_thread = threading.Thread(
            target=self.run_websocket_server, daemon=True
        )
        self.websocket_thread.start()
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose',
            self.pose_callback,
            10
        )
        self.ws_clients = []

    def run_websocket_server(self):
        """Start the WebSocket server in an infinite loop so that it restarts upon any error."""
        patch_websocket_server()
        while True:
            try:
                self.get_logger().info('WebSocket server running on port 9090...')
                self.server = WebsocketServer(host='0.0.0.0', port=9090)
                self.server.set_fn_new_client(self.on_new_client)
                self.server.set_fn_message_received(self.websocket_handler)
                self.server.set_fn_client_left(self.on_client_disconnect)
                self.server.run_forever()
            except Exception as e:
                self.get_logger().error(f'WebSocket error: {e}')
                self.get_logger().info('Restarting WebSocket server in 5 seconds...')
                time.sleep(5)

    def on_new_client(self, client, server):
        """Handle new client connections."""
        try:
            if client is None:
                self.get_logger().warning('An unknown client has connected.')
                return
            self.ws_clients.append(client)
            self.get_logger().info(
                f'New WebSocket client connected: {client.get("id", "unknown")}'
            )
        except Exception as e:
            self.get_logger().warning(f'Error during client connection: {e}')

    def on_client_disconnect(self, client, server):
        """Handle client disconnections safely."""
        try:
            if client is None:
                self.get_logger().warning('An unknown client disconnected.')
                return
            if client in self.ws_clients:
                self.ws_clients.remove(client)
            self.get_logger().info(
                f'WebSocket client {client.get("id", "unknown")} has disconnected.'
            )
        except Exception as e:
            self.get_logger().warning(f'Error during client disconnection: {e}')

    def websocket_handler(self, client, server, message):
        """Process incoming WebSocket messages."""
        try:
            data = json.loads(message)
            self.process_message(data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                f'Invalid JSON received from client {client.get("id", "unknown")}: {message}'
            )
        except KeyError as e:
            self.get_logger().warning(
                f'Missing key in message: {e} - Message: {message}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Unexpected error from client {client.get("id", "unknown")}: {e}'
            )

    def process_message(self, data):
        """
        Process an incoming message command and act accordingly.

        This method expects the input data to be a dictionary representing a JSON-formatted message.
        It handles different commands as follows:
            - "move": Requires 'speed' and 'angle' keys. Publishes a VehicleCommand with the specified values.
            - "status": Logs a status request (functionality not implemented).
            - "stop": Publishes a VehicleCommand with speed and angle set to zero.
        If the input is not a dictionary, or if required keys for a command are missing, a warning is logged.
        Any exceptions during processing are caught and logged as errors.
        Parameters:
            data (dict): A dictionary containing a 'command' key and other relevant keys based on the command type.
        Returns:
            None
        """
        try:
            if not isinstance(data, dict):
                self.get_logger().warning(f'Invalid JSON format: {data}')
                return

            cmd = data.get('command', None)
            if cmd == 'move':
                if 'speed' in data and 'angle' in data:
                    msg = VehicleCommand()
                    msg.command = 'move'

                    msg.speed = float(data['speed'])
                    msg.angle = float(data['angle'])
                    self.cmd_publisher.publish(msg)
                    self.get_logger().info(
                        f'Published command: command={msg.command} speed={msg.speed}, angle={msg.angle}'
                    )
                else:
                    self.get_logger().warning('Move command missing speed or angle.')
            elif cmd == 'status':
                self.get_logger().info('Status request received (not implemented).')
                # Optional: Send a reply via WebSocket (z. B. server.send_message(client, "..."))
            elif cmd == 'stop':
                msg = VehicleCommand()
                msg.speed = 0.0
                msg.angle = 0.0
                self.cmd_publisher.publish(msg)
                self.get_logger().info('Stop command issued.')
            else:
                self.get_logger().warning(f'Unknown command type: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

    def lidar_callback(self, msg):
        """
        Forward lidar data as JSON to all connected WebSocket clients.

        Parameters:
            msg (object): A message object (typically of type sensor_msgs.msg.LaserScan)
                containing lidar data. The object is expected to have:
                - header.stamp.sec (int): The seconds part of the timestamp.
                - header.stamp.nanosec (int): The nanoseconds part of the timestamp.
                - angle_min (float): The minimum angle of the scan.
                - angle_max (float): The maximum angle of the scan.
                - angle_increment (float): The angular increment between consecutive measurements.
                - ranges (Iterable[float]): The range measurements from the lidar sensor.
                - intensities (Iterable[float]): The intensity values corresponding to each range reading.

        Raises:
            Exception: If there is an error sending the JSON message to any client,
                the exception is caught and an error message is logged.

        Returns:
            None
        """
        data = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
        }
        json_data = json.dumps({'lidar': data})

        for client in self.ws_clients:
            try:
                self.server.send_message(client, json_data)
            except Exception as e:
                self.get_logger().error(f"Error sending to client {client.get('id', 'unknown')}: {e}")

    def map_callback(self, msg):
        """Forward full OccupancyGrid message as JSON to all connected WebSocket clients."""
        try:
            # Convert ROS message to dictionary and then to JSON
            msg_dict = MessageToDict(msg, preserving_proto_field_name=True, including_default_value_fields=True)
            json_data = json.dumps({'map': msg_dict})
            for client in self.ws_clients:
                try:
                    self.server.send_message(client, json_data)
                except Exception as e:
                    self.get_logger().error(f"Error sending map to client {client.get('id', 'unknown')}: {e}")
        except Exception as e:
            self.get_logger().error(f'Map callback error: {e}')

    def pose_callback(self, msg):
        """Forward pose data as JSON to all connected WebSocket clients."""
        try:
            data = {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'covariance': list(msg.pose.covariance)
            }
            json_data = json.dumps(data)
            for client in self.ws_clients:
                try:
                    self.server.send_message(client, json_data)
                except Exception as e:
                    self.get_logger().error(f"Error sending pose to client {client.get('id', 'unknown')}: {e}")
        except Exception as e:
            self.get_logger().error(f'Pose callback error: {e}')


def main(args=None):
    """
    Run the ROS2 Bridge node.

    This function initializes the ROS2 Python client library, creates an instance
    of the ROSBridge class, and starts spinning the node to process callbacks.
    It handles keyboard interrupts and other exceptions, logs errors, and ensures
    proper shutdown of the node and the ROS2 system.

    :param args: Command-line arguments passed to the ROS2 Python client library. Defaults to None.
    :type args: list or None
    """
    rclpy.init(args=args)
    ros_bridge = ROSBridge()
    try:
        rclpy.spin(ros_bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        ros_bridge.get_logger().error(f'ROS2 Bridge error: {e}')
    finally:
        ros_bridge.get_logger().info('Shutting down ROS2 Bridge...')
        ros_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
