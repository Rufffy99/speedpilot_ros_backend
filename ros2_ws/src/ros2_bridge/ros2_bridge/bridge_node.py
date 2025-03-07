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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        self.cmd_publisher = self.create_publisher(String, 'vehicle_command', 10)
        self.get_logger().info('Starting WebSocket server...')
        self.websocket_thread = threading.Thread(
            target=self.run_websocket_server, daemon=True
        )
        self.websocket_thread.start()

    def run_websocket_server(self):
        """Start the WebSocket server in an infinite loop so that it restarts upon any error."""
        patch_websocket_server()  # Apply monkey patch before starting the server
        while True:
            try:
                self.get_logger().info('WebSocket server running on port 9090...')
                server = WebsocketServer(host='0.0.0.0', port=9090)
                server.set_fn_new_client(self.on_new_client)
                server.set_fn_message_received(self.websocket_handler)
                server.set_fn_client_left(self.on_client_disconnect)
                server.run_forever()
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
        """Process incoming WebSocket messages and send them to ROS2."""
        try:
            if not isinstance(data, dict):
                self.get_logger().warning(f'Invalid JSON format: {data}')
                return
            if 'command' in data:
                msg = String()
                msg.data = data['command']
                self.cmd_publisher.publish(msg)
                self.get_logger().info(f'Sent: {msg.data}')
            else:
                self.get_logger().warning(
                    f'JSON message without "command" key received: {data}'
                )
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')


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
