# noqa
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


import json # noqa
import time
import unittest

import rclpy
from ros2_bridge.bridge_node import patch_websocket_server, ROSBridge # noqa


class TestROSBridge(unittest.TestCase):
    """Tests for the ROSBridge, including message processing and the monkey patch."""

    @classmethod
    def setUpClass(cls): # noqa
        rclpy.init(args=None)
        cls.node = ROSBridge()
        # Wait briefly for the WebSocket server to start
        time.sleep(1)

    @classmethod
    def tearDownClass(cls): # noqa
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self): # noqa
        self.published_messages = []

        def dummy_publish(msg):
            self.published_messages.append(msg.data)

        self.node.cmd_publisher.publish = dummy_publish

    def test_process_message_valid(self):
        """Valid JSON dictionary with 'command' key is processed."""
        data = {'command': 'vorwärts'}
        self.node.process_message(data)
        self.assertEqual(len(self.published_messages), 1)
        self.assertEqual(self.published_messages[0], 'vorwärts')

    def test_process_message_invalid_format(self):
        """Non-dictionary does not trigger publication."""
        data = 'das ist kein dict'
        self.node.process_message(data)
        self.assertEqual(len(self.published_messages), 0)

    def test_process_message_missing_command(self):
        """Missing 'command' key results in no publication."""
        data = {'not_command': 'wert'}
        self.node.process_message(data)
        self.assertEqual(len(self.published_messages), 0)

    def test_websocket_handler_valid(self):
        """Simulate the WebSocket handler with valid JSON."""
        client = {'id': 1}
        message = json.dumps({'command': 'stop'})
        self.node.websocket_handler(client, None, message)
        self.assertEqual(len(self.published_messages), 1)
        self.assertEqual(self.published_messages[0], 'stop')

    def test_websocket_handler_invalid_json(self):
        """Simulate invalid JSON; process_message should not be called."""
        client = {'id': 2}
        message = 'kein json'
        original_process_message = self.node.process_message
        self.node.process_message = lambda data: self.fail(
            'process_message should not be called for invalid JSON'
        )
        try:
            self.node.websocket_handler(client, None, message)
        except Exception:
            self.fail('websocket_handler raised an exception for invalid JSON.')
        finally:
            self.node.process_message = original_process_message

    def test_on_new_client(self):
        """Test on_new_client with valid client and None."""
        valid_client = {'id': 42}
        try:
            self.node.on_new_client(valid_client, None)
            self.node.on_new_client(None, None)
        except Exception:
            self.fail('on_new_client raised an exception.')

    def test_on_client_disconnect(self):
        """Test on_client_disconnect with valid client and None."""
        valid_client = {'id': 42}
        try:
            self.node.on_client_disconnect(valid_client, None)
            self.node.on_client_disconnect(None, None)
        except Exception:
            self.fail('on_client_disconnect raised an exception.')

    def test_patch_websocket_server(self):
        """
        Test the monkey patch of the handshake() method.

        After patching, the method name should be 'safe_handshake'.
        """
        try:
            from websocket_server.websocket_server import WebSocketHandler
        except ImportError:
            self.skipTest('websocket_server module not available')
        original_handshake = WebSocketHandler.handshake
        patch_websocket_server()
        try:
            from websocket_server.websocket_server import WebSocketHandler
            patched_handshake = WebSocketHandler.handshake
            if original_handshake is not None:
                self.assertEqual(patched_handshake.__name__, 'safe_handshake')
        except Exception:
            self.fail('patch_websocket_server did not work correctly.')


if __name__ == '__main__':
    unittest.main()
