# zmq_broker.py
# version: 1.1.0
# Original Author: Theodore Tasman
# Creation Date: 2025-09-24
# Last Modified: 2025-09-24
# Organization: PSU UAS

from typing import Any
import zmq
from MAVez.translate_message import translate_message
import json

class ZMQBroker():
    """
    A ZeroMQ broker for publishing messages. 

    Args:
        host (str): The hostname to bind the ZeroMQ socket. Default is "localhost".
        port (int): The port number to bind the ZeroMQ socket. Default is 5555.
    """

    def __init__(self, host: str = "localhost", port: int = 5555):
        self.host = host
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{self.host}:{self.port}")

    async def publish(self, topic: str, message: Any) -> None:
        """
        Publish a message to a specified topic. The topic will be combined with the message type.

        Args:
            topic (str): The topic to publish the message to.
            message (Any): The MAVLink message to publish.

        Returns:
            None
        """
        message_type = message.get_type()
        data = translate_message(message)
        self.socket.send_string(f"{topic}_{message_type} {json.dumps(data)}")

    def close(self) -> None:
        """
        Close the ZeroMQ socket and terminate the context.

        Returns:
            None
        """
        self.socket.close()
        self.context.term()

