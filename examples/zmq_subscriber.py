"""
A simple ZMQ subscriber example that connects to a ZMQ the controller's built-in broker and listens for all messages.

To run this example, first start the controller with ZMQ broker enabled:
    python examples/basic_with_zmq.py
Then run this subscriber script:
    python examples/zmq_subscriber.py
"""

import zmq
import json
import time

class ZMQSubscriber:
    def __init__(self, host, port, topic):
        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(f"tcp://{host}:{port}")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.topic = topic

    def recv(self, flags=0):
        try:
            raw = self.sub.recv_string(flags=flags)
            if raw is None:
                return None
            topic, payload = raw.split(" ", 1)
            if topic.startswith(self.topic):
                return payload
            return None
        except zmq.Again:
            return None
        except Exception as e:
            print(f"[ZMQSubscriber] Error: {e}")
            return None

def main():
    subscriber = ZMQSubscriber(host='localhost', port=5555, topic='mavlink')
    while True:
        try:
            msg = subscriber.recv()
            if msg is None:
                continue
            print(json.loads(msg))
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()