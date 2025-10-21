"""
A simple ZMQ subscriber example that connects to a ZMQ the controller's built-in broker and listens for all messages.

To run this example, first start the controller with ZMQ broker enabled:
    python examples/basic_messaging.py
Then run this subscriber script:
    python examples/message_receiver.py
"""

import asyncio
from uas_messenger.subscriber import Subscriber

async def callback(msg):
    print(msg)

async def main():
    subscriber = Subscriber(host='127.0.0.1', port=5555, callback=callback)
    subscriber.start()
    while True:
        try:
            await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down subscriber...")
            await subscriber.close()
            break

if __name__ == "__main__":
    asyncio.run(main())