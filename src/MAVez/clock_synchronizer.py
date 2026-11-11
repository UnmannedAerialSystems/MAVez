# clock_synchronizer.py
# version: 1.0.0
# Original Author: Taiyo Molessa
# Creation Date: 2026-11-11
# Last Modified: 2026-11-11
# Organization: PSU UAS

import time
import asyncio

class ClockSynchronizer:
    """
        Synchronizes ArduPilot time with local monotonic time.
        offset = ArduPilot time - local monotonic time.
    """
    def __init__(self):
        self.offset_ns: int | None = None
        self.rtt_ns: int | None = None
        self.pending_ts1: int | None = None
        self.pending_response: asyncio.Future | None = None

    def process_timesync_response(self, t1_ns: int, tc1_ns: int, t4_ns: int) -> None:
        """
            Processes a timesync response from ArduPilot and calculates the offset, midpoint, and round-trip time.

            Args:
                t1_ns (int): Local monotonic time when the request was sent (in nanoseconds).
                tc1_ns (int): ArduPilot time when the request was received (in nanoseconds).
                t4_ns (int): Local monotonic time when the response was received (in nanoseconds).
        """
        self.rtt_ns = t4_ns - t1_ns # RTT
        midpoint = (t1_ns + t4_ns) // 2
        self.offset_ns = tc1_ns - midpoint # Offset

    def handle_timesync_message(self, message: dict, received_ns: int) -> None:
        """
            Handles a timesync message from ArduPilot. If the message is a response to a pending request,
            it processes the response and calculates the offset and round-trip time.

            Args:
                message (dict): The timesync message received from ArduPilot.
                received_ns (int): Local monotonic time when the message was received (in nanoseconds).
        """
        if self.pending_ts1 is not None and self.pending_response is not None and message.get('ts1') == self.pending_ts1 and message.get('tc1') != 0:
            tc1_ns = message['tc1']
            if not self.pending_response.done():
                self.process_timesync_response(self.pending_ts1, tc1_ns, received_ns)
                self.pending_response.set_result(True)

    def to_autopilot_time(self, local_ns: int) -> int | None:
        """
            Converts local monotonic time to ArduPilot time using the calculated offset.

            Args:
                local_ns (int): Local monotonic time in nanoseconds.

            Returns:
                int | None: Corresponding ArduPilot time in nanoseconds, or None if offset is not set.
        """
        if self.offset_ns is None:
            return None
        return local_ns + self.offset_ns

    def to_local_time(self, autopilot_ns: int) -> int | None:
        """
            Converts ArduPilot time to local monotonic time using the calculated offset.

            Args:
                autopilot_ns (int): ArduPilot time in nanoseconds.

            Returns:
                int | None: Corresponding local monotonic time in nanoseconds, or None if offset is not set.
        """
        if self.offset_ns is None:
            return None
        return autopilot_ns - self.offset_ns

if __name__ == "__main__":
    clock = ClockSynchronizer()

    clock.process_timesync_response(
        t1_ns=1_000_000_000,
        tc1_ns=6_010_000_000,
        t4_ns=1_020_000_000
    )

    assert clock.rtt_ns == 20_000_000
    assert clock.offset_ns == 5_000_000_000

    print("passed")