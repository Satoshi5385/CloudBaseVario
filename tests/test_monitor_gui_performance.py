"""Regression tests for monitor GUI backpressure and chart reduction."""

from __future__ import annotations

import unittest

from tools.monitor_gui.cloudbasevario_serial import SerialEventInbox
from tools.monitor_gui.cloudbasevario_widgets import downsample_values


class SerialEventInboxTests(unittest.TestCase):
    def test_periodic_telemetry_is_coalesced_independently(self) -> None:
        inbox = SerialEventInbox()

        for sequence in range(10000):
            inbox.put(("line", f"BARO seq={sequence}"))
        for sequence in range(20):
            inbox.put(("line", f"GPS sequence={sequence}"))

        baro, gps = inbox.take_latest_lines()
        self.assertEqual(baro, "BARO seq=9999")
        self.assertEqual(gps, "GPS sequence=19")
        self.assertEqual(inbox.control_event_count, 0)
        self.assertLessEqual(len(inbox.drain_telemetry_log()), 5000)

    def test_control_events_keep_fifo_order(self) -> None:
        inbox = SerialEventInbox()
        inbox.put(("line", "PARAM alpha=one"))
        inbox.put(("line", "OK"))
        inbox.put(("serial_error", "read failed"))

        self.assertEqual(inbox.get_nowait(), ("line", "PARAM alpha=one"))
        self.assertEqual(inbox.get_nowait(), ("line", "OK"))
        self.assertEqual(inbox.get_nowait(), ("serial_error", "read failed"))


class ChartReductionTests(unittest.TestCase):
    def test_downsampling_retains_bucket_extrema(self) -> None:
        values = [(float(index), float(index % 17)) for index in range(600)]

        sampled = downsample_values(values, 120)

        self.assertLessEqual(len(sampled), 120)
        self.assertEqual(min(value for _, value in sampled), 0.0)
        self.assertEqual(max(value for _, value in sampled), 16.0)
        self.assertEqual(sampled[0][0], values[0][0])
        self.assertEqual(sampled[-1][0], values[-1][0])

    def test_downsampling_preserves_invalid_boundaries(self) -> None:
        values = [(float(index), None if index in (0, 599) else 1.0)
                  for index in range(600)]

        sampled = downsample_values(values, 120)

        self.assertIsNone(sampled[0][1])
        self.assertIsNone(sampled[-1][1])


if __name__ == "__main__":
    unittest.main()
