"""Pruebas sin Webots para el estado que publica el supervisor."""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "vision_supervisor"))

from contract_state import round_clock  # noqa: E402


class RoundClockTest(unittest.TestCase):
    PREPARATION_MS = 60_000
    DURATION_MS = 600_000

    def test_idle_does_not_count(self):
        self.assertEqual(
            round_clock("IDLE", 123, self.PREPARATION_MS, self.DURATION_MS),
            {"elapsed_ms": 0, "remaining_ms": 0, "total_ms": 0},
        )

    def test_ready_uses_preparation_duration(self):
        self.assertEqual(
            round_clock("READY", 12_345, self.PREPARATION_MS, self.DURATION_MS),
            {"elapsed_ms": 12_345, "remaining_ms": 47_655, "total_ms": 60_000},
        )

    def test_finished_preserves_unused_round_time(self):
        self.assertEqual(
            round_clock("FINISHED", 123_456, self.PREPARATION_MS, self.DURATION_MS),
            {"elapsed_ms": 123_456, "remaining_ms": 476_544, "total_ms": 600_000},
        )

    def test_elapsed_is_bounded_to_the_phase(self):
        self.assertEqual(
            round_clock("RUNNING", 700_000, self.PREPARATION_MS, self.DURATION_MS),
            {"elapsed_ms": 600_000, "remaining_ms": 0, "total_ms": 600_000},
        )


if __name__ == "__main__":
    unittest.main()
