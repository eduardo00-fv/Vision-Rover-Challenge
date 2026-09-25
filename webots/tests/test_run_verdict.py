"""El tiempo solo cuenta si la misión se completa sin faltas."""

import sys
import unittest
from pathlib import Path

WEBOTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEBOTS / "controllers" / "vision_supervisor"))

from run_verdict import evaluate  # noqa: E402


EXPECTED = {"required_deliveries": ["green", "blue"]}
DELIVERED = {"green": {"delivered": True}, "blue": {"delivered": True}}


class RunVerdictTest(unittest.TestCase):
    def test_incomplete_delivery_fails_even_without_collision(self):
        result = evaluate(EXPECTED, {"green": {"delivered": True}}, [], 1000)
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["missing_deliveries"], ["blue"])
        self.assertIsNone(result["completion_ms"])

    def test_safety_failure_disqualifies_a_fast_delivery(self):
        result = evaluate(EXPECTED, DELIVERED, ["rover_contact:10-11"], 500)
        self.assertEqual(result["status"], "FAIL")
        self.assertIsNone(result["completion_ms"])

    def test_complete_safe_run_records_official_time(self):
        result = evaluate(EXPECTED, DELIVERED, [], 12345, official_completion=True)
        self.assertEqual(result["status"], "INCONCLUSIVE")
        self.assertEqual(result["completion_ms"], 12345)

    def test_geometry_alone_cannot_pass_after_timeout_or_manual_stop(self):
        result = evaluate(EXPECTED, DELIVERED, [], 12345, physics_calibrated=True)
        self.assertEqual(result["status"], "FAIL")
        self.assertIsNone(result["completion_ms"])


if __name__ == "__main__":
    unittest.main()
