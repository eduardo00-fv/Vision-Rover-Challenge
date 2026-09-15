"""Pruebas de la geometría usada para reprobar incidentes de seguridad."""

import sys
import unittest
from pathlib import Path

WEBOTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEBOTS / "controllers" / "vision_supervisor"))

from safety_oracle import evaluate  # noqa: E402


GRID = {"cols": 43, "rows": 43, "cell_mm": 20.0}


def truth(*rovers):
    return {"rovers": [{"id": rover_id, "col": col, "row": row, "theta": theta}
                       for rover_id, col, row, theta in rovers], "cubes": []}


class SafetyOracleTest(unittest.TestCase):
    def test_separated_rovers_are_safe(self):
        report = evaluate(truth((10, 8, 8, 0), (11, 20, 8, 0)), GRID, 120, 100)
        self.assertEqual(report["failures"], [])

    def test_oriented_chassis_contact_is_a_failure(self):
        report = evaluate(truth((10, 10, 10, 0), (11, 15, 10, 0)), GRID, 120, 100)
        self.assertEqual(report["failures"], ["rover_contact:10-11"])

    def test_chassis_crossing_the_boundary_is_a_failure(self):
        report = evaluate(truth((10, 1, 21.5, 0), (11, 20, 20, 0)), GRID, 120, 100)
        self.assertIn("rover_out_of_bounds:10", report["failures"])

    def test_clearance_is_reported_without_calling_it_a_contact(self):
        report = evaluate(truth((10, 10, 10, 0), (11, 17, 10, 0)), GRID, 120, 100, 25)
        self.assertEqual(report["contacts"], [])
        self.assertEqual(report["proximity"][0]["rovers"], [10, 11])


if __name__ == "__main__":
    unittest.main()
