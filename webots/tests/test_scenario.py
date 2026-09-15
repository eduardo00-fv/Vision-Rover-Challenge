"""Pruebas de carga para escenarios reproducibles de Webots."""

import unittest
from pathlib import Path
import sys

WEBOTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEBOTS / "controllers" / "vision_supervisor"))

from scenario import load_scenario  # noqa: E402


class ScenarioTest(unittest.TestCase):
    def test_nominal_uses_the_contract_configuration(self):
        scenario, config = load_scenario(WEBOTS / "scenarios" / "nominal_3_cubos.json")
        self.assertEqual(scenario["id"], "nominal_3_cubos")
        self.assertEqual([rover["id"] for rover in config["rovers"]], [10, 11])
        self.assertEqual({cube["color"] for cube in config["cubes"]}, {"green", "blue", "red"})
        self.assertEqual(
            {key: config["grid"][key] for key in ("cols", "rows", "cell_mm")},
            {"cols": 43, "rows": 43, "cell_mm": 20.0},
        )


if __name__ == "__main__":
    unittest.main()
