"""La fase de la simulación debe obedecer al árbitro de visión oficial."""

import sys
import unittest
from pathlib import Path

WEBOTS = Path(__file__).resolve().parents[1]
ROOT = WEBOTS.parent
sys.path.insert(0, str(ROOT / "vision-system"))

from vision.configuracion import cargar_config  # noqa: E402
from vision.sistema import Arbitro  # noqa: E402


class RefereeIntegrationTest(unittest.TestCase):
    def test_ready_requires_geometry_and_running_is_automatic(self):
        now = [0.0]
        cfg = cargar_config(str(ROOT / "vision-system" / "vision" / "config_vision.json"))
        referee = Arbitro(cfg, reloj=lambda: now[0])
        self.assertIn("no se puede preparar", referee.intentar("ready"))
        referee.tictac(True)
        self.assertEqual(referee.intentar("ready"), "fase: IDLE -> READY")
        now[0] += cfg.ronda.preparacion_ms / 1000.0
        self.assertIn("READY -> RUNNING", referee.tictac(True))
        self.assertEqual(referee.fase, "RUNNING")


if __name__ == "__main__":
    unittest.main()
