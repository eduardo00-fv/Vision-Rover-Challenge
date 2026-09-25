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

    def test_completion_closes_round_and_requires_a_new_ready(self):
        now = [0.0]
        cfg = cargar_config(str(ROOT / "vision-system" / "vision" / "config_vision.json"))
        referee = Arbitro(cfg, reloj=lambda: now[0])
        referee.tictac(True)
        referee.intentar("ready")
        now[0] += cfg.ronda.preparacion_ms / 1000.0
        referee.tictac(True)
        referee.observar_reto(False, None)
        now[0] += 2
        self.assertIn("reto cumplido", referee.observar_reto(True, now[0] - 1))
        self.assertEqual(referee.fase, "FINISHED")
        self.assertIsNone(referee.observar_reto(True, now[0]))
        self.assertIn("FINISHED -> READY", referee.intentar("ready"))


if __name__ == "__main__":
    unittest.main()
