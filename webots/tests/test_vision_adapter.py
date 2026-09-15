"""El puente usa imágenes y el pipeline oficial, no poses de Webots."""

import sys
import unittest
from pathlib import Path

import numpy as np

WEBOTS = Path(__file__).resolve().parents[1]
ROOT = WEBOTS.parent
sys.path.insert(0, str(WEBOTS / "controllers" / "vision_supervisor"))
sys.path.insert(0, str(ROOT / "vision-system"))

from contrato.schema import validate_message  # noqa: E402
from vision.configuracion import cargar_config  # noqa: E402
from vision.sources.generador_sintetico import FuenteSintetica  # noqa: E402
from vision_adapter import VisionAdapter, webots_bgra_to_bgr  # noqa: E402


class VisionAdapterTest(unittest.TestCase):
    def test_bgra_conversion_preserves_bgr_channels(self):
        raw = bytes((3, 2, 1, 255, 30, 20, 10, 255))
        image = webots_bgra_to_bgr(raw, 2, 1)
        self.assertEqual(image.tolist(), [[[3, 2, 1], [30, 20, 10]]])

    def test_official_pipeline_produces_a_valid_contract_message(self):
        config_path = ROOT / "vision-system" / "vision" / "config_vision.json"
        cfg = cargar_config(str(config_path))
        with FuenteSintetica(cfg) as source:
            frame = source.leer()
        adapter = VisionAdapter(config_path, frame.imagen.shape[1], frame.imagen.shape[0], 0.85)
        message = adapter.observe(
            frame.imagen, frame.ts_ms, "IDLE",
            {"elapsed_ms": 0, "remaining_ms": 0, "total_ms": 0}, 1,
        )
        self.assertIsNone(validate_message(message))
        self.assertIsNone(adapter.last_error)
        self.assertEqual(message["v"], 2)


if __name__ == "__main__":
    unittest.main()
