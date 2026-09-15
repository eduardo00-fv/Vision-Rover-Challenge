"""Adaptador entre una imagen Webots y el pipeline oficial de visión.

La única entrada dinámica es una imagen BGR. No acepta poses, cubos ni estado
del Supervisor; por construcción no puede filtrar verdad física hacia la
telemetría.
"""

from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
VISION_SYSTEM = ROOT / "vision-system"
if str(VISION_SYSTEM) not in sys.path:
    sys.path.insert(0, str(VISION_SYSTEM))

from vision.configuracion import cargar_config  # noqa: E402
from vision.geometry.coordenadas import AnclajeCancha  # noqa: E402
from vision.mundo import EstadoMundo, RelojRonda, a_mensaje  # noqa: E402
from vision.sistema import procesar  # noqa: E402
from vision.sources.fuente import Cuadro  # noqa: E402
from vision.tracking.admision import RegistroAdmision  # noqa: E402
from vision.tracking.seguimiento import Seguidor  # noqa: E402


def webots_bgra_to_bgr(raw, width, height):
    """Convierte el buffer BGRA de `Camera.getImage()` a la entrada OpenCV."""
    pixels = np.frombuffer(raw, dtype=np.uint8)
    expected = width * height * 4
    if pixels.size != expected:
        raise ValueError("imagen Webots: {} bytes, se esperaban {}".format(pixels.size, expected))
    return pixels.reshape((height, width, 4))[:, :, :3].copy()


class VisionAdapter:
    """Estado productor del mismo pipeline que corre con cámara física."""

    def __init__(self, config_path, width, height, field_of_view):
        self.cfg = cargar_config(str(config_path))
        # Cámara pinhole cuadrada: Webots entrega una imagen sin distorsión. La
        # pose se recupera a partir de los ArUco de esquina, igual que en la base.
        focal = (width / 2.0) / np.tan(field_of_view / 2.0)
        self.matrix = np.array(((focal, 0.0, width / 2.0),
                                (0.0, focal, height / 2.0),
                                (0.0, 0.0, 1.0)), dtype=np.float64)
        self.seguidor = Seguidor(self.cfg)
        self.anclaje = AnclajeCancha(self.cfg)
        self.admision = RegistroAdmision(self.cfg)
        self.descartados, self.duplicados = set(), []
        self.rechazos, self.demorados = [], []
        self.indice = 0
        self.last_state = None
        self.last_error = None

    def observe(self, image_bgr, ts_ms, phase, clock, seq):
        """Procesa un cuadro y produce exactamente un mensaje v2.

        Ante una falla, conserva la última observación buena y su timestamp,
        igual que el publicador real. En frío publica una foto vacía: nunca usa
        la verdad del mundo para completar lo que no detectó.
        """
        self.indice += 1
        round_clock = RelojRonda(transcurrido_ms=clock["elapsed_ms"],
                                  restante_ms=clock["remaining_ms"],
                                  total_ms=clock["total_ms"])
        frame = Cuadro(imagen=image_bgr, ts_ms=ts_ms, indice=self.indice)
        try:
            _, state = procesar(frame, self.cfg, self.matrix, phase, round_clock,
                                 self.seguidor, self.anclaje, self.descartados,
                                 self.duplicados, self.rechazos, self.admision,
                                 self.demorados)
            self.last_state, self.last_error = state, None
        except Exception as exc:  # misma política falla-abierto de vision.sistema
            self.last_error = "{}: {}".format(type(exc).__name__, exc)
        if self.last_state is None:
            self.last_state = EstadoMundo(ts_ms=ts_ms, fase="IDLE")
        message = a_mensaje(self.last_state, self.cfg, seq).a_dict()
        return message
