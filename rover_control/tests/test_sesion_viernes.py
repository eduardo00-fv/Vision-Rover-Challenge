import importlib.util
import math
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "diagnostico" / "sesion_viernes.py"
spec = importlib.util.spec_from_file_location("sesion_viernes", SCRIPT)
sv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sv)

CONFIG = """constexpr char VISION_HOST[] = "192.168.0.100";
constexpr uint8_t ROVER_ID = 11;
#define VRC_PHYSICAL_ROVER 2
#define VRC_NETWORK_ONLY 1
#define VRC_ENABLE_MOTOR_BENCH 0
#define ARENA_SETTLE_MS 500U
"""


def test_config_banco_habilita_banco_y_quita_ritmo():
    t = sv.reescribir_config(CONFIG, "banco", "10.0.0.5", 10, None)
    assert 'VISION_HOST[] = "10.0.0.5"' in t and "ROVER_ID = 10;" in t
    assert "#define VRC_PHYSICAL_ROVER 1" in t
    assert "#define VRC_NETWORK_ONLY 0" in t and "#define VRC_ENABLE_MOTOR_BENCH 1" in t
    assert "ARENA_SETTLE_MS" not in t


def test_config_autonomo_con_ritmo():
    t = sv.reescribir_config(CONFIG, "autonomo", "10.0.0.5", 10, (500, 350))
    assert "#define VRC_NETWORK_ONLY 0" in t and "#define VRC_ENABLE_MOTOR_BENCH 0" in t
    assert t.count("ARENA_SETTLE_MS 500U") == 1 and "ARENA_MAX_DRIVE_MS 350U" in t


def test_marco_del_rover_y_espacio_libre():
    rover = {"x": 100.0, "y": 500.0, "th": 90.0}  # mirando hacia arriba (y decrece)
    adelante, lateral = sv.relativo(rover, (100.0, 393.0))
    assert math.isclose(adelante, 107, abs_tol=1e-6) and abs(lateral) < 1e-6
    assert math.isclose(sv.libre(rover, 1000, 1000, 1), 500)
    assert math.isclose(sv.libre(rover, 1000, 1000, -1), 500)


def test_cubo_en_su_zona_como_el_contrato():
    grid = {"cols": 43, "rows": 43, "cell_mm": 20.0}
    size = {"length": 10.0, "depth": 7.5}
    depot = {"color": "green", "col": 21.5, "row": 3.75}
    assert sv.cubo_en_su_zona({"col": 21.48, "row": 3.762}, depot, size, grid, 3.0)[0]
    assert not sv.cubo_en_su_zona({"col": 21.5, "row": 5.5}, depot, size, grid, 3.0)[0]


def test_ajuste_ir_ubica_sensores_y_detecta_trabado():
    import numpy as np
    sys_path = str(SCRIPT.parent)
    import sys
    if sys_path not in sys.path:
        sys.path.insert(0, sys_path)
    import analisis_ir as A
    rng = np.random.default_rng(3)
    reales = [(55, -30), (55, 30), (-55, -30)]
    poses, x, y, th = [], 430.0, 430.0, 20.0
    for n, dv, dth in [(30, 3, 0), (30, -3, 0), (8, 0, 5)] * 3 + [(40, 0, 4), (40, 0, -4)]:
        for _ in range(n):
            x += dv * np.cos(np.radians(th)); y -= dv * np.sin(np.radians(th)); th += dth
            poses.append((x, y, th))
    P = np.array(poses)
    B = np.ones((len(P), 4), int)  # S4 trabado
    for i, (f, l) in enumerate(reales):
        B[:, i] = A.color(*A.mundo(P[:, 0], P[:, 1], P[:, 2], f, l), 10, 10)
    ruido = P + np.c_[rng.normal(0, 2, len(P)), rng.normal(0, 2, len(P)), rng.normal(0, 1, len(P))]
    ajustes, _ = A.ajustar(ruido, B, rango=80)
    for a, (f, l) in zip(ajustes, reales):
        assert abs(a.f - f) <= 3 and abs(a.l - l) <= 3 and a.acierto > 0.8
    assert ajustes[3].trabado


def test_line_guard_python_detecta_borde_y_no_el_ruido():
    import sys
    sys.path.insert(0, str(SCRIPT.parent))
    import analisis_ir as A
    g = A.LineGuard(60, 60, 0b0001, 0b0010)
    g.sample(0)
    for k in range(200):
        g.pose(300 + (k % 3) - 1, 300, 90 + (k % 2))
    assert g.fuera() == 0
    for k in range(1, 40):
        g.pose(300 + 3 * k, 300, 0)
    assert g.fuera() & 0b0001
