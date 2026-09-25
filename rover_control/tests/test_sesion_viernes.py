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
