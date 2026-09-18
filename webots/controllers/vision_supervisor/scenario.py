"""Carga el escenario Webots sin depender de la API gráfica.

El archivo de escenario apunta a la configuración contractual que describe la
cancha. Así la simulación no mantiene una segunda copia de posiciones, tamaños,
tiempos o patologías.
"""

import json
from pathlib import Path


def load_scenario(path):
    """Devuelve ``(metadata, config)`` y rechaza referencias incompletas."""
    scenario_path = Path(path).resolve()
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    if not isinstance(scenario.get("id"), str) or not scenario["id"]:
        raise ValueError("scenario: falta un id no vacío")
    config_ref = scenario.get("contract_config")
    if not isinstance(config_ref, str) or not config_ref:
        raise ValueError("scenario: falta contract_config")
    config_path = (scenario_path.parent / config_ref).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    required = ("grid", "rovers", "cubes", "depots", "ronda", "ruido", "patologias")
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError("scenario: configuración contractual incompleta: {}".format(", ".join(missing)))
    oracle = scenario.get("oracle", {})
    for key in ("rover_length_mm", "rover_width_mm"):
        if not isinstance(oracle.get(key), (int, float)) or oracle[key] <= 0:
            raise ValueError("scenario: oracle.{} debe ser un número > 0".format(key))
    expected = scenario.get("expected", {})
    required_deliveries = expected.get("required_deliveries")
    colors = {cube["color"] for cube in config["cubes"]}
    if (not isinstance(required_deliveries, list) or not required_deliveries or
            any(not isinstance(color, str) for color in required_deliveries) or
            len(set(required_deliveries)) != len(required_deliveries) or
            not set(required_deliveries).issubset(colors)):
        raise ValueError("scenario: expected.required_deliveries debe listar cubos configurados, sin repetir")
    if not isinstance(scenario.get("physics_calibrated"), bool):
        raise ValueError("scenario: physics_calibrated debe ser booleano")
    return scenario, config
