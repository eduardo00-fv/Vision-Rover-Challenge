"""Veredicto reproducible de una corrida Webots.

El tiempo solo es comparable entre corridas que terminaron la misión sin una
falta de seguridad. La fidelidad física se decide aparte: mientras no esté
calibrada, una corrida correcta es evidencia útil pero no un ``PASS``.
"""


def evaluate(expected, deliveries, safety_failures, elapsed_ms, physics_calibrated=False,
             official_completion=False):
    """Devuelve el resultado, la causa y los datos para ordenar corridas."""
    required = expected["required_deliveries"]
    missing = [color for color in required if not deliveries.get(color, {}).get("delivered", False)]
    if safety_failures:
        return {
            "status": "FAIL",
            "reason": "; ".join(sorted(safety_failures)),
            "completion_ms": None,
            "missing_deliveries": missing,
        }
    if missing:
        return {
            "status": "FAIL",
            "reason": "Entregas requeridas incompletas: {}".format(", ".join(missing)),
            "completion_ms": None,
            "missing_deliveries": missing,
        }
    if not official_completion:
        return {
            "status": "FAIL",
            "reason": "El árbitro no confirmó el reto cumplido durante la ronda.",
            "completion_ms": None,
            "missing_deliveries": [],
        }
    if not physics_calibrated:
        return {
            "status": "INCONCLUSIVE",
            "reason": "Misión completa y segura, pero falta calibración física para emitir PASS.",
            "completion_ms": elapsed_ms,
            "missing_deliveries": [],
        }
    return {
        "status": "PASS",
        "reason": "Misión completa y segura.",
        "completion_ms": elapsed_ms,
        "missing_deliveries": [],
    }
