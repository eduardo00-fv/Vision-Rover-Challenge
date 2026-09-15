"""Reglas puras del estado publicado por el supervisor Webots.

No importa Webots para que se pueda probar el reloj contractual sin abrir una
ventana ni ejecutar una simulación.
"""


def round_clock(phase, elapsed_ms, preparation_ms, duration_ms):
    """Construye el reloj v2 para una fase y tiempo transcurrido.

    ``elapsed_ms`` es el tiempo de la fase que cuenta. Para ``FINISHED`` el
    supervisor entrega el tiempo que transcurrió antes de terminar, preservando
    correctamente el margen que quedaba de la ronda.
    """
    if phase == "IDLE":
        return {"elapsed_ms": 0, "remaining_ms": 0, "total_ms": 0}
    total_ms = preparation_ms if phase == "READY" else duration_ms
    elapsed_ms = min(total_ms, max(0, int(elapsed_ms)))
    return {
        "elapsed_ms": elapsed_ms,
        "remaining_ms": total_ms - elapsed_ms,
        "total_ms": total_ms,
    }
