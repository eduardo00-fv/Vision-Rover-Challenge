#!/usr/bin/env python3
"""Registro de la telemetría de visión para las mediciones en la arena.

Se conecta al mismo TCP que los rovers (NDJSON, contrato v2) y guarda cada
rover y cubo con la hora de captura (`ts_ms`) y la de recepción. Después
resume velocidades, giros, latencia y la distancia de contacto rover-cubo.

  python3 registro_vision.py grabar --host 192.168.137.1 --salida pwm30.csv
  python3 registro_vision.py resumen pwm30.csv --rover 10
  python3 registro_vision.py contacto empuje.csv --rover 10 --color red

La latencia solo es válida si el registro corre en la misma máquina que la
visión (mismo reloj).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import statistics
import sys
import time
from pathlib import Path

CAMPOS = ["recv_ms", "ts_ms", "seq", "phase", "tipo", "clave", "col", "row", "theta", "age_ms", "cell_mm"]


def grabar(args: argparse.Namespace) -> None:
    fin = time.monotonic() + args.segundos if args.segundos else math.inf
    filas = 0
    with socket.create_connection((args.host, args.port), timeout=5) as sock, \
            open(args.salida, "w", newline="") as salida:
        sock.settimeout(1.0)
        escritor = csv.DictWriter(salida, fieldnames=CAMPOS)
        escritor.writeheader()
        pendiente = b""
        print(f"grabando {args.host}:{args.port} -> {args.salida} (Ctrl+C para terminar)")
        try:
            while time.monotonic() < fin:
                try:
                    bloque = sock.recv(65536)
                except socket.timeout:
                    continue
                if not bloque:
                    print("la visión cerró la conexión", file=sys.stderr)
                    break
                recv_ms = time.time() * 1000
                pendiente += bloque
                *lineas, pendiente = pendiente.split(b"\n")
                for linea in lineas:
                    try:
                        msg = json.loads(linea)
                    except json.JSONDecodeError:
                        continue
                    base = {"recv_ms": f"{recv_ms:.1f}", "ts_ms": msg.get("ts_ms"), "seq": msg.get("seq"),
                            "phase": msg.get("phase"), "cell_mm": msg.get("grid", {}).get("cell_mm", 20.0)}
                    for r in msg.get("rovers", []):
                        escritor.writerow({**base, "tipo": "rover", "clave": r["id"], "col": r["col"],
                                           "row": r["row"], "theta": r.get("theta", ""), "age_ms": r.get("age_ms", 0)})
                        filas += 1
                    for c in msg.get("cubes", []):
                        escritor.writerow({**base, "tipo": "cubo", "clave": c["color"], "col": c["col"],
                                           "row": c["row"], "theta": "", "age_ms": c.get("age_ms", 0)})
                        filas += 1
        except KeyboardInterrupt:
            pass
    print(f"{filas} filas")


def leer(archivo: Path, tipo: str, clave: str) -> list[dict]:
    """Muestras frescas (age_ms 0) de un rover o cubo, sin repetir captura."""
    vistas, muestras = set(), []
    with open(archivo, newline="") as f:
        for fila in csv.DictReader(f):
            if fila["tipo"] != tipo or fila["clave"] != clave or float(fila["age_ms"] or 0) > 0:
                continue
            if fila["ts_ms"] in vistas:
                continue
            vistas.add(fila["ts_ms"])
            cell = float(fila["cell_mm"])
            muestras.append({"t": float(fila["ts_ms"]), "recv": float(fila["recv_ms"]),
                             "x": float(fila["col"]) * cell, "y": float(fila["row"]) * cell,
                             "th": float(fila["theta"]) if fila["theta"] else None})
    return sorted(muestras, key=lambda m: m["t"])


def envolver(grados: float) -> float:
    return (grados + 180) % 360 - 180


def resumen(args: argparse.Namespace) -> None:
    m = leer(args.archivo, "rover", str(args.rover))
    if len(m) < 6:
        sys.exit("muy pocas muestras de ese rover")
    latencias = [s["recv"] - s["t"] for s in m]
    print(f"{len(m)} capturas en {(m[-1]['t'] - m[0]['t']) / 1000:.1f} s")
    print(f"latencia captura→recepción: mediana {statistics.median(latencias):.0f} ms, "
          f"p95 {sorted(latencias)[int(0.95 * (len(latencias) - 1))]:.0f} ms (mismo reloj)")
    # Tramos en movimiento: velocidad > umbral medida en ventanas de 200 ms
    # (entre capturas consecutivas el ruido de pose parece movimiento).
    tramos, actual, j = [], [], 0
    for i in range(1, len(m)):
        while m[i]["t"] - m[j + 1]["t"] >= 200:
            j += 1
        a, b = m[j], m[i]
        dt = (b["t"] - a["t"]) / 1000
        if dt < 0.2:
            continue
        v = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt
        w = envolver(b["th"] - a["th"]) / dt if a["th"] is not None and b["th"] is not None else 0.0
        if v > args.umbral_mm_s or abs(w) > args.umbral_dps:
            actual.append((m[i - 1], b))
        elif actual:
            tramos.append(actual)
            actual = []
    if actual:
        tramos.append(actual)
    tramos = [t for t in tramos if t[-1][1]["t"] - t[0][0]["t"] >= 300]
    print(f"\n{'tramo':>5} {'dur s':>6} {'dist mm':>8} {'v media mm/s':>13} {'giro °/s':>9} {'giro total °':>12}")
    for k, t in enumerate(tramos, 1):
        inicio, fin = t[0][0], t[-1][1]
        dur = (fin["t"] - inicio["t"]) / 1000
        dist = math.hypot(fin["x"] - inicio["x"], fin["y"] - inicio["y"])
        giro = sum(envolver(b["th"] - a["th"]) for a, b in t if a["th"] is not None and b["th"] is not None)
        print(f"{k:>5} {dur:>6.2f} {dist:>8.0f} {dist / dur if dur else 0:>13.0f} "
              f"{giro / dur if dur else 0:>9.1f} {giro:>12.1f}")


def contacto(args: argparse.Namespace) -> None:
    rover = leer(args.archivo, "rover", str(args.rover))
    cubo = {s["t"]: s for s in leer(args.archivo, "cubo", args.color)}
    distancias = []
    for r in rover:
        c = cubo.get(r["t"])
        if c is None or r["th"] is None:
            continue
        dx, dy = c["x"] - r["x"], c["y"] - r["y"]
        # Marco del rover (theta CCW con y hacia abajo): x adelante, y lateral.
        adelante = dx * math.cos(math.radians(r["th"])) - dy * math.sin(math.radians(r["th"]))
        lateral = dx * math.sin(math.radians(r["th"])) + dy * math.cos(math.radians(r["th"]))
        if 60 < adelante < 160 and abs(lateral) < 40:
            distancias.append(math.hypot(dx, dy))
    if not distancias:
        sys.exit("no hay capturas con el cubo frente a las paletas")
    print(f"{len(distancias)} capturas con el cubo al frente")
    print(f"centro rover → centro cubo: mediana {statistics.median(distancias):.1f} mm, "
          f"mín {min(distancias):.1f}, máx {max(distancias):.1f}")
    print("Con el cubo apoyado en las paletas, la mediana es NAV_PUSH_CONTACT_OFFSET_MM (hoy 107).")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="orden", required=True)
    g = sub.add_parser("grabar", help="guardar la telemetría en CSV")
    g.add_argument("--host", default="192.168.137.1")
    g.add_argument("--port", type=int, default=2026)
    g.add_argument("--salida", type=Path, required=True)
    g.add_argument("--segundos", type=float, default=0, help="0 = hasta Ctrl+C")
    r = sub.add_parser("resumen", help="velocidad, giro y latencia de un rover")
    r.add_argument("archivo", type=Path)
    r.add_argument("--rover", type=int, required=True)
    r.add_argument("--umbral-mm-s", type=float, default=15)
    r.add_argument("--umbral-dps", type=float, default=15)
    c = sub.add_parser("contacto", help="distancia rover-cubo con el cubo en las paletas")
    c.add_argument("archivo", type=Path)
    c.add_argument("--rover", type=int, required=True)
    c.add_argument("--color", required=True)
    args = p.parse_args()
    {"grabar": grabar, "resumen": resumen, "contacto": contacto}[args.orden](args)


if __name__ == "__main__":
    main()
