"""Análisis de los IR sobre el tablero de ajedrez (prueba `ir` de sesion_viernes.py).

Los IR son digitales y la cancha es un tablero de ajedrez de celdas de 20 mm:
leer negro no significa borde. Con la pose de la cámara y el patrón conocido del
tablero se puede, en cambio:

- ubicar cada sensor en el chasis (adelante/atrás, izquierda/derecha), buscando
  el desplazamiento que mejor explica sus lecturas mientras el rover se mueve;
- saber si un sensor está trabado (no cambia nunca);
- medir la racha más larga de un color SOBRE la cancha, que fija el umbral de
  borde de `line_guard.h`;
- reproducir `LineGuard` con los datos grabados para ver cuándo habría
  detectado el borde.

Convenciones: x = col·cell (derecha), y = row·cell (abajo), theta en grados
antihorario. Posición de un sensor en el chasis: `f` hacia adelante y `l`
hacia la derecha, en mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CELDA_MM = 20.0


def mundo(x, y, th_deg, f, l):
    """Posición en la cancha de un punto (f adelante, l a la derecha) del rover."""
    th = np.radians(th_deg)
    return x + f * np.cos(th) + l * np.sin(th), y - f * np.sin(th) + l * np.cos(th)


def color(x, y, px, py):
    """Paridad de la casilla (0/1) con el tablero desfasado (px, py) mm."""
    return (np.floor((x - px) / CELDA_MM) + np.floor((y - py) / CELDA_MM)).astype(np.int64) & 1


@dataclass
class Ajuste:
    f: float
    l: float
    invertido: bool
    acierto: float     # fracción de muestras explicadas
    cambios: int       # cuántas veces cambió la lectura
    trabado: bool


def ajustar(poses: np.ndarray, bits: np.ndarray, rango: float = 120, paso: float = 4) -> tuple[list[Ajuste], tuple[float, float]]:
    """Posición de los 4 sensores. poses: (N, 3) x, y, th; bits: (N, 4) lecturas 0/1.

    El desfase del tablero (px, py) es común a los cuatro; se busca junto con
    ellos. Primero una grilla gruesa, después se refina alrededor del mejor.
    """
    x, y, th = poses[:, 0:1], poses[:, 1:2], poses[:, 2:3]
    cambios = [int(np.count_nonzero(np.diff(bits[:, i]))) for i in range(4)]

    def mejor(px, py, fs, ls, i):
        F, L = np.meshgrid(fs, ls, indexing="ij")
        F, L = F.ravel()[None, :], L.ravel()[None, :]
        wx, wy = mundo(x, y, th, F, L)
        c = color(wx, wy, px, py)
        acierto = (c == bits[:, i:i + 1]).mean(axis=0)
        k = int(np.argmax(np.maximum(acierto, 1 - acierto)))
        a = acierto[k]
        return max(a, 1 - a), float(F[0, k]), float(L[0, k]), a < 0.5

    grueso = np.arange(-rango, rango + 0.1, paso)
    mejor_total, mejor_fase = -1.0, (0.0, 0.0)
    for px in np.arange(0, CELDA_MM, paso):
        for py in np.arange(0, CELDA_MM, paso):
            total = sum(mejor(px, py, grueso, grueso, i)[0] for i in range(4) if cambios[i] >= 3)
            if total > mejor_total:
                mejor_total, mejor_fase = total, (float(px), float(py))
    px0, py0 = mejor_fase
    resultados, mejor_total = None, -1.0
    for px in np.arange(px0 - paso, px0 + paso + 0.1, 1.0):
        for py in np.arange(py0 - paso, py0 + paso + 0.1, 1.0):
            parcial = []
            for i in range(4):
                if cambios[i] < 3:
                    parcial.append(Ajuste(0, 0, False, 0, cambios[i], True))
                    continue
                a, f, l, inv = mejor(px, py, grueso, grueso, i)
                fino = np.arange(-paso, paso + 0.1, 1.0)
                a, f, l, inv = mejor(px, py, f + fino, l + fino, i)
                parcial.append(Ajuste(f, l, inv, a, cambios[i], False))
            total = sum(p.acierto for p in parcial)
            if total > mejor_total:
                mejor_total, resultados, mejor_fase = total, parcial, (float(px % CELDA_MM), float(py % CELDA_MM))
    return resultados, mejor_fase


def racha_maxima(poses: np.ndarray, bits: np.ndarray, i: int, a: Ajuste) -> float:
    """Distancia más larga (mm, en línea recta) que recorrió el sensor sin cambiar de color."""
    wx, wy = mundo(poses[:, 0], poses[:, 1], poses[:, 2], a.f, a.l)
    ancla, maxima = 0, 0.0
    for k in range(1, len(bits)):
        if bits[k, i] != bits[k - 1, i]:
            ancla = k
        maxima = max(maxima, math.hypot(wx[k] - wx[ancla], wy[k] - wy[ancla]))
    return maxima


class LineGuard:
    """Copia de rover_control/line_guard.h, para reproducir la decisión con datos grabados."""
    DIAG_BAND, SLOPE_BAND, DIAG_FACTOR, MAX_JUMP_MM, MAX_JUMP_DEG = 12, 6, 2, 80, 60

    def __init__(self, edge_mm=60.0, radius_mm=60.0, front=0, rear=0):
        self.edge, self.radius, self.front, self.rear = edge_mm, radius_mm, front, rear
        self.mask = None
        self.pose_ = None
        self.anclas = [None] * 4

    @staticmethod
    def _ang(a, b):
        d = abs(a - b) % 360
        return 360 - d if d > 180 else d

    def sample(self, mask: int) -> None:
        cambio = 0x0F if self.mask is None else mask ^ self.mask
        for i in range(4):
            if cambio & (1 << i):
                self.anclas[i] = self.pose_
        self.mask = mask

    def pose(self, x, y, th) -> None:
        salto = self.pose_ is not None and (math.hypot(x - self.pose_[0], y - self.pose_[1]) > self.MAX_JUMP_MM
                                            or self._ang(th, self.pose_[2]) > self.MAX_JUMP_DEG)
        self.pose_ = (x, y, th)
        for i in range(4):
            if salto or self.anclas[i] is None:
                self.anclas[i] = self.pose_

    def distancia(self, i) -> float:
        a, p = self.anclas[i], self.pose_
        if a is None or p is None:
            return 0.0
        return math.hypot(p[0] - a[0], p[1] - a[1]) + self._ang(p[2], a[2]) * math.pi / 180 * self.radius

    def umbral(self) -> float:
        m = abs(self.pose_[2]) % 90 if self.pose_ else 0
        diag = abs(m - 45) < self.DIAG_BAND or abs(m - 26.57) < self.SLOPE_BAND or abs(m - 63.43) < self.SLOPE_BAND
        return self.edge * (self.DIAG_FACTOR if diag else 1)

    def fuera(self) -> int:
        u = self.umbral()
        return sum(1 << i for i in range(4) if self.distancia(i) > u)
