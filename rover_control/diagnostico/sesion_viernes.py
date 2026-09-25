#!/usr/bin/env python3
"""Asistente de la sesión en la arena: prepara el firmware y corre la hoja de mediciones.

Hace solo lo que no necesita manos: detecta la IP de la visión y el puerto USB,
reescribe `config.h`, compila y carga, manda las órdenes de banco, graba la
telemetría y calcula cada tabla de `HOJA_MEDICIONES_VIERNES.md`. Solo se detiene
para pedir algo físico (colocar el cubo, reubicar el rover, apretar `r` en la
visión). Ctrl+C manda `!` al rover antes de salir.

  python3 sesion_viernes.py                      # todo, en orden, reanudable
  python3 sesion_viernes.py banco                # solo pruebas 1-6
  python3 sesion_viernes.py ir                   # solo IR: posición, polaridad y borde
  python3 sesion_viernes.py ritmo                # solo prueba 7
  python3 sesion_viernes.py preparar autonomo    # solo config + compilar + cargar
  python3 sesion_viernes.py informe              # rehace RESULTADOS.md

Cada sesión vive en `diagnostico/sesiones/<fecha>/`: CSV de telemetría por
prueba, log serie, `estado.json` (lo ya medido; al reanudar se salta) y
`RESULTADOS.md` con las tablas llenas y los valores sugeridos.
La visión debe correr en esta misma máquina (latencia con el mismo reloj).
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import json
import math
import re
import socket
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

AQUI = Path(__file__).resolve().parent
SKETCH = AQUI.parent
CONFIG = SKETCH / "config.h"
ARDUINO_CLI = "/opt/arduino-ide/resources/app/lib/backend/resources/arduino-cli"
FQBN = "esp32:esp32:esp32"
PUERTO_VISION = 2026
sys.path.insert(0, str(AQUI))
from registro_vision import CAMPOS  # noqa: E402  mismo CSV: sirve `registro_vision.py resumen`

# Filas de la hoja. Los signos y duraciones son los de HOJA_MEDICIONES_VIERNES.md.
PWM_RECTA = [(18, 20), (25, 27), (30, 33), (35, 35)]
PWM_EMPUJE = [(18, 20), (25, 27), (30, 33)]
PWM_CURVA = [(25, 22), (25, 18), (30, 20)]
DUR_MS = 1500
GIRO_MS = 1000
REPOSO_MS = 800        # espera tras cada orden para medir el residual
LIBRE_MM = 260         # recorrido libre mínimo hacia donde va a moverse
RITMOS = {"500/350": (500, 350), "150/1000": (150, 1000)}


class Abortar(Exception):
    pass


def pedir(texto: str) -> None:
    input(f"\n>>> {texto}\n    [Enter para seguir] ")


# ---------------------------------------------------------------- geometría

def envolver(grados: float) -> float:
    return (grados + 180) % 360 - 180


def relativo(rover: dict, punto: tuple[float, float]) -> tuple[float, float]:
    """(adelante, lateral) de un punto en mm, en el marco del rover (theta CCW, y hacia abajo)."""
    dx, dy = punto[0] - rover["x"], punto[1] - rover["y"]
    th = math.radians(rover["th"])
    return dx * math.cos(th) - dy * math.sin(th), dx * math.sin(th) + dy * math.cos(th)


def libre(rover: dict, ancho: float, alto: float, sentido: int) -> float:
    """Distancia al borde de la cancha avanzando (+1) o retrocediendo (-1)."""
    th = math.radians(rover["th"])
    ux, uy = sentido * math.cos(th), -sentido * math.sin(th)
    t = math.inf
    for p, u, lim in ((rover["x"], ux, ancho), (rover["y"], uy, alto)):
        if u > 1e-9:
            t = min(t, (lim - p) / u)
        elif u < -1e-9:
            t = min(t, -p / u)
    return t


def cubo_en_su_zona(cubo, depot, depot_size, grid, cube_side):
    """Copia literal de CONTRATO.md: (adentro, cuánto falta en celdas)."""
    distancias = {"arriba": depot["row"], "abajo": grid["rows"] - depot["row"],
                  "izquierda": depot["col"], "derecha": grid["cols"] - depot["col"]}
    lado = min(distancias, key=lambda l: distancias[l])
    if lado in ("arriba", "abajo"):
        semi_col, semi_row = depot_size["length"] / 2, depot_size["depth"] / 2
    else:
        semi_col, semi_row = depot_size["depth"] / 2, depot_size["length"] / 2
    margen = cube_side * math.sqrt(2) / 2
    exceso_col = max(0.0, abs(cubo["col"] - depot["col"]) - (semi_col - margen))
    exceso_row = max(0.0, abs(cubo["row"] - depot["row"]) - (semi_row - margen))
    falta = math.hypot(exceso_col, exceso_row)
    return falta == 0.0, falta


def pendiente(xs: list[float], ys: list[float]) -> float:
    if len(xs) < 3:
        return math.nan
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else math.nan


# ---------------------------------------------------------------- telemetría

class Telemetria:
    """Lee la visión en un hilo: guarda el último mensaje y cada captura nueva."""

    def __init__(self, host: str, port: int = PUERTO_VISION):
        self.host, self.port = host, port
        self.ultimo: dict | None = None
        self.capturas: list[dict] = []   # {t, recv, rovers:{id:pose}, cubos:{color:pose}}
        self._csv = None
        self._lock = threading.Lock()
        self._fin = threading.Event()
        self._hilo = threading.Thread(target=self._correr, daemon=True)

    def iniciar(self, espera_s: float = 5) -> None:
        self._hilo.start()
        limite = time.monotonic() + espera_s
        while self.ultimo is None and time.monotonic() < limite:
            time.sleep(0.05)
        if self.ultimo is None:
            raise Abortar(f"no llega telemetría de {self.host}:{self.port}: ¿está corriendo la visión?")

    def grabar(self, archivo: Path | None) -> None:
        with self._lock:
            if self._csv:
                self._csv[0].close()
            self._csv = None
            if archivo:
                f = open(archivo, "w", newline="")
                w = csv.DictWriter(f, fieldnames=CAMPOS)
                w.writeheader()
                self._csv = (f, w)

    def cerrar(self) -> None:
        self._fin.set()
        self.grabar(None)

    def _correr(self) -> None:
        while not self._fin.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=3) as sock:
                    sock.settimeout(1.0)
                    pendiente_ = b""
                    while not self._fin.is_set():
                        try:
                            bloque = sock.recv(65536)
                        except socket.timeout:
                            continue
                        if not bloque:
                            break
                        recv = time.time() * 1000
                        pendiente_ += bloque
                        *lineas, pendiente_ = pendiente_.split(b"\n")
                        for linea in lineas:
                            try:
                                self._procesar(json.loads(linea), recv)
                            except (json.JSONDecodeError, KeyError, TypeError):
                                continue
            except OSError:
                time.sleep(0.5)

    def _procesar(self, msg: dict, recv: float) -> None:
        cell = msg["grid"]["cell_mm"]
        pose = lambda o: {"x": o["col"] * cell, "y": o["row"] * cell, "th": o.get("theta"),
                          "col": o["col"], "row": o["row"], "age": o.get("age_ms", 0)}
        rovers = {r["id"]: pose(r) for r in msg.get("rovers", [])}
        cubos = {c["color"]: pose(c) for c in msg.get("cubes", [])}
        with self._lock:
            nuevo = not self.capturas or msg["ts_ms"] != self.capturas[-1]["t"]
            self.ultimo = {**msg, "_recv": recv}
            if nuevo:
                self.capturas.append({"t": msg["ts_ms"], "recv": recv, "rovers": rovers, "cubos": cubos})
                if len(self.capturas) > 20000:
                    del self.capturas[:5000]
            if self._csv:
                base = {"recv_ms": f"{recv:.1f}", "ts_ms": msg["ts_ms"], "seq": msg.get("seq"),
                        "phase": msg.get("phase"), "cell_mm": cell}
                for r in msg.get("rovers", []):
                    self._csv[1].writerow({**base, "tipo": "rover", "clave": r["id"], "col": r["col"], "row": r["row"],
                                           "theta": r.get("theta", ""), "age_ms": r.get("age_ms", 0)})
                for c in msg.get("cubes", []):
                    self._csv[1].writerow({**base, "tipo": "cubo", "clave": c["color"], "col": c["col"],
                                           "row": c["row"], "theta": "", "age_ms": c.get("age_ms", 0)})

    def serie(self, rover: int, desde: float, hasta: float = math.inf) -> list[dict]:
        """Capturas frescas del rover entre dos instantes (ms época)."""
        with self._lock:
            caps = list(self.capturas)
        salida = []
        for c in caps:
            r = c["rovers"].get(rover)
            if r and r["age"] == 0 and r["th"] is not None and desde <= c["t"] <= hasta:
                salida.append({**r, "t": c["t"], "recv": c["recv"], "cubos": c["cubos"]})
        return salida

    def pose(self, rover: int, max_edad_ms: float = 400) -> dict | None:
        caps = self.serie(rover, time.time() * 1000 - max_edad_ms - 300)
        return caps[-1] if caps else None

    def arena_mm(self) -> tuple[float, float]:
        g = self.ultimo["grid"]
        return g["cols"] * g["cell_mm"], g["rows"] * g["cell_mm"]


# ---------------------------------------------------------------- serie

class Rover:
    def __init__(self, puerto: str | None, log: Path):
        self.puerto = puerto
        self.log = open(log, "a", buffering=1)
        self.lineas: list[str] = []
        self.t_lineas: list[float] = []  # hora de llegada (ms época) de cada línea
        self.ser = None
        if puerto:
            import serial
            self.ser = serial.Serial(puerto, 115200, timeout=0.05)
            self._hilo = threading.Thread(target=self._leer, daemon=True)
            self._hilo.start()

    def _leer(self) -> None:
        pendiente_ = b""
        while self.ser:
            try:
                pendiente_ += self.ser.read(512)
            except Exception:  # cable desconectado: se sigue sin serie
                self.ser = None
                self.log.write(f"{time.time():.3f} !! serie perdida\n")
                return
            *lineas, pendiente_ = pendiente_.split(b"\n")
            for l in lineas:
                texto = l.decode("utf-8", "replace").rstrip()
                self.t_lineas.append(time.time() * 1000)
                self.lineas.append(texto)
                self.log.write(f"{time.time():.3f} < {texto}\n")

    def enviar(self, linea: str) -> None:
        self.log.write(f"{time.time():.3f} > {linea}\n")
        if self.ser:
            self.ser.write(linea.encode() + b"\n")

    def esperar(self, patron: str, timeout_s: float, desde: int = 0) -> str | None:
        limite = time.monotonic() + timeout_s
        while time.monotonic() < limite:
            for l in self.lineas[desde:]:
                if re.search(patron, l):
                    return l
            time.sleep(0.02)
        return None

    def parar(self) -> None:
        if self.ser:
            try:
                self.ser.write(b"!\n")
            except Exception:
                pass

    def cerrar(self) -> None:
        self.parar()
        s, self.ser = self.ser, None
        if s:
            time.sleep(0.1)
            s.close()


class RoverWifi(Rover):
    """Consola del banco por Wi-Fi: el rover se conecta a esta laptop en BENCH_PORT."""

    def __init__(self, log: Path, puerto_tcp: int = 2027):
        super().__init__(None, log)
        self.sock = None
        self._srv = socket.create_server(("0.0.0.0", puerto_tcp), reuse_port=True)
        self._srv.settimeout(1.0)
        self._fin = threading.Event()
        threading.Thread(target=self._aceptar, daemon=True).start()

    def conectado(self) -> bool:
        return self.sock is not None

    def _aceptar(self) -> None:
        while not self._fin.is_set():
            try:
                conn, addr = self._srv.accept()
            except (socket.timeout, OSError):
                continue
            conn.settimeout(0.2)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.log.write(f"{time.time():.3f} !! rover conectado desde {addr[0]}\n")
            self.sock = conn
            pendiente_ = b""
            while not self._fin.is_set():
                try:
                    bloque = conn.recv(512)
                except socket.timeout:
                    continue
                except OSError:
                    bloque = b""
                if not bloque:
                    break
                pendiente_ += bloque
                *lineas, pendiente_ = pendiente_.split(b"\n")
                for l in lineas:
                    texto = l.decode("utf-8", "replace").rstrip()
                    self.t_lineas.append(time.time() * 1000)
                    self.lineas.append(texto)
                    self.log.write(f"{time.time():.3f} < {texto}\n")
            self.sock = None
            self.log.write(f"{time.time():.3f} !! rover desconectado\n")
            conn.close()

    def enviar(self, linea: str) -> None:
        self.log.write(f"{time.time():.3f} > {linea}\n")
        s = self.sock
        if s is None:
            raise Abortar("el rover no está conectado por Wi-Fi (¿encendido? ¿misma red?)")
        s.sendall(linea.encode() + b"\n")

    def parar(self) -> None:
        s = self.sock
        if s:
            try:
                s.sendall(b"!\n")
            except OSError:
                pass

    def cerrar(self) -> None:
        self.parar()
        self._fin.set()
        self._srv.close()


# ---------------------------------------------------------------- firmware

def ip_vision() -> str:
    """IPv4 de esta máquina para los rovers: la de Ethernet si hay, si no la de la ruta por defecto."""
    salida = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True).stdout
    ips = [(m.group(1), m.group(2)) for m in re.finditer(r"^\d+:\s+(\S+)\s+inet\s+([\d.]+)/", salida, re.M)]
    for iface, ip in ips:
        if iface.startswith(("en", "eth")):
            return ip
    ruta = subprocess.run(["ip", "-4", "route", "get", "1.1.1.1"], capture_output=True, text=True).stdout
    m = re.search(r"src ([\d.]+)", ruta)
    if m:
        return m.group(1)
    raise Abortar("no se encontró IP; usá --ip")


def puerto_usb(esperar: bool = True) -> str:
    while True:
        puertos = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if len(puertos) == 1:
            return puertos[0]
        if len(puertos) > 1:
            raise Abortar(f"hay varios puertos {puertos}: usá --puerto")
        if not esperar:
            raise Abortar("no hay rover por USB")
        pedir("Conectá el rover 1 por USB (cable de datos)")


def reescribir_config(texto: str, modo: str, ip: str, marcador: int, ritmo: tuple[int, int] | None) -> str:
    def poner(t: str, patron: str, nuevo: str) -> str:
        t, n = re.subn(patron, nuevo, t, flags=re.M)
        if n != 1:
            raise Abortar(f"config.h: no encontré exactamente una línea para {patron!r}")
        return t

    texto = poner(texto, r'^constexpr char VISION_HOST\[\] = ".*";', f'constexpr char VISION_HOST[] = "{ip}";')
    texto = poner(texto, r"^constexpr uint8_t ROVER_ID = \d+;", f"constexpr uint8_t ROVER_ID = {marcador};")
    texto = poner(texto, r"^#define VRC_PHYSICAL_ROVER \d+", "#define VRC_PHYSICAL_ROVER 1")
    texto = poner(texto, r"^#define VRC_NETWORK_ONLY \d+", f"#define VRC_NETWORK_ONLY {int(modo == 'red')}")
    texto = poner(texto, r"^#define VRC_ENABLE_MOTOR_BENCH \d+", f"#define VRC_ENABLE_MOTOR_BENCH {int(modo == 'banco')}")
    texto = re.sub(r"^#define ARENA_(SETTLE_MS|MAX_DRIVE_MS) .*\n", "", texto, flags=re.M)
    if ritmo:
        texto = texto.rstrip("\n") + f"\n#define ARENA_SETTLE_MS {ritmo[0]}U\n#define ARENA_MAX_DRIVE_MS {ritmo[1]}U\n"
    return texto


def preparar(modo: str, ip: str, marcador: int, puerto: str | None, ritmo=None, cargar=True) -> None:
    original = CONFIG.with_suffix(".h.orig")
    if not original.exists():
        original.write_text(CONFIG.read_text())
    CONFIG.write_text(reescribir_config(CONFIG.read_text(), modo, ip, marcador, ritmo))
    ssid = re.search(r'WIFI_SSID\[\] = "(.*)"', CONFIG.read_text()).group(1)
    print(f"[firmware] modo={modo} VISION_HOST={ip} ROVER_ID={marcador} ritmo={ritmo or 'por defecto'} "
          f"Wi-Fi='{ssid}'")
    build = Path.home() / ".cache" / f"vrc-build-{modo}"
    orden = [ARDUINO_CLI, "compile", "--fqbn", FQBN, "--build-path", str(build), str(SKETCH)]
    if cargar:
        orden += ["--upload", "-p", puerto or puerto_usb()]
    print("[firmware] compilando" + (" y cargando…" if cargar else "…"))
    r = subprocess.run(orden, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:], sep="\n")
        raise Abortar("falló compilar/cargar (arriba el error)")
    print("[firmware] listo")


# ---------------------------------------------------------------- sesión

class Sesion:
    def __init__(self, args):
        self.args = args
        self.dir = AQUI / "sesiones" / (args.sesion or dt.date.today().isoformat())
        self.dir.mkdir(parents=True, exist_ok=True)
        self.estado_path = self.dir / "estado.json"
        self.estado = json.loads(self.estado_path.read_text()) if self.estado_path.exists() else {}
        self.tel: Telemetria | None = None
        self.rover: Rover | None = None
        self.modo_cargado: str | None = None
        self.marcador = args.marcador

    def guardar(self, clave: str, valor) -> None:
        self.estado[clave] = valor
        self.estado_path.write_text(json.dumps(self.estado, indent=1, ensure_ascii=False))

    def hecho(self, clave: str) -> bool:
        return clave in self.estado and not self.args.repetir

    # -- infraestructura

    def conectar_vision(self) -> None:
        if self.tel:
            return
        self.tel = Telemetria(self.args.host)
        self.tel.iniciar()
        if self.marcador is None:
            self.marcador = self.detectar_marcador()
        print(f"[visión] conectado; fase={self.tel.ultimo['phase']} rover={self.marcador}")

    def detectar_marcador(self) -> int:
        while True:
            vistos = sorted({i for c in self.tel.capturas[-60:] for i, r in c["rovers"].items() if r["age"] == 0})
            if len(vistos) == 1:
                return vistos[0]
            if len(vistos) > 1:
                raise Abortar(f"la visión ve varios rovers {vistos}: pasá --marcador")
            pedir("La visión no ve ningún rover: poné el rover 1 (con su marcador) en la cancha")
            time.sleep(1)

    def firmware(self, modo: str, ritmo=None) -> None:
        """Carga por USB (si hace falta) y después se trabaja sin cable."""
        clave = f"{modo}:{ritmo}"
        if self.modo_cargado == clave:
            return
        self.conectar_vision()
        if not self.args.no_cargar:
            puerto = self.args.puerto or puerto_usb()
            preparar(modo, self.args.ip or ip_vision(), self.marcador, puerto, ritmo)
            pedir("Firmware cargado. Desconectá el USB (el rover sigue encendido con la batería) "
                  "y dejalo quieto unos segundos")
        self.modo_cargado = clave
        if modo != "banco":
            if self.rover:
                self.rover.cerrar()
            self.rover = Rover(None, self.dir / "serie.log")  # sin consola: `stop` en la visión lo frena
            return
        if not isinstance(self.rover, RoverWifi):
            self.rover = RoverWifi(self.dir / "serie.log")
        print("[banco] esperando que el rover se conecte por Wi-Fi…")
        limite = time.monotonic() + 45
        while not self.rover.conectado() and time.monotonic() < limite:
            time.sleep(0.2)
        if not self.rover.conectado():
            raise Abortar("el rover no se conectó por Wi-Fi en 45 s: revisá que esté encendido y en 'Atta-Bot'")
        print("[banco] rover conectado por Wi-Fi")

    def pose(self) -> dict:
        while True:
            p = self.tel.pose(self.marcador)
            if p:
                return p
            pedir(f"La visión no ve el marcador {self.marcador}: revisá que esté en la cancha y sin tapar")

    def asegurar_espacio(self, sentidos: tuple[int, ...], motivo: str) -> dict:
        ancho, alto = self.tel.arena_mm()
        while True:
            p = self.pose()
            falta = [s for s in sentidos if libre(p, ancho, alto, s) < LIBRE_MM]
            if not falta:
                return p
            hacia = "adelante" if 1 in falta else "atrás"
            pedir(f"Poco espacio {hacia} ({motivo}). Llevá el rover al centro de la cancha "
                  f"con ≥{LIBRE_MM / 10:.0f} cm libres {hacia}")

    def orden(self, izq: int, der: int, ms: int) -> tuple[float, float]:
        """Manda una orden de banco; devuelve (inicio, fin) en ms época, ya asentado."""
        desde = len(self.rover.lineas)
        self.rover.enviar("ARM")
        if not self.rover.esperar(r"\[banco\] armado", 2, desde):
            raise Abortar("el banco no se armó")
        desde = len(self.rover.lineas)
        t0 = time.time() * 1000
        self.rover.enviar(f"MOTOR {izq} {der} {ms}")
        if not self.rover.esperar(r"\[banco\] izq=", 2, desde):
            raise Abortar(f"el banco rechazó MOTOR {izq} {der} {ms}: {self.rover.lineas[-1:]}")
        if not self.rover.esperar(r"\[banco\] fin de orden", ms / 1000 + 3, desde):
            self.rover.parar()
            raise Abortar("el banco no terminó la orden")
        time.sleep(max(0.0, (t0 + ms) / 1000 - time.time()) + REPOSO_MS / 1000)
        return t0, t0 + ms

    def tramo(self, t0: float, t1: float) -> dict:
        """Mide un tramo del banco con la telemetría de alrededor."""
        antes = self.tel.serie(self.marcador, t0 - 1500, t0)
        durante = self.tel.serie(self.marcador, t0, t1 + REPOSO_MS)
        if not antes or len(durante) < 5:
            return {"ok": False}
        a, b = antes[-1], durante[-1]
        dur = (t1 - t0) / 1000
        th = [a["th"]]
        for s in durante:
            th.append(th[-1] + envolver(s["th"] - th[-1]))
        al_cortar = [x for x, s in zip(th[1:], durante) if s["t"] <= t1 + 60]
        firme = [(s["t"], s) for s in durante if t0 + 300 <= s["t"] <= t1]
        rumbo = math.radians(a["th"])
        avance = [((s["x"] - a["x"]) * math.cos(rumbo) - (s["y"] - a["y"]) * math.sin(rumbo)) for _, s in firme]
        giro_firme = [x for x, s in zip(th[1:], durante) if t0 + 200 <= s["t"] <= t1]
        return {
            "ok": True,
            "dist_mm": math.hypot(b["x"] - a["x"], b["y"] - a["y"]),
            "v_media_mm_s": math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dur,
            "v_firme_mm_s": abs(pendiente([t / 1000 for t, _ in firme], avance)),
            "giro_total_deg": th[-1] - th[0],
            "giro_dps": pendiente([s["t"] / 1000 for s in durante if t0 + 200 <= s["t"] <= t1], giro_firme),
            "residual_deg": th[-1] - (al_cortar[-1] if al_cortar else th[0]),
            "capturas": len(durante),
        }

    def cubo_al_frente(self, p: dict) -> str | None:
        for color, c in p["cubos"].items():
            adelante, lateral = relativo(p, (c["x"], c["y"]))
            if 60 < adelante < 180 and abs(lateral) < 50:
                return color
        return None

    def esperar_cubo(self, texto: str) -> tuple[dict, str]:
        while True:
            pedir(texto)
            time.sleep(0.5)  # que la pose sea de después de soltar el cubo
            p = self.pose()
            color = self.cubo_al_frente(p)
            if color:
                return p, color
            print("    No veo un cubo apoyado al frente de las paletas; probemos otra vez.")

    # -- pruebas de la hoja

    def p1_recta(self) -> None:
        for izq, der in PWM_RECTA:
            clave = f"1:{izq}/{der}"
            if self.hecho(clave):
                continue
            self.asegurar_espacio((1, -1), f"prueba 1, PWM {izq}/{der}")
            self.tel.grabar(self.dir / f"p1_{izq}_{der}.csv")
            ida = self.tramo(*self.orden(izq, der, DUR_MS))
            vuelta = self.tramo(*self.orden(-izq, -der, DUR_MS))  # vuelve al mismo lugar
            self.tel.grabar(None)
            self.guardar(clave, {"ida": ida, "vuelta": vuelta})
            print(f"    1) {izq}/{der}: {ida.get('v_media_mm_s', math.nan):.0f} mm/s, "
                  f"giro {ida.get('giro_total_deg', math.nan):+.1f}°")

    def p2_empuje(self) -> None:
        for izq, der in PWM_EMPUJE:
            clave = f"2:{izq}/{der}"
            if self.hecho(clave):
                continue
            self.asegurar_espacio((1,), f"prueba 2, PWM {izq}/{der}")
            p, color = self.esperar_cubo(f"Prueba 2 ({izq}/{der}): apoyá un cubo en las paletas, centrado")
            self.tel.grabar(self.dir / f"p2_{izq}_{der}.csv")
            m = self.tramo(*self.orden(izq, der, DUR_MS))
            fin = self.pose()
            self.tel.grabar(None)
            m["cubo"] = color
            m["despegado"] = self.cubo_al_frente(fin) != color
            self.guardar(clave, m)
            self.orden(-izq, -der, DUR_MS)  # se aleja del cubo
            print(f"    2) {izq}/{der}: {m.get('v_media_mm_s', math.nan):.0f} mm/s con cubo, "
                  f"{'SE DESPEGÓ' if m['despegado'] else 'cubo pegado'}")

    def p3_curva(self) -> None:
        for izq, der in PWM_CURVA:
            clave = f"3:{izq}/{der}"
            if self.hecho(clave):
                continue
            self.asegurar_espacio((1,), f"prueba 3, {izq}/{der}")
            p, color = self.esperar_cubo(f"Prueba 3 ({izq}/{der}): apoyá un cubo en las paletas, centrado")
            _, lat0 = relativo(p, (p["cubos"][color]["x"], p["cubos"][color]["y"]))
            self.tel.grabar(self.dir / f"p3_{izq}_{der}.csv")
            m = self.tramo(*self.orden(izq, der, DUR_MS))
            fin = self.pose()
            self.tel.grabar(None)
            c = fin["cubos"].get(color)
            lat1 = relativo(fin, (c["x"], c["y"]))[1] if c else math.nan
            m.update(cubo=color, corrimiento_mm=lat1 - lat0, salio=self.cubo_al_frente(fin) != color)
            self.guardar(clave, m)
            self.orden(-izq, -der, DUR_MS)
            print(f"    3) {izq}/{der}: {m.get('giro_dps', math.nan):.1f} °/s, corrimiento "
                  f"{m['corrimiento_mm']:+.0f} mm, {'SE SALIÓ' if m['salio'] else 'sigue'}")

    def p4_giro(self) -> None:
        pedir("Prueba 4: sacá los cubos de alrededor del rover (va a girar en sitio)")
        for i in range(3):
            for nombre, izq, der in (("izquierda", -25, 25), ("derecha", 25, -25)):
                clave = f"4:{nombre}:{i + 1}"
                if self.hecho(clave):
                    continue
                self.tel.grabar(self.dir / f"p4_{nombre}_{i + 1}.csv")
                m = self.tramo(*self.orden(izq, der, GIRO_MS))
                self.tel.grabar(None)
                self.guardar(clave, m)
                print(f"    4) {nombre} {i + 1}: {abs(m.get('giro_dps', math.nan)):.1f} °/s, "
                      f"residual {m.get('residual_deg', math.nan):+.1f}°")

    def p6_contacto(self) -> None:
        for toma, texto in ((1, "apoyá un cubo en las paletas, centrado"),
                            (2, "girá el rover 90° (con el cubo apoyado en las paletas)")):
            clave = f"6:{toma}"
            if self.hecho(clave):
                continue
            p, color = self.esperar_cubo(f"Prueba 6, toma {toma}: {texto}; después no toques nada 10 s")
            t0 = time.time() * 1000
            self.tel.grabar(self.dir / f"p6_toma{toma}.csv")
            time.sleep(10)
            self.tel.grabar(None)
            d = []
            for s in self.tel.serie(self.marcador, t0):
                c = s["cubos"].get(color)
                if c and c["age"] == 0:
                    adelante, lateral = relativo(s, (c["x"], c["y"]))
                    if 60 < adelante < 180 and abs(lateral) < 50:
                        d.append(math.hypot(c["x"] - s["x"], c["y"] - s["y"]))
            m = {"n": len(d), "mediana": statistics.median(d) if d else None,
                 "min": min(d, default=None), "max": max(d, default=None), "cubo": color}
            self.guardar(clave, m)
            print(f"    6) toma {toma}: mediana {m['mediana'] or math.nan:.1f} mm ({len(d)} capturas)")

    def p5_latencia(self) -> None:
        caps = [c for c in self.tel.capturas if c["rovers"]]
        lat = sorted(c["recv"] - c["t"] for c in caps)
        if len(lat) > 20:
            self.guardar("5", {"n": len(lat), "mediana": statistics.median(lat),
                               "p95": lat[int(0.95 * (len(lat) - 1))]})
            print(f"    5) latencia mediana {self.estado['5']['mediana']:.0f} ms, p95 {self.estado['5']['p95']:.0f} ms")

    # -- prueba IR: tablero de ajedrez (ver analisis_ir.py y line_guard.h)

    def ir_eventos(self, desde: float, hasta: float = math.inf) -> list[tuple[float, int]]:
        eventos = []
        for t, l in zip(list(self.rover.t_lineas), list(self.rover.lineas)):
            m = re.match(r"\[ir\] t=\d+ m=(\d+)", l)
            if m and desde <= t <= hasta:
                eventos.append((t, int(m.group(1))))
        return eventos

    def ir_moda(self, segundos: float) -> int | None:
        t0 = time.time() * 1000
        time.sleep(segundos)
        masks = [m for _, m in self.ir_eventos(t0)]
        return max(set(masks), key=masks.count) if masks else None

    def p_ir(self) -> None:
        import numpy as np
        import analisis_ir as A
        self.firmware("banco")
        desde = len(self.rover.lineas)
        self.rover.enviar("IR ON")
        if not self.rover.esperar(r"\[banco\] IR transmitiendo", 3, desde):
            raise Abortar("el firmware de banco no conoce `IR ON`: cargá la versión nueva")

        if not self.hecho("ir:estatico"):
            lect = {}
            for clave, texto in (("tablero", "IR 1/3: rover quieto sobre el tablero"),
                                 ("lona", "IR 2/3: poné el rover con los 4 sensores sobre la lona, FUERA del tablero"),
                                 ("aire", "IR 3/3: levantá el rover unos 3 cm y sostenelo quieto")):
                pedir(texto + " (se mide 2 s después del Enter)")
                lect[clave] = self.ir_moda(2)
            self.guardar("ir:estatico", lect)
            print(f"    IR quieto: tablero={lect['tablero']} lona={lect['lona']} aire={lect['aire']} "
                  "(bit i = S(i+1) en HIGH)")
            pedir("Volvé a poner el rover en el centro del tablero, sin cubos cerca")

        if not self.hecho("ir:ajuste"):
            print("[ir] recorrido automático: rectas en 3 rumbos y giros en sitio (~40 s)")
            t0 = time.time() * 1000
            for _ in range(3):
                self.asegurar_espacio((1, -1), "prueba IR")
                self.orden(18, 20, DUR_MS)
                self.orden(-18, -20, DUR_MS)
                self.orden(-25, 25, 400)
            self.orden(-25, 25, GIRO_MS)
            self.orden(25, -25, GIRO_MS)
            poses = self.tel.serie(self.marcador, t0)
            eventos = self.ir_eventos(t0 - 500)

            def muestras(desfase_ms: float):
                """Lectura IR vigente en el instante de cada captura, corrida `desfase_ms`."""
                filas, bits, k = [], [], 0
                for s_ in poses:
                    t = s_["t"] + desfase_ms
                    while k + 1 < len(eventos) and eventos[k + 1][0] <= t:
                        k += 1
                    if eventos and eventos[k][0] <= t:
                        filas.append((s_["x"], s_["y"], s_["th"]))
                        bits.append([(eventos[k][1] >> i) & 1 for i in range(4)])
                return np.array(filas), np.array(bits)

            P, B = muestras(15)
            if len(P) < 100:
                raise Abortar(f"muy pocas muestras IR+pose ({len(P)}): ¿se ve el marcador? ¿llega el IR?")
            # El reloj de captura de la cámara y la llegada del IR por Wi-Fi no están
            # alineados exactos; un desfase de 50 ms en un giro rápido ya es una
            # casilla. Se prueba cuál explica mejor las lecturas.
            print(f"[ir] ajustando posiciones con {len(P)} muestras (1-2 min)…")
            mejor = None
            for desfase in (-40, 0, 15, 40, 80, 120):
                Pd, Bd = muestras(desfase)
                ajustes_d, fase_d = A.ajustar(Pd, Bd)
                total = sum(a.acierto for a in ajustes_d)
                if mejor is None or total > mejor[0]:
                    mejor = (total, desfase, ajustes_d, fase_d, Pd, Bd)
            _, desfase, ajustes, fase, P, B = mejor
            print(f"    desfase IR-cámara que mejor explica los datos: {desfase} ms")
            np.savez(self.dir / "ir_recorrido.npz", poses=P, bits=B)
            datos = []
            for i, a in enumerate(ajustes):
                racha = None if a.trabado else A.racha_maxima(P, B, i, a)
                datos.append({"f": a.f, "l": a.l, "acierto": float(a.acierto), "cambios": a.cambios,
                              "trabado": a.trabado, "racha_mm": racha})
                print(f"    S{i + 1}: " + ("TRABADO (no cambió)" if a.trabado else
                      f"{a.f:+.0f} mm adelante, {a.l:+.0f} mm a la derecha, acierto {a.acierto:.0%}, "
                      f"racha máx. {racha:.0f} mm"))
            self.guardar("ir:ajuste", {"sensores": datos, "fase": fase, "desfase_ms": desfase})

        sensores = self.estado["ir:ajuste"]["sensores"]
        buenos = [i for i, d in enumerate(sensores) if not d["trabado"] and d["acierto"] >= 0.75]
        front = sum(1 << i for i in buenos if sensores[i]["f"] > 20)
        rear = sum(1 << i for i in buenos if sensores[i]["f"] < -20)
        rachas = [sensores[i]["racha_mm"] for i in buenos]
        edge = max(60.0, math.ceil(max(rachas, default=50) * 1.2 / 5) * 5)
        radio = statistics.fmean([math.hypot(sensores[i]["f"], sensores[i]["l"]) for i in buenos]) if buenos else 60
        self.guardar("ir:parametros", {"front": front, "rear": rear, "edge_mm": edge, "radio_mm": radio})
        if not front:
            print("    Sin sensores delanteros confiables: se salta la prueba de borde")
            return

        for n, texto in ((1, "mirando DERECHO hacia afuera (perpendicular al borde)"),
                         (2, "mirando hacia afuera en DIAGONAL (unos 45° respecto del borde)")):
            clave = f"ir:borde:{n}"
            if self.hecho(clave):
                continue
            pedir(f"Borde {n}/2: poné el rover a unos 10 cm del borde del tablero, {texto}. "
                  "Quedate cerca: avanza de a 2-3 cm hasta que el guardia detecte el borde")
            self.guardar(clave, self.ir_borde(A, front, rear, edge, radio, sensores))
            r = self.estado[clave]
            if r["detectado"]:
                print(f"    borde {n}: detectado tras {r['pulsos']} pasos; sensor {r['fuera_mm']:+.0f} mm "
                      "más allá del borde del tablero")
            else:
                print(f"    borde {n}: NO detectado en {r['pulsos']} pasos")
        self.rover.enviar("IR OFF")

    def ir_borde(self, A, front, rear, edge, radio, sensores) -> dict:
        """Avanza de a pasos cortos reproduciendo LineGuard con lo grabado hasta que detecta."""
        guard = A.LineGuard(edge, radio, front, rear)
        ancho, alto = self.tel.arena_mm()
        margen = 70.0  # el tablero físico se extiende 3,5 celdas más allá de la cancha lógica (MONTAJE.md)
        t0 = time.time() * 1000 - 300
        pulsos, detectado, fuera_mm, procesados = 0, None, None, 0
        while pulsos < 12 and detectado is None:
            self.orden(18, 20, 400)
            pulsos += 1
            eventos = sorted([(t, 0, m) for t, m in self.ir_eventos(t0)] +
                             [(s_["t"], 1, (s_["x"], s_["y"], s_["th"])) for s_ in self.tel.serie(self.marcador, t0)],
                             key=lambda e: (e[0], e[1]))
            guard = A.LineGuard(edge, radio, front, rear)  # se reproduce todo en orden cada vez
            for t, tipo, v in eventos:
                if tipo == 0:
                    guard.sample(v)
                else:
                    guard.pose(*v)
                    if guard.fuera() & front:
                        detectado = v
                        break
        if detectado:
            x, y, th = detectado
            peores = []
            for i in range(4):
                if front & (1 << i):
                    wx, wy = A.mundo(x, y, th, sensores[i]["f"], sensores[i]["l"])
                    peores.append(max(-margen - wx, -margen - wy, wx - ancho - margen, wy - alto - margen))
            fuera_mm = float(max(peores))
        for _ in range(pulsos):
            self.orden(-18, -20, 400)
        return {"detectado": detectado is not None, "pulsos": pulsos, "fuera_mm": fuera_mm}

    def banco(self) -> None:
        self.conectar_vision()
        pendientes = [c for c in ("1", "2", "3", "4", "6") if not any(k.startswith(c + ":") for k in self.estado)]
        if self.args.repetir or pendientes:
            self.firmware("banco")
            pedir("Rover en el centro de la cancha, sin cubos cerca")
            print("[prueba] paro de emergencia: ARM + MOTOR 18 20 1500 y a los 0,4 s se manda `!`")
            t0 = time.time()
            self.orden_con_paro()
            print(f"    `!` detuvo el rover ({time.time() - t0:.1f} s)")
            self.p1_recta()
            self.p2_empuje()
            self.p3_curva()
            self.p4_giro()
            self.p6_contacto()
        self.p5_latencia()
        self.informe()

    def orden_con_paro(self) -> None:
        self.rover.enviar("ARM")
        self.rover.esperar(r"\[banco\] armado", 2)
        t0 = time.time() * 1000
        self.rover.enviar("MOTOR 18 20 1500")
        time.sleep(0.4)
        self.rover.parar()
        time.sleep(0.8)
        s = self.tel.serie(self.marcador, t0 + 600, t0 + 1500)
        if len(s) >= 2 and math.hypot(s[-1]["x"] - s[0]["x"], s[-1]["y"] - s[0]["y"]) > 15:
            raise Abortar("¡`!` NO detuvo el rover! No seguir hasta revisar el firmware")
        self.guardar("paro", True)

    def ritmo(self) -> None:
        self.conectar_vision()
        for nombre, valores in RITMOS.items():
            for i in range(1, self.args.rondas + 1):
                clave = f"7:{nombre}:{i}"
                if self.hecho(clave):
                    continue
                self.firmware("autonomo", valores)
                pedir(f"Prueba 7, ritmo {nombre}, ronda {i}/{self.args.rondas}: rover en la salida y UN cubo "
                      "en la misma posición de siempre. En la terminal de la visión apretá `r` "
                      "(para frenarlo en cualquier momento: `stop` en la visión)")
                self.guardar(clave, self.una_ronda(f"p7_{nombre.replace('/', '_')}_{i}.csv"))
                r = self.estado[clave]
                print(f"    7) {nombre} #{i}: {r['tiempo_s'] or math.nan:.1f} s, "
                      f"{'ENTREGADO' if r['entregado'] else 'no entregado'}, falta {r['falta_mm']:.0f} mm")
        self.informe()

    def una_ronda(self, archivo: str) -> dict:
        self.tel.grabar(self.dir / archivo)
        desde = len(self.rover.lineas)
        print("    esperando RUNNING…")
        while self.tel.ultimo["phase"] != "RUNNING":
            time.sleep(0.1)
        print("    RUNNING. Cuando termine (o `stop` en la visión) sigo solo.")
        while self.tel.ultimo["phase"] == "RUNNING":
            time.sleep(0.1)
        time.sleep(0.5)
        self.tel.grabar(None)
        msg = self.tel.ultimo
        faltas, entregados = [], 0
        for c in msg["cubes"]:
            depot = next(d for d in msg["depots"] if d["color"] == c["color"])
            adentro, falta = cubo_en_su_zona(c, depot, msg["depot_size"], msg["grid"], msg["cube_side"])
            entregados += adentro
            faltas.append(falta * msg["grid"]["cell_mm"])
        lineas = self.rover.lineas[desde:]
        motivos = [m.group(1) for l in lineas if (m := re.search(r"motivo=(\S+)", l))]
        clock = msg["clock"]
        return {"tiempo_s": clock["elapsed_ms"] / 1000, "cumplido": clock["remaining_ms"] > 0 and bool(entregados),
                "entregado": entregados == len(msg["cubes"]) and entregados > 0,
                "falta_mm": max(faltas, default=math.nan),
                "motivos": {m: motivos.count(m) for m in set(motivos)}}

    # -- informe

    def informe(self) -> None:
        e = self.estado
        f = lambda v, d=0: "—" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{d}f}"
        L = [f"# Resultados — sesión {self.dir.name}", "",
             f"Rover físico 1, marcador {self.marcador}. Generado por `sesion_viernes.py`; "
             "CSV y `serie.log` en esta carpeta.", "",
             f"Paro con `!`: {'OK' if e.get('paro') else 'sin probar'}", "",
             "## 1. Velocidad sin cubo", "", "| PWM | v media (mm/s) | v en régimen (mm/s) | Giro total (°) | v reversa (mm/s) |",
             "| --- | --- | --- | --- | --- |"]
        for izq, der in PWM_RECTA:
            m = e.get(f"1:{izq}/{der}", {})
            ida, vu = m.get("ida", {}), m.get("vuelta", {})
            L.append(f"| {izq}/{der} | {f(ida.get('v_media_mm_s'))} | {f(ida.get('v_firme_mm_s'))} | "
                     f"{f(ida.get('giro_total_deg'), 1)} | {f(vu.get('v_media_mm_s'))} |")
        L += ["", "## 2. Empujando un cubo", "", "| PWM | v con cubo | v sin cubo | Factor | ¿Se despegó? |", "| --- | --- | --- | --- | --- |"]
        factores = []
        for izq, der in PWM_EMPUJE:
            m = e.get(f"2:{izq}/{der}", {})
            sin = e.get(f"1:{izq}/{der}", {}).get("ida", {}).get("v_media_mm_s")
            con = m.get("v_media_mm_s")
            fac = con / sin if con and sin else None
            if fac:
                factores.append(fac)
            L.append(f"| {izq}/{der} | {f(con)} | {f(sin)} | {f(fac, 2)} | "
                     f"{'—' if not m else ('sí' if m.get('despegado') else 'no')} |")
        L += ["", "## 3. Curva con cubo", "", "| Izq/der | Giro (°/s) | Corrimiento lateral (mm) | ¿Se salió? |", "| --- | --- | --- | --- |"]
        for izq, der in PWM_CURVA:
            m = e.get(f"3:{izq}/{der}", {})
            L.append(f"| {izq}/{der} | {f(m.get('giro_dps'), 1)} | {f(m.get('corrimiento_mm'))} | "
                     f"{'—' if not m else ('sí' if m.get('salio') else 'no')} |")
        L += ["", "## 4. Giro en sitio a 25 %", "", "| Sentido | °/s (1) | °/s (2) | °/s (3) | Residual medio (°) |", "| --- | --- | --- | --- | --- |"]
        tasas = []
        for nombre in ("izquierda", "derecha"):
            ms = [e.get(f"4:{nombre}:{i}", {}) for i in (1, 2, 3)]
            ts = [abs(m["giro_dps"]) for m in ms if m.get("giro_dps") is not None and not math.isnan(m["giro_dps"])]
            tasas += ts
            res = [abs(m["residual_deg"]) for m in ms if "residual_deg" in m]
            L.append(f"| {nombre} | " + " | ".join(f(abs(m.get('giro_dps', math.nan)), 1) for m in ms)
                     + f" | {f(statistics.fmean(res) if res else None, 1)} |")
        lat = e.get("5", {})
        L += ["", "## 5. Latencia de la visión", "", f"Mediana {f(lat.get('mediana'))} ms · p95 {f(lat.get('p95'))} ms "
              f"({lat.get('n', 0)} capturas).", "", "## 6. Distancia de contacto", "",
              "| Toma | Mediana (mm) | Mín | Máx | Capturas |", "| --- | --- | --- | --- | --- |"]
        contactos = []
        for toma in (1, 2):
            m = e.get(f"6:{toma}", {})
            if m.get("mediana"):
                contactos.append(m["mediana"])
            L.append(f"| {toma} | {f(m.get('mediana'), 1)} | {f(m.get('min'), 1)} | {f(m.get('max'), 1)} | {m.get('n', '—')} |")
        L += ["", "## 7. Ritmo", "", "| Ritmo | Tiempos (s) | Entregados | Falta máx. (mm) | Motivos serie |", "| --- | --- | --- | --- | --- |"]
        for nombre in RITMOS:
            rs = [e[k] for k in sorted(e) if k.startswith(f"7:{nombre}:")]
            L.append(f"| {nombre} | {', '.join(f(r['tiempo_s'], 1) for r in rs) or '—'} | "
                     f"{sum(r['entregado'] for r in rs)}/{len(rs)} | {f(max((r['falta_mm'] for r in rs), default=None))} | "
                     f"{'; '.join(f'{k}×{v}' for r in rs for k, v in r['motivos'].items()) or '—'} |")
        L += self.informe_ir()
        v1820 = e.get("1:18/20", {}).get("ida", {}).get("v_media_mm_s")
        L += ["", "## Valores sugeridos (no se aplican solos)", "",
              f"- `kNominalMmPerS` / `Physics::k_mm_s`: v a 18/20 = {f(v1820)} mm/s (referencia 19/09: ~64).",
              f"- `Physics::push_factor`: {f(statistics.fmean(factores) if factores else None, 2)}.",
              f"- `kBlindTurnDps`: la tasa más alta medida = {f(max(tasas) if tasas else None, 1)} °/s (hoy 60).",
              f"- `NAV_PUSH_CONTACT_OFFSET_MM`: {f(statistics.fmean(contactos) if contactos else None)} mm (hoy 107).",
              f"- `ARENA_SETTLE_MS`: p95 de latencia {f(lat.get('p95'))} ms; si pasa de ~150, subir la pausa.", ""]
        (self.dir / "RESULTADOS.md").write_text("\n".join(L))
        print(f"[informe] {self.dir / 'RESULTADOS.md'}")

    def informe_ir(self) -> list[str]:
        e = self.estado
        if "ir:ajuste" not in e:
            return []
        L = ["", "## IR sobre el tablero de ajedrez", ""]
        q = e.get("ir:estatico", {})
        if q:
            L += [f"Quieto (bit i = S(i+1) en HIGH): tablero {q.get('tablero')}, lona {q.get('lona')}, "
                  f"en el aire {q.get('aire')}.", ""]
        L += ["| Sensor | Adelante (mm) | Derecha (mm) | Acierto | Cambios | Racha máx. sobre la cancha (mm) |",
              "| --- | --- | --- | --- | --- | --- |"]
        for i, d in enumerate(e["ir:ajuste"]["sensores"]):
            if d["trabado"]:
                L.append(f"| S{i + 1} | — | — | — | {d['cambios']} | TRABADO |")
            else:
                L.append(f"| S{i + 1} | {d['f']:+.0f} | {d['l']:+.0f} | {d['acierto']:.0%} | {d['cambios']} | "
                         f"{d['racha_mm']:.0f} |")
        par = e.get("ir:parametros", {})
        bordes = [e[k] for k in ("ir:borde:1", "ir:borde:2") if k in e]
        for n, b in enumerate(bordes, 1):
            L += ["", f"Borde {n}: " + (f"detectado con el sensor {b['fuera_mm']:+.0f} mm más allá del tablero"
                                        if b["detectado"] else f"NO detectado en {b['pulsos']} pasos")]
        lona = q.get("lona")
        habilitar = bool(par.get("front")) and len(bordes) == 2 and all(b["detectado"] for b in bordes)
        L += ["", "Configuración sugerida para `config.h`" + ("" if habilitar else
              " (NO habilitar todavía: falta detectar el borde en las dos pruebas)") + ":", "", "```",
              f"#define LINE_STOP_ON_DETECTION {'true' if habilitar else 'false'}"]
        if lona is not None:
            L.append(f"#define LINE_BLACK_IS_LOW {'true' if lona == 15 else 'false'}  // la lona (blanca) leyó {lona}")
        L += [f"#define LINE_FRONT_MASK 0b{par.get('front', 0):04b}",
              f"#define LINE_REAR_MASK 0b{par.get('rear', 0):04b}",
              f"#define LINE_EDGE_MM {par.get('edge_mm', 60):.0f}.0F",
              f"#define LINE_SENSOR_RADIUS_MM {par.get('radio_mm', 60):.0f}.0F", "```"]
        return L

    def cerrar(self) -> None:
        if self.rover:
            self.rover.cerrar()
        if self.tel:
            self.tel.cerrar()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("orden", nargs="?", default="todo", choices=("todo", "banco", "ir", "ritmo", "preparar", "informe"))
    p.add_argument("modo", nargs="?", choices=("banco", "autonomo", "red"), help="solo con `preparar`")
    p.add_argument("--host", default="127.0.0.1", help="visión, para leer la telemetría")
    p.add_argument("--ip", help="IP que se graba en el rover (por defecto: se detecta)")
    p.add_argument("--puerto", help="USB del rover (por defecto: se detecta)")
    p.add_argument("--marcador", type=int, help="ID ArUco del rover (por defecto: el único que se ve)")
    p.add_argument("--rondas", type=int, default=3, help="rondas por ritmo en la prueba 7")
    p.add_argument("--sesion", help="carpeta de sesión (por defecto la fecha de hoy)")
    p.add_argument("--ritmo", choices=tuple(RITMOS), help="solo con `preparar autonomo`")
    p.add_argument("--repetir", action="store_true", help="rehacer también lo ya medido")
    p.add_argument("--no-cargar", action="store_true", help="no compilar/cargar (el firmware ya está)")
    args = p.parse_args()

    if args.orden == "preparar":
        if not args.modo:
            p.error("preparar necesita el modo: banco, autonomo o red")
        ritmo = RITMOS.get(args.ritmo) if args.ritmo else None
        try:
            preparar(args.modo, args.ip or ip_vision(), args.marcador or 10, args.puerto, ritmo,
                     cargar=not args.no_cargar)
        except Abortar as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 1
        return 0

    s = Sesion(args)
    try:
        if args.orden == "informe":
            s.informe()
        if args.orden in ("todo", "banco"):
            s.banco()
        if args.orden in ("todo", "ir"):
            s.conectar_vision()
            s.p_ir()
            s.informe()
        if args.orden in ("todo", "ritmo"):
            s.ritmo()
    except KeyboardInterrupt:
        print("\n[!] interrumpido: rover detenido; lo medido quedó guardado, se reanuda desde ahí")
        return 130
    except Abortar as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        return 1
    finally:
        s.cerrar()
    return 0


if __name__ == "__main__":
    sys.exit(main())
