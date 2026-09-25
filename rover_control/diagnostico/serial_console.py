#!/usr/bin/env python3
"""Consola Linux: comandos manuales y log compartido. Firmware Arduino C++."""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import select
import sys
import termios
import tty
import uuid

import serial


class Keyboard:
    """Edición local; ! siempre produce paro sin esperar Enter."""
    def __init__(self):
        self.draft = bytearray()

    def accept(self, char: int):
        if char in (3, 4):
            self.draft.clear()
            return "exit", b"!\n"
        if char == ord("!"):
            self.draft.clear()
            return "send", b"!\n"  # LF clears firmware discard state
        if char in (10, 13):
            command = bytes(self.draft).strip()
            self.draft.clear()
            return ("send", command + b"\n") if command else ("echo", b"\n")
        if char in (8, 127):
            if self.draft:
                self.draft.pop()
                return "echo", b"\b \b"
            return "echo", b""
        if 32 <= char < 127:
            if len(self.draft) >= 95:
                self.draft.clear()
                return "send", b"!\n"
            self.draft.append(char)
            return "echo", bytes([char])
        return "echo", b""


def main() -> int:
    parser = argparse.ArgumentParser(description="Consola CenfoBot: comandos manuales y registro compartido")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--rover", required=True, choices=("1", "2"), help="identidad física declarada por el operador")
    parser.add_argument("--log", type=Path, help="por defecto /tmp/cenfobot-roverN.ndjson (append)")
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("abrí la consola en una terminal interactiva; no admite secuencias por pipe")
    path = args.log or Path(f"/tmp/cenfobot-rover{args.rover}.ndjson")
    path.parent.mkdir(parents=True, exist_ok=True)
    session = uuid.uuid4().hex
    log = path.open("a", encoding="utf-8", buffering=1)
    link = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.5, exclusive=True)
    link.dtr = False
    link.rts = False
    link.port = args.port
    terminal = None
    pending = b""

    def record(direction, text):
        log.write(json.dumps({"ts": dt.datetime.now().astimezone().isoformat(timespec="milliseconds"),
                              "session": session, "rover": args.rover,
                              "direction": direction, "text": text}, ensure_ascii=False) + "\n")

    def send(data):
        record("tx", data.decode("ascii", errors="replace"))
        if link.write(data) != len(data):
            raise serial.SerialException("escritura serial incompleta")

    try:
        link.open()
        record("session", f"port={args.port} baud=115200; rover declarado por operador")
        send(b"!\n")
        print(f"Rover {args.rover}: {args.port} @115200. Registro: {path}", flush=True)
        print("Comandos + Enter. ! para inmediato. Ctrl-C/Ctrl-D: paro y salir.", flush=True)
        print("Esperá cada resultado antes de la siguiente orden. ARM siempre manual.", flush=True)
        terminal = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        keyboard = Keyboard()
        while True:
            ready, _, _ = select.select([sys.stdin.fileno(), link.fileno()], [], [], 0.1)
            if sys.stdin.fileno() in ready:
                keys = os.read(sys.stdin.fileno(), 1024)
                if not keys:
                    break
                for char in keys:
                    action, data = keyboard.accept(char)
                    if action == "exit":
                        return 0
                    if action == "send":
                        send(data)
                        print(f"\n>>> {data.decode().strip()}", flush=True)
                    else:
                        sys.stdout.write(data.decode())
                        sys.stdout.flush()
            if link.fileno() in ready:
                data = link.read(min(link.in_waiting or 1, 4096))
                if not data:
                    raise serial.SerialException("dispositivo desconectado")
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
                pending += data
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    record("rx", line.decode("utf-8", errors="replace").rstrip("\r"))
                if len(pending) >= 4096:
                    record("rx_partial", pending.decode("utf-8", errors="replace"))
                    pending = b""
    except KeyboardInterrupt:
        return 0
    except (OSError, serial.SerialException) as exc:
        print(f"\nConsola cerrada: {exc}", file=sys.stderr)
        try:
            record("error", str(exc))
        except OSError:
            pass
        return 2
    finally:
        if link.is_open:
            try:
                link.write(b"!\n")
                record("tx", "!\n")
            except (OSError, serial.SerialException):
                pass
            link.close()
        if terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, terminal)
        if pending:
            try:
                record("rx_partial", pending.decode("utf-8", errors="replace"))
            except OSError:
                pass
        log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
