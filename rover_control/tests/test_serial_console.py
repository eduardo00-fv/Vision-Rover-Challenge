"""Real console process over pseudo-terminals; no physical hardware is touched."""
import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "diagnostico" / "serial_console.py"
spec = importlib.util.spec_from_file_location("serial_console", SCRIPT)
console = importlib.util.module_from_spec(spec)
spec.loader.exec_module(console)


class ConsoleTest(unittest.TestCase):
    def test_stop_clears_incomplete_command_without_enter(self):
        keyboard = console.Keyboard()
        for byte in b"ARM":
            self.assertEqual(keyboard.accept(byte)[0], "echo")
        self.assertEqual(keyboard.accept(ord("!")), ("send", b"!\n"))
        self.assertEqual(keyboard.accept(10), ("echo", b"\n"))
        for byte in b"STATUS":
            keyboard.accept(byte)
        self.assertEqual(keyboard.accept(10), ("send", b"STATUS\n"))

    def test_persistent_manual_console_records_and_stops(self):
        device, device_slave = pty.openpty()
        terminal, terminal_slave = pty.openpty()

        def read_until(fd, expected):
            result = b""
            deadline = time.monotonic() + 5
            while expected not in result and time.monotonic() < deadline:
                if select.select([fd], [], [], 0.1)[0]:
                    result += os.read(fd, 8192)
            self.assertIn(expected, result)
            return result

        process = None
        try:
            with tempfile.TemporaryDirectory() as directory:
                log = Path(directory) / "session.ndjson"
                process = subprocess.Popen(
                    [sys.executable, str(SCRIPT), "--port", os.ttyname(device_slave),
                     "--rover", "2", "--log", str(log)],
                    stdin=terminal_slave, stdout=terminal_slave, stderr=terminal_slave)
                read_until(device, b"!\n")
                read_until(terminal, b"ARM siempre manual")
                os.write(terminal, b"!")  # deliberately no LF
                read_until(device, b"!\n")
                os.write(terminal, b"STATUS\n")
                read_until(device, b"STATUS\n")
                os.write(device, b"STATUS armed=NO ")
                os.write(device, b"moving=NO\r\n")
                read_until(terminal, b"moving=NO")
                os.write(terminal, b"\x04")
                read_until(device, b"!\n")
                self.assertEqual(process.wait(timeout=5), 0)
                records = [json.loads(line) for line in log.read_text().splitlines()]
                self.assertTrue(all(item["rover"] == "2" for item in records))
                self.assertTrue(any(item["text"] == "STATUS armed=NO moving=NO" for item in records))
                tx = [item["text"] for item in records if item["direction"] == "tx"]
                self.assertEqual(tx, ["!\n", "!\n", "STATUS\n", "!\n"])
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            for fd in (device, device_slave, terminal, terminal_slave):
                os.close(fd)


if __name__ == "__main__":
    unittest.main()
