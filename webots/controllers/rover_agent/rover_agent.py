"""Cliente de referencia del contrato v2; no usa la pose real de Webots."""
from controller import Robot
import json, socket, time
from rover_logic import assigned_color, navigate, should_yield

STEP = 32
class Feed:
    def __init__(self): self.sock = None; self.buf = b""; self.world = None; self.retry = 0
    def poll(self):
        if self.sock is None and time.monotonic() >= self.retry:
            self.retry = time.monotonic() + 1
            try:
                self.sock = socket.create_connection(("127.0.0.1", 2026), .2); self.sock.setblocking(False)
            except OSError: self.sock = None; return
        if self.sock is None: return
        try:
            while True:
                data = self.sock.recv(4096)
                if not data: raise OSError()
                self.buf += data
        except BlockingIOError: pass
        except OSError: self.sock.close(); self.sock = None; return
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                candidate = json.loads(line)
                if candidate.get("v") == 2: self.world = candidate
            except ValueError: pass

def control(world, rover_id, color, pushing):
    return navigate(world, rover_id, color, pushing)

robot = Robot(); feed = Feed(); rover_id = 10 if robot.getName() == "rover_10" else 11
pushing = False
while robot.step(STEP) != -1:
    feed.poll()
    target = assigned_color(feed.world, rover_id)
    left, right, pushing = control(feed.world, rover_id, target, pushing) if target else (0, 0, False)
    if should_yield(feed.world, rover_id):
        left, right, pushing = 0, 0, False
    robot.setCustomData(f"{left:.4f},{right:.4f}")
