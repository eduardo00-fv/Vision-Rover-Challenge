"""Cliente de referencia del contrato v2; no usa la pose real de Webots."""
from controller import Robot
import json, os, socket, time
from rover_logic import assigned_color, navigate, should_yield

STEP = 32
# Radio y velocidad máxima de rueda del modelo Webots. Son parámetros del
# simulador, no de la estrategia: la estrategia continúa entregando mandos
# normalizados en [-1, 1], igual que el firmware.
MAX_WHEEL_RAD_S = 4.8


def bench_command():
    """Mando fijo opcional para calibrar ruedas sin alterar una misión.

    Solo se habilita explícitamente con ``VRC_ACTUATOR_TEST=izquierda,derecha``
    y permite medir desplazamiento, giro y frenado físicos desde una traza.
    """
    value = os.environ.get("VRC_ACTUATOR_TEST")
    if not value:
        return None
    try:
        left, right = (float(item) for item in value.split(","))
    except ValueError as exc:
        raise RuntimeError("VRC_ACTUATOR_TEST debe ser izquierda,derecha") from exc
    return max(-1.0, min(1.0, left)), max(-1.0, min(1.0, right))
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
test_command = bench_command()
left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)
pushing = False
while robot.step(STEP) != -1:
    feed.poll()
    target = assigned_color(feed.world, rover_id)
    left, right, pushing = control(feed.world, rover_id, target, pushing) if target else (0, 0, False)
    if should_yield(feed.world, rover_id):
        left, right, pushing = 0, 0, False
    if test_command is not None:
        left, right = test_command
    left_motor.setVelocity(left * MAX_WHEEL_RAD_S)
    right_motor.setVelocity(right * MAX_WHEEL_RAD_S)
    # Solo telemetría para la traza: el Supervisor jamás la usa para imponer
    # movimiento; la física de ruedas es la única que cambia la pose.
    robot.setCustomData(f"{left:.4f},{right:.4f}")
