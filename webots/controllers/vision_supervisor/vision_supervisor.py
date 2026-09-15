"""Fuente de verdad de Webots y publicador TCP del contrato oficial v2."""
from controller import Supervisor, Keyboard
import json, math, os, socket, sys, time
from pathlib import Path
from run_log import RunLog
from scenario import load_scenario
from safety_oracle import evaluate as evaluate_safety
from vision_adapter import VisionAdapter, webots_bgra_to_bgr

STEP = 32
root = Path(__file__).resolve().parents[3]
scenario_ref = Path(os.environ.get("VRC_SCENARIO", "webots/scenarios/nominal_3_cubos.json"))
scenario_path = scenario_ref if scenario_ref.is_absolute() else root / scenario_ref
scenario, cfg = load_scenario(scenario_path)
sys.path.insert(0, str(root / "vision-system"))
from contrato.schema import cubo_en_depot, geometria_depot
from vision.sistema import Arbitro

CELL = cfg["grid"]["cell_mm"] / 1000
grid = {"cols": cfg["grid"]["cols"], "rows": cfg["grid"]["rows"], "cell_mm": cfg["grid"]["cell_mm"]}
depots = [{"color": d["color"], "col": d["col"], "row": d["row"]} for d in cfg["depots"]]
cubes_cfg = cfg["cubes"]
patologias = cfg["patologias"]
sup = Supervisor(); keyboard = sup.getKeyboard(); keyboard.enable(STEP)
overhead_camera = sup.getDevice("overhead_camera"); overhead_camera.enable(STEP)
vision_adapter = VisionAdapter(
    root / "vision-system/vision/config_vision.json",
    overhead_camera.getWidth(), overhead_camera.getHeight(), overhead_camera.getFov(),
)
if (vision_adapter.cfg.tablero.cols, vision_adapter.cfg.tablero.rows,
        vision_adapter.cfg.tablero.cell_mm) != (grid["cols"], grid["rows"], grid["cell_mm"]):
    raise RuntimeError("La visión y el escenario Webots declaran canchas distintas")
server = socket.socket(); server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); server.bind(("0.0.0.0", 2026)); server.listen(); server.setblocking(False)
clients, seq, last_pub = [], 0, 0.0
referee = Arbitro(vision_adapter.cfg, "IDLE")
run_log = RunLog(os.environ.get("VRC_RUN_DIR", "/tmp/vision-rover-webots"), scenario, cfg)
summary_written = False
critical_failures = set()

def node_for(def_name):
    node = sup.getFromDef(def_name)
    if node is None:
        raise RuntimeError("El escenario {} requiere DEF {} en el mundo".format(scenario["id"], def_name))
    return node

def movable(def_name):
    node = node_for(def_name)
    return {"node": node, "t": node.getField("translation"), "r": node.getField("rotation")}

rovers = []
for item in cfg["rovers"]:
    rover = movable("ROVER_%d" % item["id"])
    rover.update({"id": item["id"], "initial": item, "last": None, "age": 0})
    rovers.append(rover)
cubes = []
for item in cubes_cfg:
    cube = movable("CUBE_" + item["color"].upper())
    cube.update({"color": item["color"], "initial": item, "last": [item["col"], item["row"]], "age": 0})
    cubes.append(cube)
depot_nodes = []
for depot in depots:
    node = node_for("DEPOT_" + depot["color"].upper())
    depot_nodes.append((depot, node.getField("translation")))

def reset_world():
    """Restablece la verdad física desde el escenario y limpia observaciones."""
    for rover in rovers:
        initial = rover["initial"]
        rover["t"].setSFVec3f([initial["col"] * CELL, .035, initial["row"] * CELL])
        rover["r"].setSFRotation([0, 1, 0, math.radians(initial["theta"]) - math.pi / 2])
        rover["node"].getField("customData").setSFString("0,0")
        rover["last"], rover["age"] = None, 0
    for cube in cubes:
        initial = cube["initial"]
        cube["t"].setSFVec3f([initial["col"] * CELL, .035, initial["row"] * CELL])
        cube["last"], cube["age"] = [initial["col"], initial["row"]], 0
    for depot, translation in depot_nodes:
        translation.setSFVec3f([depot["col"] * CELL, .002, depot["row"] * CELL])

reset_world()

def truth():
    """Estado físico: solo para oráculo y trazas, nunca para los rovers."""
    truth_rovers = []
    for rover in rovers:
        col, row, theta = pose(rover)
        truth_rovers.append({"id": rover["id"], "col": col, "row": row, "theta": theta,
                             "command": rover["node"].getField("customData").getSFString()})
    truth_cubes = []
    for cube in cubes:
        position = cube["t"].getSFVec3f()
        truth_cubes.append({"color": cube["color"], "col": position[0] / CELL,
                            "row": position[2] / CELL})
    return {"rovers": truth_rovers, "cubes": truth_cubes}

def deliveries_from_truth():
    """Aplica la misma geometría contractual al estado físico final."""
    depot_by_color = {depot["color"]: depot for depot in depots}
    results = {}
    for cube in truth()["cubes"]:
        depot = depot_by_color[cube["color"]]
        geometry = geometria_depot(
            col=depot["col"], row=depot["row"],
            length=cfg["depot_size"]["length"], depth=cfg["depot_size"]["depth"],
            cols=grid["cols"], rows=grid["rows"], cube_side=cfg["cube_side"],
        )
        verdict = cubo_en_depot(col=cube["col"], row=cube["row"], geometria=geometry)
        results[cube["color"]] = {"delivered": verdict.adentro, "missing_cells": verdict.falta_celdas}
    return results

def pose(rover):
    p, rot = rover["t"].getSFVec3f(), rover["r"].getSFRotation()
    theta = (math.degrees(rot[3] + math.pi / 2) % 360)
    return p[0] / CELL, p[2] / CELL, theta

def update_rover(rover, dt):
    try: left, right = (float(v) for v in rover["node"].getField("customData").getSFString().split(","))
    except ValueError: left = right = 0
    left, right = max(-1, min(1, left)), max(-1, min(1, right))
    col, row, theta = pose(rover)
    theta = (theta + (right - left) * cfg["giro_rover_grados_s"] * dt) % 360
    speed_cells_s = (left + right) * .5 * cfg["velocidad_rover_celdas_s"]
    col = min(grid["cols"] - 2, max(2, col + math.cos(math.radians(theta)) * speed_cells_s * dt))
    row = min(grid["rows"] - 2, max(2, row - math.sin(math.radians(theta)) * speed_cells_s * dt))
    rover["t"].setSFVec3f([col * CELL, .035, row * CELL]); rover["r"].setSFRotation([0, 1, 0, math.radians(theta) - math.pi / 2])
    # Empuje simplificado con las paletas: arrastra un cubo frente al rover.
    for cube in cubes:
        cp = cube["t"].getSFVec3f(); dc, dr = cp[0] / CELL - col, cp[2] / CELL - row
        if math.hypot(dc, dr) < patologias["radio_empuje_celdas"] and speed_cells_s > 0:
            margin = cfg["cube_side"] * CELL / 2
            cube["t"].setSFVec3f([
                min(grid["cols"] * CELL - margin, max(margin, cp[0] + math.cos(math.radians(theta)) * speed_cells_s * CELL * dt)),
                .035,
                min(grid["rows"] * CELL - margin, max(margin, cp[2] - math.sin(math.radians(theta)) * speed_cells_s * CELL * dt)),
            ])

def clock():
    """El reloj es del Arbitro de visión, no del tiempo de Webots."""
    current = referee.reloj()
    return {"elapsed_ms": current.transcurrido_ms, "remaining_ms": current.restante_ms,
            "total_ms": current.total_ms}

def observation():
    """Telemetría que sale de la imagen, nunca de la verdad del Supervisor."""
    global seq
    seq += 1
    image = webots_bgra_to_bgr(overhead_camera.getImage(), overhead_camera.getWidth(),
                                overhead_camera.getHeight())
    current_phase, current_clock = referee.instantanea()
    current_clock = {"elapsed_ms": current_clock.transcurrido_ms,
                     "remaining_ms": current_clock.restante_ms,
                     "total_ms": current_clock.total_ms}
    message = vision_adapter.observe(image, int(time.time() * 1000), current_phase,
                                      current_clock, seq)
    # El árbitro solo avanza tras una geometría que el pipeline pudo construir.
    # Así READY no se consume a ciegas y RUNNING cierra si la visión se pierde.
    referee.tictac(vision_adapter.last_error is None)
    return message

while sup.step(STEP) != -1:
    previous_phase = referee.fase
    key = keyboard.getKey()
    command = None
    if key in (ord("R"), ord("r")):
        reset_world()
        command = "ready"
    elif key in (ord("I"), ord("i")):
        command = "abort"
    elif key in (ord("F"), ord("f")):
        command = "stop"
    elif key in (ord("S"), ord("s")):
        run_log.write("phase_request_rejected", ts_ms=int(time.time() * 1000),
                      command="start", reason="RUNNING solo lo inicia el Arbitro al vencer READY")
    if command is not None:
        run_log.write("phase_request", ts_ms=int(time.time() * 1000), command=command,
                      result=referee.intentar(command))
    if referee.fase == "RUNNING":
        for rover in rovers: update_rover(rover, STEP / 1000)
    try:
        while True: clients.append(server.accept()[0]); clients[-1].setblocking(False)
    except BlockingIOError: pass
    if sup.getTime() - last_pub >= 1 / cfg["tasas"]["pub_hz"]:
        last_pub = sup.getTime(); message = observation(); payload = (json.dumps(message, separators=(",", ":")) + "\n").encode()
        truth_state = truth()
        safety = evaluate_safety(truth_state, grid, **scenario["oracle"])
        new_failures = set(safety["failures"]) - critical_failures
        critical_failures.update(safety["failures"])
        for failure in sorted(new_failures):
            run_log.write("safety_failure", ts_ms=message["ts_ms"], failure=failure, truth=truth_state)
        run_log.write("sample", telemetry=message, truth=truth_state, safety=safety,
                      vision_error=vision_adapter.last_error)
        for client in clients[:]:
            try:
                client.sendall(payload)
            except OSError:
                clients.remove(client); client.close()
    phase = referee.fase
    if phase != previous_phase:
        run_log.write("phase", ts_ms=int(time.time() * 1000), phase=phase, clock=clock())
    if phase == "FINISHED" and not summary_written:
        status = "FAIL" if critical_failures else "INCONCLUSIVE"
        reason = ("; ".join(sorted(critical_failures)) if critical_failures else
                  "Falta física calibrada y control de rover real para emitir PASS.")
        run_log.close(
            status=status,
            reason=reason,
            phase=phase,
            clock=clock(),
            deliveries=deliveries_from_truth(),
            safety_failures=sorted(critical_failures),
        )
        summary_written = True
