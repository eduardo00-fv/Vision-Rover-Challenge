"""Fuente de verdad de Webots y publicador TCP del contrato oficial v2."""
from controller import Supervisor, Keyboard
import json, math, os, socket, sys, time
from pathlib import Path
from run_log import RunLog
from scenario import load_scenario
from safety_oracle import evaluate as evaluate_safety
from run_verdict import evaluate as evaluate_verdict
from vision_adapter import VisionAdapter, webots_bgra_to_bgr

STEP = 32
root = Path(__file__).resolve().parents[3]
scenario_ref = Path(os.environ.get("VRC_SCENARIO", "webots/scenarios/nominal_3_cubos.json"))
scenario_path = scenario_ref if scenario_ref.is_absolute() else root / scenario_ref
scenario, cfg = load_scenario(scenario_path)
sys.path.insert(0, str(root / "vision-system"))
from contrato.schema import cubo_en_depot, geometria_depot
from vision.sistema import Arbitro
from vision.reglas.acopio import ContadorAcopio

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
counter = ContadorAcopio(vision_adapter.cfg)
run_log = RunLog(os.environ.get("VRC_RUN_DIR", "/tmp/vision-rover-webots"), scenario, cfg)
summary_written = False
critical_failures = set()
official_completion = False

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
        rover["t"].setSFVec3f([initial["col"] * CELL, initial["row"] * CELL, .0525])
        rover["r"].setSFRotation([0, 0, 1, -math.radians(initial["theta"])])
        rover["node"].resetPhysics()
        rover["node"].getField("customData").setSFString("0,0")
        rover["last"], rover["age"] = None, 0
    for cube in cubes:
        initial = cube["initial"]
        cube["t"].setSFVec3f([initial["col"] * CELL, initial["row"] * CELL, .035])
        cube["r"].setSFRotation([0, 0, 1, 0])
        cube["node"].resetPhysics()
        cube["last"], cube["age"] = [initial["col"], initial["row"]], 0
    for depot, translation in depot_nodes:
        translation.setSFVec3f([depot["col"] * CELL, depot["row"] * CELL, .002])

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
                            "row": position[1] / CELL})
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
    p = rover["t"].getSFVec3f()
    orientation = rover["node"].getOrientation()
    theta = -math.degrees(math.atan2(orientation[3], orientation[0])) % 360
    return p[0] / CELL, p[1] / CELL, theta

def clock():
    """El reloj es del Arbitro de visión, no del tiempo de Webots."""
    current = referee.reloj()
    return {"elapsed_ms": current.transcurrido_ms, "remaining_ms": current.restante_ms,
            "total_ms": current.total_ms}

def observation():
    """Telemetría que sale de la imagen, nunca de la verdad del Supervisor."""
    global seq, official_completion
    seq += 1
    dump_path = os.environ.get("VRC_CAMERA_DUMP")
    dump_seq = int(os.environ.get("VRC_CAMERA_DUMP_SEQ", "1"))
    if dump_path and seq == dump_seq:
        overhead_camera.saveImage(dump_path, 100)
    image = webots_bgra_to_bgr(overhead_camera.getImage(), overhead_camera.getWidth(),
                                overhead_camera.getHeight())
    current_phase, current_clock = referee.instantanea()
    current_clock = {"elapsed_ms": current_clock.transcurrido_ms,
                     "remaining_ms": current_clock.restante_ms,
                     "total_ms": current_clock.total_ms}
    message = vision_adapter.observe(image, int(time.time() * 1000), current_phase,
                                      current_clock, seq)
    # El árbitro solo avanza tras procesar este cuadro. Así la transición usa
    # geometría actual, no el resultado (posiblemente obsoleto) del cuadro
    # anterior.
    referee.tictac(vision_adapter.last_error is None)
    if vision_adapter.last_error is None:
        state = vision_adapter.last_state
        delivery = counter.actualizar(state, state.ts_ms)
        if referee.observar_reto(delivery.completo, delivery.instante_completo):
            official_completion = True
    return message

while sup.step(STEP) != -1:
    previous_phase = referee.fase
    key = keyboard.getKey()
    command = None
    if key in (ord("R"), ord("r")):
        command = "ready"
    elif key in (ord("I"), ord("i")):
        command = "abort"
    elif key in (ord("F"), ord("f")):
        command = "stop"
    elif key in (ord("S"), ord("s")):
        run_log.write("phase_request_rejected", ts_ms=int(time.time() * 1000),
                      command="start", reason="RUNNING solo lo inicia el Arbitro al vencer READY")
    if command is not None:
        result = referee.intentar(command)
        if command == "ready" and "-> READY" in result:
            run_log.close(status="INCONCLUSIVE", reason="Nueva preparación")
            run_log = RunLog(os.environ.get("VRC_RUN_DIR", "/tmp/vision-rover-webots"), scenario, cfg)
            summary_written = False
            official_completion = False
            critical_failures.clear()
            counter = ContadorAcopio(vision_adapter.cfg)
            vision_adapter = VisionAdapter(root / "vision-system/vision/config_vision.json",
                overhead_camera.getWidth(), overhead_camera.getHeight(), overhead_camera.getFov())
            reset_world()
        run_log.write("phase_request", ts_ms=int(time.time() * 1000), command=command,
                      result=result)
    try:
        while True: clients.append(server.accept()[0]); clients[-1].setblocking(False)
    except BlockingIOError: pass
    if sup.getTime() - last_pub >= 1 / cfg["tasas"]["pub_hz"]:
        last_pub = sup.getTime(); message = observation(); payload = (json.dumps(message, separators=(",", ":")) + "\n").encode()
        truth_state = truth()
        safety = evaluate_safety(truth_state, grid, **scenario["oracle"])
        new_failures = set(safety["failures"]) - critical_failures
        if previous_phase == "RUNNING":
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
        deliveries = deliveries_from_truth()
        verdict = evaluate_verdict(scenario["expected"], deliveries, sorted(critical_failures),
                                   clock()["elapsed_ms"], scenario["physics_calibrated"],
                                   official_completion=official_completion)
        run_log.close(
            **verdict,
            phase=phase,
            clock=clock(),
            deliveries=deliveries,
            official_completion=official_completion,
            safety_failures=sorted(critical_failures),
        )
        summary_written = True

run_log.close(status="INCONCLUSIVE", reason="Webots terminó antes de cerrar la ronda")
