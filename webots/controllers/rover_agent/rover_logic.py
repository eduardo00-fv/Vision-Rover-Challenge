"""Puente ctypes al núcleo C++ compartido por firmware y Webots."""

import ctypes
from pathlib import Path


class Grid(ctypes.Structure):
    _fields_ = [("cols", ctypes.c_ushort), ("rows", ctypes.c_ushort), ("cell_mm", ctypes.c_float)]


class Point(ctypes.Structure):
    _fields_ = [("col", ctypes.c_float), ("row", ctypes.c_float)]


class Pose(ctypes.Structure):
    _fields_ = [("col", ctypes.c_float), ("row", ctypes.c_float), ("theta_deg", ctypes.c_float)]


class Cube(ctypes.Structure):
    _fields_ = [("position", Point), ("age_ms", ctypes.c_uint)]


class DepotSize(ctypes.Structure):
    _fields_ = [("length_cells", ctypes.c_float), ("depth_cells", ctypes.c_float)]


class DriveCommand(ctypes.Structure):
    _fields_ = [("left", ctypes.c_float), ("right", ctypes.c_float), ("active", ctypes.c_ubyte),
                ("pushing", ctypes.c_ubyte), ("mode", ctypes.c_ubyte)]


class TaskRover(ctypes.Structure):
    _fields_ = [("id", ctypes.c_ubyte), ("pose", Pose), ("age_ms", ctypes.c_uint)]


class TaskCube(ctypes.Structure):
    _fields_ = [("color", ctypes.c_ubyte), ("cube", Cube)]


class TaskDepot(ctypes.Structure):
    _fields_ = [("color", ctypes.c_ubyte), ("position", Point)]


_library = ctypes.CDLL(str(Path(__file__).with_name("librover_logic.so")))
_library.vrc_navigate.argtypes = [Grid, DepotSize, ctypes.c_float, Pose, Cube, Point,
                                  ctypes.POINTER(ctypes.c_ubyte)]
_library.vrc_navigate.restype = DriveCommand
_library.vrc_assign.argtypes = [Grid, DepotSize, ctypes.c_float, ctypes.POINTER(TaskRover), ctypes.c_ubyte,
                                ctypes.POINTER(TaskCube), ctypes.c_ubyte, ctypes.POINTER(TaskDepot),
                                ctypes.c_ubyte, ctypes.POINTER(ctypes.c_ubyte)]
_library.vrc_should_yield.argtypes = [Pose, ctypes.c_ubyte, Pose, ctypes.c_ubyte, ctypes.c_uint,
                                      ctypes.c_float, ctypes.c_float]
_library.vrc_should_yield.restype = ctypes.c_ubyte

_COLORS = {"green": 1, "blue": 2, "red": 3}
_NAMES = {value: key for key, value in _COLORS.items()}


def assigned_color(world, rover_id):
    """Asignación global determinista; evita que ambos rovers persigan un cubo."""
    if not world or world.get("phase") != "RUNNING":
        return None
    rovers = world["rovers"][:2]
    cubes = world["cubes"][:3]
    depots = world["depots"][:3]
    if not rovers:
        return None
    rover_array = (TaskRover * len(rovers))(*(TaskRover(item["id"], Pose(item["col"], item["row"], item["theta"]), item["age_ms"]) for item in rovers))
    cube_array = (TaskCube * len(cubes))(*(TaskCube(_COLORS.get(item["color"], 0), Cube(Point(item["col"], item["row"]), item["age_ms"])) for item in cubes))
    depot_array = (TaskDepot * len(depots))(*(TaskDepot(_COLORS.get(item["color"], 0), Point(item["col"], item["row"])) for item in depots))
    assigned = (ctypes.c_ubyte * len(rovers))()
    _library.vrc_assign(Grid(world["grid"]["cols"], world["grid"]["rows"], world["grid"]["cell_mm"]), DepotSize(world["depot_size"]["length"], world["depot_size"]["depth"]), world["cube_side"], rover_array, len(rovers), cube_array, len(cubes), depot_array, len(depots), assigned)
    for index, rover in enumerate(rovers):
        if rover["id"] == rover_id:
            return _NAMES.get(assigned[index])
    return None


def should_yield(world, rover_id, separation_mm=200.0):
    if not world or world.get("phase") != "RUNNING":
        return False
    me = next((item for item in world["rovers"] if item["id"] == rover_id), None)
    if not me:
        return True
    for other in world["rovers"]:
        if other["id"] == rover_id:
            continue
        if _library.vrc_should_yield(Pose(me["col"], me["row"], me["theta"]), rover_id,
                                     Pose(other["col"], other["row"], other["theta"]), other["id"], other["age_ms"],
                                     world["grid"]["cell_mm"], separation_mm):
            return True
    return False


def navigate(world, rover_id, color, pushing):
    """Devuelve rueda izquierda/derecha y el estado de empuje del núcleo C++."""
    if not world or world.get("phase") != "RUNNING":
        return 0.0, 0.0, False
    me = next((item for item in world["rovers"] if item["id"] == rover_id), None)
    cube = next((item for item in world["cubes"] if item["color"] == color), None)
    depot = next((item for item in world["depots"] if item["color"] == color), None)
    if not me or not cube or not depot or me["age_ms"] > 300:
        return 0.0, 0.0, False
    state = ctypes.c_ubyte(bool(pushing))
    command = _library.vrc_navigate(
        Grid(world["grid"]["cols"], world["grid"]["rows"], world["grid"]["cell_mm"]),
        DepotSize(world["depot_size"]["length"], world["depot_size"]["depth"]), world["cube_side"],
        Pose(me["col"], me["row"], me["theta"]), Cube(Point(cube["col"], cube["row"]), cube["age_ms"]),
        Point(depot["col"], depot["row"]), ctypes.byref(state),
    )
    return float(command.left), float(command.right), bool(state.value)
