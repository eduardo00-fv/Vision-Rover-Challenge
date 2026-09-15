"""Oráculo geométrico de seguridad para las trazas Webots.

Opera sobre la verdad física y nunca forma parte de la entrada de un rover.
Representa cada chasis como su rectángulo orientado visto desde arriba; no usa
una distancia entre centros, que ocultaría choques al girar.
"""

import math


def _rectangle(col, row, theta_deg, length_cells, width_cells):
    forward = (math.cos(math.radians(theta_deg)), -math.sin(math.radians(theta_deg)))
    lateral = (-forward[1], forward[0])
    half_length, half_width = length_cells / 2.0, width_cells / 2.0
    return [
        (col + forward[0] * long + lateral[0] * wide,
         row + forward[1] * long + lateral[1] * wide)
        for long, wide in ((half_length, half_width), (half_length, -half_width),
                           (-half_length, -half_width), (-half_length, half_width))
    ]


def _axes(polygon):
    for index, point in enumerate(polygon):
        other = polygon[(index + 1) % len(polygon)]
        dx, dy = other[0] - point[0], other[1] - point[1]
        length = math.hypot(dx, dy)
        yield (-dy / length, dx / length)


def _overlap(first, second):
    """SAT: rectángulos que se tocan cuentan como contacto."""
    for axis in list(_axes(first)) + list(_axes(second)):
        first_projection = [point[0] * axis[0] + point[1] * axis[1] for point in first]
        second_projection = [point[0] * axis[0] + point[1] * axis[1] for point in second]
        if max(first_projection) < min(second_projection) or max(second_projection) < min(first_projection):
            return False
    return True


def _point_segment_distance(point, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    squared = dx * dx + dy * dy
    if squared == 0:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    ratio = max(0.0, min(1.0, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / squared))
    return math.hypot(point[0] - (first[0] + ratio * dx), point[1] - (first[1] + ratio * dy))


def _distance(first, second):
    if _overlap(first, second):
        return 0.0
    distances = []
    for point in first:
        for index, edge_start in enumerate(second):
            distances.append(_point_segment_distance(point, edge_start, second[(index + 1) % len(second)]))
    for point in second:
        for index, edge_start in enumerate(first):
            distances.append(_point_segment_distance(point, edge_start, first[(index + 1) % len(first)]))
    return min(distances)


def evaluate(truth, grid, rover_length_mm, rover_width_mm, minimum_clearance_mm=0.0):
    """Devuelve contactos, proximidades y salidas de cancha de una muestra."""
    cell_mm = grid["cell_mm"]
    length_cells, width_cells = rover_length_mm / cell_mm, rover_width_mm / cell_mm
    polygons = {
        rover["id"]: _rectangle(rover["col"], rover["row"], rover["theta"], length_cells, width_cells)
        for rover in truth["rovers"]
    }
    out_of_bounds = []
    for rover_id, polygon in polygons.items():
        if any(col < 0 or col > grid["cols"] or row < 0 or row > grid["rows"] for col, row in polygon):
            out_of_bounds.append(rover_id)

    contacts, proximity = [], []
    rover_ids = sorted(polygons)
    for index, first_id in enumerate(rover_ids):
        for second_id in rover_ids[index + 1:]:
            distance_mm = _distance(polygons[first_id], polygons[second_id]) * cell_mm
            pair = {"rovers": [first_id, second_id], "distance_mm": distance_mm}
            if distance_mm == 0:
                contacts.append(pair)
            elif distance_mm < minimum_clearance_mm:
                proximity.append(pair)

    failures = (["rover_contact:{}-{}".format(*item["rovers"]) for item in contacts] +
                ["rover_out_of_bounds:{}".format(rover_id) for rover_id in out_of_bounds])
    return {
        "contacts": contacts,
        "proximity": proximity,
        "out_of_bounds": out_of_bounds,
        "failures": failures,
    }
