#include "rover_logic_core.h"

#include <cmath>
#include <initializer_list>

namespace rover_logic {
namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kDeg = kPi / 180.0F;

// Geometría en mm: x = col·cell (derecha), y = row·cell (abajo). theta es
// antihorario como en el contrato: dirección (cos θ, −sin θ).
struct Vec { float x = 0, y = 0; };
Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y}; }
Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y}; }
Vec operator*(Vec a, float k) { return {a.x * k, a.y * k}; }
float dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
float cross(Vec a, Vec b) { return a.x * b.y - a.y * b.x; }
float norm(Vec a) { return std::sqrt(dot(a, a)); }
Vec unit(Vec a) { const float n = norm(a); return n > 1e-6F ? a * (1.0F / n) : Vec{1, 0}; }
Vec rotate(Vec a, float rad) {  // en pantalla (y abajo): positivo = horario
  const float c = std::cos(rad), s = std::sin(rad);
  return {a.x * c - a.y * s, a.x * s + a.y * c};
}
Vec heading(float theta_deg) { return {std::cos(theta_deg * kDeg), -std::sin(theta_deg * kDeg)}; }
float bearing_deg(Vec d) { return std::atan2(-d.y, d.x) / kDeg; }

float clamp(float value) { return value < -1.0F ? -1.0F : (value > 1.0F ? 1.0F : value); }
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float distance(const Point& first, const Point& second) {
  const float dc = second.col - first.col;
  const float dr = second.row - first.row;
  return std::sqrt(dc * dc + dr * dr);
}
float wrap(float degrees) {
  while (degrees > 180.0F) degrees -= 360.0F;
  while (degrees <= -180.0F) degrees += 360.0F;
  return degrees;
}

struct Frame {
  float cell;
  Vec mm(const Point& p) const { return {p.col * cell, p.row * cell}; }
  Point cells(Vec v) const { return {v.x / cell, v.y / cell}; }
};

// Coordenadas del punto en el marco del rover: x adelante, y a la izquierda.
Vec local(const Pose& self, Vec self_mm, Vec p) {
  const Vec f = heading(self.theta_deg);
  const Vec l = {f.y, -f.x};  // θ + 90°: (−sin θ, −cos θ)
  const Vec d = p - self_mm;
  return {dot(d, f), dot(d, l)};
}

bool inside(const Grid& grid, float margin, Vec p) {
  const float w = grid.cols * grid.cell_mm, h = grid.rows * grid.cell_mm;
  return p.x >= margin && p.y >= margin && p.x <= w - margin && p.y <= h - margin;
}
// Distancia al borde lógico más cercano (negativa afuera).
float edgeDistance(const Grid& grid, Vec p) {
  const float w = grid.cols * grid.cell_mm, h = grid.rows * grid.cell_mm;
  return std::fmin(std::fmin(p.x, p.y), std::fmin(w - p.x, h - p.y));
}
// Fin de un tramo recto: lo que avanza a velocidad nominal en el tramo máximo
// (~64 mm) o la distancia pedida si es menor.
constexpr float kMaxLegMm = 70.0F;
Vec legEnd(const Pose& self, Vec s, const DriveCommand& command) {
  const float step = std::fmin(command.distance_mm > 0 ? command.distance_mm : kMaxLegMm, kMaxLegMm);
  return s + heading(self.theta_deg) * (command.reverse ? -step : step);
}
// Un tramo recto que termina fuera del margen y más cerca del borde que ahora.
bool leavesSurface(const Grid& grid, const NavigationConfig& config, const Pose& self, Vec s,
                   const DriveCommand& command) {
  if (!command.active || (!command.reverse && command.left * command.right < 0)) return false;
  const Vec end = legEnd(self, s, command);
  return !inside(grid, config.arena_margin_mm / 2, end) && edgeDistance(grid, end) < edgeDistance(grid, s);
}
// Dentro del cono frontal de ±60°: lo que el rover embestiría al avanzar.
bool ahead(Vec p) { return p.x > 0 && std::fabs(p.y) < p.x * 1.732F; }

Vec clampInside(const Grid& grid, float margin, Vec p) {
  const float w = grid.cols * grid.cell_mm, h = grid.rows * grid.cell_mm;
  return {clampf(p.x, margin, w - margin), clampf(p.y, margin, h - margin)};
}

float segmentDistance(Vec a, Vec b, Vec p) {
  const Vec ab = b - a;
  const float len2 = dot(ab, ab);
  const float t = len2 > 1e-6F ? clampf(dot(p - a, ab) / len2, 0, 1) : 0;
  return norm(a + ab * t - p);
}

// Distancia mínima entre dos segmentos (sin cruce; si se cruzan vale 0).
float segmentsDistance(Vec a, Vec b, Vec c, Vec d) {
  const Vec ab = b - a, cd = d - c;
  const float den = cross(ab, cd);
  if (std::fabs(den) > 1e-6F) {
    const float t = cross(c - a, cd) / den, u = cross(c - a, ab) / den;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0;
  }
  return std::fmin(std::fmin(segmentDistance(a, b, c), segmentDistance(a, b, d)),
                   std::fmin(segmentDistance(c, d, a), segmentDistance(c, d, b)));
}

// Dos empujes chocan si un cubo está sobre el corredor del otro, o si los
// corredores se superponen casi paralelos (mismo pasillo). Cruzarse en ángulo
// no es conflicto: los rovers pasan en momentos distintos y se esquivan.
bool corridorsConflict(Vec a0, Vec a1, Vec b0, Vec b1, float clearance) {
  if (segmentDistance(a0, a1, b0) < clearance * 0.67F || segmentDistance(b0, b1, a0) < clearance * 0.67F)
    return true;
  const float cosine = std::fabs(dot(unit(a1 - a0), unit(b1 - b0)));
  return cosine > 0.87F && segmentsDistance(a0, a1, b0, b1) < clearance;
}

DriveCommand drive_to(const Pose& self, Vec self_mm, Vec target, const NavigationConfig& config,
                      bool pushing, Mode aligned_mode, Mode align_mode, Mode done_mode,
                      bool allow_reverse = true) {
  DriveCommand command;
  command.pushing = pushing;
  const Vec d = target - self_mm;
  const float distance_mm = norm(d);
  if (distance_mm < config.goal_tolerance_mm) {
    command.mode = done_mode;
    return command;
  }
  const float error = wrap(bearing_deg(d) - self.theta_deg);
  command.heading_error_deg = error;
  command.distance_mm = distance_mm;
  command.active = true;
  // Un objetivo corto justo detrás se alcanza retrocediendo, sin girar 180°.
  if (allow_reverse && std::fabs(error) > 160.0F && distance_mm < 250.0F) {
    command.reverse = true;
    command.heading_error_deg = 0;
    command.left = command.right = -config.drive_speed;
    command.mode = Mode::BACKOFF;
    return command;
  }
  if (std::fabs(error) > config.turn_in_place_deg) {
    const float turn = error > 0.0F ? config.turn_speed : -config.turn_speed;
    command.left = -turn;
    command.right = turn;
    command.mode = align_mode;
    return command;
  }
  float speed = pushing ? config.push_speed : config.drive_speed;
  if (distance_mm < config.slowdown_mm) speed *= 0.55F;
  const float correction = clamp(error / config.turn_in_place_deg) * config.steer_gain;
  command.left = clamp(speed - correction);
  command.right = clamp(speed + correction);
  command.mode = aligned_mode;
  return command;
}

// Retroceso recto, acortado a lo que cabe detrás sin salir de la superficie.
DriveCommand backoff(const Grid& grid, const NavigationConfig& config, const Pose& self, Vec self_mm) {
  const Vec behind = heading(self.theta_deg) * -1.0F;
  float room = config.backoff_mm;
  while (room > 0 && !inside(grid, config.arena_margin_mm / 2, self_mm + behind * room)) room -= 10.0F;
  DriveCommand command;
  command.mode = Mode::BACKOFF;
  if (room < 20.0F) return command;  // sin espacio: esperar a otra captura
  command.active = true;
  command.reverse = true;
  command.distance_mm = room;
  command.left = command.right = -config.drive_speed;
  return command;
}

// Reglas de paso entre rovers; true si ya decidió (esperar o retroceder).
bool roverRules(const Grid& grid, const NavigationConfig& config, const Pose& self, Vec s,
                const Obstacle* obstacles, uint8_t count, const Frame& frame, DriveCommand& out) {
  for (uint8_t i = 0; i < count; ++i) {
    if (!obstacles[i].rover) continue;
    const Vec p = local(self, s, frame.mm(obstacles[i].center));
    const float d = norm(p);
    if (obstacles[i].priority && obstacles[i].moving && d < config.rover_yield_mm) {
      if (ahead(p)) { out = backoff(grid, config, self, s); if (out.active) return true; }
      out = {}; out.mode = Mode::YIELD; return true;
    }
    if (ahead(p) && d < config.rover_stop_mm) { out = {}; out.mode = Mode::YIELD; return true; }
  }
  return false;
}

// Chasis del CenfoBot en su marco (x adelante): caja 120×100 + paletas a 77 mm.
constexpr float kRear = 60.0F, kFront = 77.0F, kHalfWidth = 50.0F;
constexpr float kContactMargin = 15.0F;

bool chassisOverlap(Vec a, float a_deg, Vec b, float b_deg, float margin) {
  Vec corners[2][4];
  const Vec centers[2] = {a, b};
  const float angles[2] = {a_deg, b_deg};
  for (int k = 0; k < 2; ++k) {
    const Vec f = heading(angles[k]), l = {f.y, -f.x};
    const float xs[2] = {kFront + margin, -kRear - margin}, ys[2] = {kHalfWidth + margin, -kHalfWidth - margin};
    for (int i = 0; i < 4; ++i) corners[k][i] = centers[k] + f * xs[i / 2] + l * ys[(i / 2) ^ (i % 2)];
  }
  for (int k = 0; k < 2; ++k) {  // ejes separadores: los de cada caja
    const Vec f = heading(angles[k]);
    for (const Vec axis : {f, Vec{f.y, -f.x}}) {
      float min0 = 1e9F, max0 = -1e9F, min1 = 1e9F, max1 = -1e9F;
      for (const Vec& p : corners[0]) { const float v = dot(p, axis); min0 = std::fmin(min0, v); max0 = std::fmax(max0, v); }
      for (const Vec& p : corners[1]) { const float v = dot(p, axis); min1 = std::fmin(min1, v); max1 = std::fmax(max1, v); }
      if (max0 < min1 || max1 < min0) return false;
    }
  }
  return true;
}

// Guarda final: predice el chasis tras el tramo (o todo el barrido del giro)
// y no deja salir una orden que toque a otro rover. Independiente de las
// heurísticas de paso: si alguna falla, esta no.
DriveCommand guardContact(const Pose& self, Vec s, const Obstacle* obstacles, uint8_t count,
                          const Frame& frame, DriveCommand command) {
  if (!command.active) return command;
  for (uint8_t i = 0; i < count; ++i) {
    if (!obstacles[i].rover) continue;
    const Vec o = frame.mm(obstacles[i].center);
    const float o_deg = obstacles[i].theta_deg;
    bool hit = false;
    const bool turning = !command.reverse && command.left * command.right < 0;
    if (turning) {
      const float sweep = command.heading_error_deg;
      for (int k = 1; k <= 8 && !hit; ++k)
        hit = chassisOverlap(s, self.theta_deg + sweep * k / 8.0F, o, o_deg, kContactMargin);
    } else {
      const float step = std::fmin(command.distance_mm > 0 ? command.distance_mm : 25.0F, 25.0F);
      const Vec end = s + heading(self.theta_deg) * (command.reverse ? -step : step);
      // Si ya se tocan, solo vale alejarse: comparar distancias.
      const bool now = chassisOverlap(s, self.theta_deg, o, o_deg, kContactMargin);
      hit = chassisOverlap(end, self.theta_deg, o, o_deg, kContactMargin) &&
            (!now || norm(end - o) <= norm(s - o));
    }
    if (hit) { DriveCommand hold; hold.mode = Mode::YIELD; return hold; }
  }
  return command;
}

// Con otro rover o un cubo dentro del barrido de giro, no girar en sitio: separarse
// primero en recta (atrás si está delante, adelante si está detrás).
DriveCommand separateBeforeTurning(const Grid& grid, const NavigationConfig& config, const Pose& self,
                                   Vec s, const Obstacle* obstacles, uint8_t count, const Frame& frame,
                                   DriveCommand command) {
  const bool turning = command.active && !command.reverse && command.left * command.right < 0;
  if (!turning) return command;
  for (uint8_t i = 0; i < count; ++i) {
    // El barrido de las esquinas (~92 mm) también arrastra cubos: uno ya
    // entregado al costado saldría de su zona.
    const float clearance = obstacles[i].rover ? config.rover_turn_clearance_mm : config.cube_turn_clearance_mm;
    const Vec p = local(self, s, frame.mm(obstacles[i].center));
    if (norm(p) >= clearance) continue;
    if (p.x > 0) {
      const DriveCommand back = backoff(grid, config, self, s);
      if (back.active) return back;
    } else {
      const Vec forward_goal = s + heading(self.theta_deg) * config.backoff_mm;
      if (inside(grid, config.arena_margin_mm, forward_goal)) {
        DriveCommand ahead_cmd = command;
        ahead_cmd.left = ahead_cmd.right = config.drive_speed;
        ahead_cmd.heading_error_deg = 0;
        ahead_cmd.distance_mm = config.backoff_mm;
        ahead_cmd.mode = Mode::AVOID;
        return ahead_cmd;
      }
    }
    DriveCommand hold;
    hold.mode = Mode::YIELD;
    return hold;
  }
  return command;
}

// Camino más corto por la grilla de la cancha (A*, 8 vecinos) evitando los
// obstáculos inflados; devuelve el punto más lejano del camino visible en
// línea recta. Sin camino, o con el destino a la vista, va directo (false).
// Búferes estáticos: ~30 KB, no en la pila del loop del ESP32.
constexpr int kMaxCells = 64 * 64;
float g_cost[kMaxCells];
int16_t g_parent[kMaxCells];
uint8_t g_state[kMaxCells];  // 0 nuevo, 1 abierto, 2 cerrado
int16_t g_heap[kMaxCells];
int16_t g_path[kMaxCells];

bool blockedAt(const Grid& grid, const NavigationConfig& config, Vec p, const Obstacle* obstacles,
               uint8_t count, const Frame& frame) {
  if (!inside(grid, config.arena_margin_mm, p)) return true;
  for (uint8_t i = 0; i < count; ++i)
    if (norm(p - frame.mm(obstacles[i].center)) < obstacles[i].radius_mm) return true;
  return false;
}

// Tramo libre si ninguna muestra cada 10 mm cae dentro de un obstáculo. Si
// ya se parte desde adentro, vale mientras se aleje de él.
bool clearSegment(const Grid& grid, const NavigationConfig& config, Vec a, Vec b,
                  const Obstacle* obstacles, uint8_t count, const Frame& frame) {
  const float length = norm(b - a);
  const bool start_blocked = blockedAt(grid, config, a, obstacles, count, frame);
  const int steps = static_cast<int>(length / 10.0F) + 1;
  for (int k = 1; k <= steps; ++k) {
    const Vec p = a + (b - a) * (static_cast<float>(k) / steps);
    if (!blockedAt(grid, config, p, obstacles, count, frame)) continue;
    if (!start_blocked) return false;
    for (uint8_t i = 0; i < count; ++i) {  // desde adentro: no acercarse a ninguno
      const Vec o = frame.mm(obstacles[i].center);
      if (norm(p - o) < obstacles[i].radius_mm && norm(p - o) < norm(a - o) - 1.0F) return false;
    }
  }
  return true;
}

bool detour(const Grid& grid, const NavigationConfig& config, Vec from, Vec goal,
            const Obstacle* obstacles, uint8_t count, const Frame& frame, Vec& waypoint) {
  if (clearSegment(grid, config, from, goal, obstacles, count, frame)) return false;
  const int cols = grid.cols, rows = grid.rows, total = cols * rows;
  if (total <= 0 || total > kMaxCells) return false;
  const float cell = grid.cell_mm;
  auto index = [&](Vec p) {
    const int i = static_cast<int>(clampf(p.x / cell, 0, cols - 1)), j = static_cast<int>(clampf(p.y / cell, 0, rows - 1));
    return j * cols + i;
  };
  auto center = [&](int n) { return Vec{(n % cols + 0.5F) * cell, (n / cols + 0.5F) * cell}; };
  const int start = index(from), target = index(goal);
  for (int n = 0; n < total; ++n) { g_state[n] = 0; g_cost[n] = 1e30F; }
  auto heuristic = [&](int n) { return norm(center(n) - center(target)); };
  auto priority = [&](int n) { return g_cost[n] + heuristic(n); };
  int heap_size = 0;
  auto push = [&](int n) {
    int k = heap_size++;
    g_heap[k] = static_cast<int16_t>(n);
    while (k > 0 && priority(g_heap[(k - 1) / 2]) > priority(g_heap[k])) {
      const int16_t t = g_heap[k]; g_heap[k] = g_heap[(k - 1) / 2]; g_heap[(k - 1) / 2] = t; k = (k - 1) / 2;
    }
  };
  auto pop = [&]() {
    const int top = g_heap[0];
    g_heap[0] = g_heap[--heap_size];
    for (int k = 0;;) {
      int m = k; const int l = 2 * k + 1, r = l + 1;
      if (l < heap_size && priority(g_heap[l]) < priority(g_heap[m])) m = l;
      if (r < heap_size && priority(g_heap[r]) < priority(g_heap[m])) m = r;
      if (m == k) break;
      const int16_t t = g_heap[k]; g_heap[k] = g_heap[m]; g_heap[m] = t; k = m;
    }
    return top;
  };
  g_cost[start] = 0; g_parent[start] = -1; g_state[start] = 1; push(start);
  bool found = false;
  while (heap_size > 0) {
    const int n = pop();
    if (g_state[n] == 2) continue;
    g_state[n] = 2;
    if (n == target) { found = true; break; }
    const int i = n % cols, j = n / cols;
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        if (!di && !dj) continue;
        const int ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
        const int m = nj * cols + ni;
        if (g_state[m] == 2) continue;
        // Atravesar una celda bloqueada cuesta mucho, no es imposible: así
        // un rover que empieza dentro de un margen encuentra la salida corta.
        const float step = (di && dj ? 1.41421F : 1.0F) * cell *
            (blockedAt(grid, config, center(m), obstacles, count, frame) ? 25.0F : 1.0F);
        if (g_cost[n] + step < g_cost[m]) {
          g_cost[m] = g_cost[n] + step; g_parent[m] = static_cast<int16_t>(n);
          g_state[m] = 1; push(m);
        }
      }
    }
  }
  if (!found) return false;
  int length = 0;
  for (int n = target; n != -1 && length < kMaxCells; n = g_parent[n]) g_path[length++] = static_cast<int16_t>(n);
  // g_path va del destino al inicio: el primero visible desde el rover es el más lejano.
  const float min_step = config.goal_tolerance_mm + 10.0F;
  for (int k = 0; k < length; ++k) {
    const Vec p = k == 0 ? goal : center(g_path[k]);
    if (norm(p - from) > 350.0F || norm(p - from) < min_step) continue;
    if (clearSegment(grid, config, from, p, obstacles, count, frame)) { waypoint = p; return true; }
  }
  // Nada visible: seguir el camino desde el nodo que ya supera la tolerancia.
  for (int k = length - 1; k >= 0; --k) {
    const Vec p = center(g_path[k]);
    if (norm(p - from) >= min_step) { waypoint = p; return true; }
  }
  return false;
}

// Dirección de empuje más cercana a la del depósito cuyos puntos de
// alineación y de contacto caben en la cancha y no quedan encima del otro
// rover. Un cubo contra el borde se despega en diagonal antes de ir derecho;
// un rover detenido junto al cubo obliga a atacarlo desde otro ángulo.
bool pushDirection(const Grid& grid, const NavigationConfig& config, Vec cube, Vec depot,
                   const Obstacle* obstacles, uint8_t count, const Frame& frame, Vec& u) {
  const Vec direct = unit(depot - cube);
  for (int k = 0; k <= 16; ++k) {
    for (int sign : {1, -1}) {
      if (k == 0 && sign < 0) continue;
      const Vec candidate = rotate(direct, sign * k * 5.0F * kDeg);
      const Vec align = cube - candidate * config.approach_standoff_mm;
      const Vec contact = cube - candidate * config.push_contact_offset_mm;
      bool ok = inside(grid, config.arena_margin_mm, align) && inside(grid, config.arena_margin_mm, contact);
      for (uint8_t i = 0; ok && i < count; ++i) {
        if (!obstacles[i].rover) continue;
        const Vec o = frame.mm(obstacles[i].center);
        ok = norm(align - o) >= obstacles[i].radius_mm && norm(contact - o) >= obstacles[i].radius_mm;
      }
      if (ok) { u = candidate; return true; }
    }
  }
  return false;
}
// Normal hacia adentro de la cancha del borde donde apoya el depósito (misma
// deducción que el contrato: el borde más cercano a su centro).
Vec depotNormal(const Grid& grid, const Point& depot) {
  const float w = grid.cols, h = grid.rows;
  const float left = depot.col, right = w - depot.col, top = depot.row, bottom = h - depot.row;
  const float nearest = std::fmin(std::fmin(left, right), std::fmin(top, bottom));
  if (nearest == top) return {0, 1};
  if (nearest == bottom) return {0, -1};
  if (nearest == left) return {1, 0};
  return {-1, 0};
}

// Adónde llevar el cubo ahora: si viene desalineado, primero a un punto frente
// a la zona; después, recto por la normal. La ventana de fondo (±32 mm) es la
// estrecha, así que el error lateral debe quedar a lo largo de la zona.
// Otro cubo en el trayecto del empujado se rodea por un punto lateral.
Vec cubeGoal(const Grid& grid, const NavigationConfig& config, Vec cube, Vec depot,
             const Point& depot_cells, const Obstacle* obstacles, uint8_t count, const Frame& frame) {
  const Vec n = depotNormal(grid, depot_cells);
  const Vec t = {-n.y, n.x};
  const float out = dot(cube - depot, n), lateral = dot(cube - depot, t);
  // Apuntar 8 mm antes del centro: pasarse hacia la pared no tiene arreglo.
  Vec goal = (out > 90.0F && std::fabs(lateral) > 25.0F) ? depot + n * 130.0F : depot + n * 8.0F;
  const Vec dir = unit(goal - cube);
  const Vec left = {dir.y, -dir.x};
  for (uint8_t i = 0; i < count; ++i) {
    if (obstacles[i].rover) continue;
    const Vec o = frame.mm(obstacles[i].center);
    const float along = dot(o - cube, dir);
    if (along <= 0 || along > norm(goal - cube) || segmentDistance(cube, goal, o) > 75.0F) continue;
    // Pasar del lado contrario al que ocupa el obstáculo respecto de la línea.
    const float side = dot(o - cube, left) >= 0 ? -1.0F : 1.0F;
    for (float s : {side, -side}) {
      const Vec w = o + left * (s * 150.0F);
      if (inside(grid, config.arena_margin_mm + 40.0F, w)) return w;
    }
  }
  return goal;
}
}  // namespace

bool cube_settled(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                  const Point& cube, const Point& depot, float margin_mm) {
  if (grid.cols == 0 || grid.rows == 0 || grid.cell_mm <= 0.0F || depot_size.length_cells <= 0.0F ||
      depot_size.depth_cells <= 0.0F || cube_side_cells <= 0.0F) return false;
  const float nearest = std::fmin(std::fmin(depot.col, static_cast<float>(grid.cols) - depot.col),
                                  std::fmin(depot.row, static_cast<float>(grid.rows) - depot.row));
  const bool horizontal = nearest == depot.row || nearest == static_cast<float>(grid.rows) - depot.row;
  const float semi_col = (horizontal ? depot_size.length_cells : depot_size.depth_cells) / 2.0F;
  const float semi_row = (horizontal ? depot_size.depth_cells : depot_size.length_cells) / 2.0F;
  const float margin = cube_side_cells * std::sqrt(2.0F) / 2.0F + margin_mm / grid.cell_mm;
  return std::fabs(cube.col - depot.col) <= semi_col - margin &&
         std::fabs(cube.row - depot.row) <= semi_row - margin;
}

bool cube_delivered(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                    const Point& cube, const Point& depot) {
  return cube_settled(grid, depot_size, cube_side_cells, cube, depot, 0.0F);
}

bool cube_done(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
               const Point& cube, const Point& depot, float margin_mm) {
  if (cube_settled(grid, depot_size, cube_side_cells, cube, depot, margin_mm)) return true;
  if (!cube_delivered(grid, depot_size, cube_side_cells, cube, depot)) return false;
  // Dentro y pasado del centro hacia la pared: empujarlo más solo lo saca.
  const Vec n = depotNormal(grid, depot);
  const float out_mm = ((cube.col - depot.col) * n.x + (cube.row - depot.row) * n.y) * grid.cell_mm;
  return out_mm < 5.0F;
}

namespace {
DriveCommand goToUnguarded(const Grid& grid, const NavigationConfig& config, const Pose& self,
                           const Point& goal, const Obstacle* obstacles, uint8_t obstacle_count, Mode mode) {
  if (grid.cell_mm <= 0.0F) return {};
  const Frame frame{grid.cell_mm};
  const Vec s = frame.mm({self.col, self.row});
  const Vec g = clampInside(grid, config.arena_margin_mm, frame.mm(goal));
  DriveCommand rule;
  if (roverRules(grid, config, self, s, obstacles, obstacle_count, frame, rule)) return rule;
  Vec w;
  const bool around = detour(grid, config, s, g, obstacles, obstacle_count, frame, w);
  DriveCommand command = drive_to(self, s, around ? w : g, config, false,
                                  around ? Mode::AVOID : mode, around ? Mode::AVOID : mode, Mode::WAIT_TARGET);
  if (!command.active) command.mode = around ? Mode::AVOID : mode;
  return separateBeforeTurning(grid, config, self, s, obstacles, obstacle_count, frame, command);
}

DriveCommand turnInPlace(const NavigationConfig& config, float error_deg, bool pushing, Mode mode) {
  DriveCommand command;
  command.active = true;
  command.pushing = pushing;
  command.mode = mode;
  command.heading_error_deg = error_deg;
  const float turn = error_deg > 0.0F ? config.turn_speed : -config.turn_speed;
  command.left = -turn;
  command.right = turn;
  return command;
}

DriveCommand navigateUnguarded(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                               const NavigationConfig& config, const Pose& self, const Cube& cube,
                               const Point& depot, const Obstacle* obstacles, uint8_t obstacle_count,
                               PushState* st) {
  const Frame frame{grid.cell_mm};
  const Vec s = frame.mm({self.col, self.row});
  const Vec c = frame.mm(cube.position);
  const Vec d = frame.mm(depot);
  const uint32_t max_age = st->stage == PushState::PUSH ? config.max_push_cube_age_ms : config.max_cube_age_ms;
  if (cube.age_ms > max_age) {
    if (st->stage != PushState::RETREAT) st->stage = PushState::GOTO;
    return {};
  }
  if (cube_done(grid, depot_size, cube_side_cells, cube.position, depot, config.settle_margin_mm)) {
    st->stage = PushState::GOTO;
    DriveCommand done;
    done.mode = Mode::CUBE_DELIVERED;
    return done;
  }
  // Todo retroceso queda comprometido (~80 mm): si se reevalúa con cada
  // mensaje, el rover avanza y retrocede milímetros frente a un cubo.
  auto retreat = [&]() {
    st->stage = PushState::RETREAT;
    st->retreat_x = s.x; st->retreat_y = s.y;
    return backoff(grid, config, self, s);
  };
  auto commit = [&](DriveCommand command) {
    if (command.active && command.reverse && st->stage != PushState::RETREAT) {
      st->stage = PushState::RETREAT;
      st->retreat_x = s.x; st->retreat_y = s.y;
    }
    return command;
  };
  if (st->stage == PushState::RETREAT) {
    const float moved = norm(s - Vec{st->retreat_x, st->retreat_y});
    if (moved < config.backoff_mm - 15.0F) {
      const DriveCommand back = backoff(grid, config, self, s);
      if (back.active) return back;
    }
    st->stage = PushState::GOTO;
  }
  // Algo pegado al frente que no es el objetivo (un cubo ya entregado):
  // retroceder antes de girar para no arrastrarlo.
  for (uint8_t i = 0; i < obstacle_count; ++i) {
    if (obstacles[i].rover) continue;
    const Vec p = local(self, s, frame.mm(obstacles[i].center));
    // Solo contacto real (paleta contra el cubo): con más holgura el ruido de
    // la visión alterna avanzar/retroceder frente a un cubo cercano.
    if (p.x > 0 && p.x < config.push_contact_offset_mm + 10.0F && std::fabs(p.y) < 45.0F) {
      return retreat();
    }
  }
  DriveCommand rule;
  if (roverRules(grid, config, self, s, obstacles, obstacle_count, frame, rule)) return commit(rule);

  const Vec goal = cubeGoal(grid, config, c, d, depot, obstacles, obstacle_count, frame);
  const Vec frozen = {st->ux, st->uy};

  if (st->stage == PushState::PUSH) {
    // Tramo congelado al alinear: el ruido no alterna preparación y entrada recta.
    const Vec leg_goal = {st->gx, st->gy};
    const Vec desired = unit(leg_goal - c);
    const Vec cube_local = local(self, s, c);
    const float rover_cube = norm(c - s);
    const bool touching = rover_cube < config.push_contact_offset_mm + 20.0F;
    const bool lost = rover_cube > config.push_contact_max_mm ||
                      (touching && (cube_local.x < 0 || std::fabs(cube_local.y) > config.push_lateral_abort_mm));
    const bool turned = std::fabs(wrap(bearing_deg(desired) - bearing_deg(frozen))) > config.push_angle_abort_deg;
    const bool misaligned = touching &&
        std::fabs(wrap(bearing_deg(desired) - self.theta_deg)) > config.push_angle_abort_deg;
    const float remaining = dot(leg_goal - c, desired);
    // Llegó al punto de preparación, resbaló o se pasó: retirarse y volver a
    // alinear para el tramo siguiente desde el lado correcto.
    const bool leg_done = st->staging && remaining < 20.0F;
    if (lost || turned || misaligned || leg_done || remaining < -config.goal_tolerance_mm) return retreat();
    DriveCommand command = drive_to(self, s, c + desired * config.push_aim_mm, config, true,
                                    Mode::PUSH, Mode::PUSH_ALIGN, Mode::PUSH_DONE, false);
    command.distance_mm = remaining > 0 ? remaining : 0;
    // Un cubo que resbala por la pared arrastra al rover fuera de la superficie.
    if (leavesSurface(grid, config, self, s, command)) return retreat();
    return command;
  }

  if (st->stage == PushState::ALIGN) {
    const Vec approach = c - frozen * config.approach_standoff_mm;
    if (norm(approach - s) > config.approach_tolerance_mm + 25.0F) {
      st->stage = PushState::GOTO;  // el cubo o el rover se movieron: volver a ir
    } else {
      const float error = wrap(bearing_deg(frozen) - self.theta_deg);
      if (std::fabs(error) <= 8.0F) {
        st->stage = PushState::PUSH;
        return navigateUnguarded(grid, depot_size, cube_side_cells, config, self, cube, depot,
                                 obstacles, obstacle_count, st);
      }
      return commit(separateBeforeTurning(grid, config, self, s, obstacles, obstacle_count, frame,
                                          turnInPlace(config, error, true, Mode::PUSH_ALIGN)));
    }
  }

  // GOTO: camino al punto de alineación de la dirección factible más directa.
  const Vec center = {grid.cols * grid.cell_mm / 2, grid.rows * grid.cell_mm / 2};
  Vec u;
  // Si no hay forma de llevarlo a la zona (pegado al borde), sacarlo hacia el centro.
  if (!pushDirection(grid, config, c, goal, obstacles, obstacle_count, frame, u) &&
      !pushDirection(grid, config, c, center, obstacles, obstacle_count, frame, u)) {
    DriveCommand none;
    none.mode = Mode::UNREACHABLE;
    return none;
  }
  const Vec approach = c - u * config.approach_standoff_mm;
  if (norm(approach - s) < config.approach_tolerance_mm) {
    st->stage = PushState::ALIGN;
    st->ux = u.x; st->uy = u.y;
    // Destino del tramo sobre la dirección factible (puede diferir del ideal
    // si el cubo se despega de un borde).
    const float reach = std::fmax(dot(goal - c, u), 0.0F);
    const Vec leg = c + u * reach;
    st->gx = leg.x; st->gy = leg.y;
    st->staging = norm(goal - d) > 60.0F || norm(leg - goal) > 30.0F;
    return navigateUnguarded(grid, depot_size, cube_side_cells, config, self, cube, depot,
                             obstacles, obstacle_count, st);
  }
  // El propio cubo también es un obstáculo hasta alinearse.
  Obstacle all[12];
  uint8_t n = 0;
  all[n++] = {cube.position, config.cube_clearance_mm, false};
  for (uint8_t i = 0; i < obstacle_count && n < 12; ++i) all[n++] = obstacles[i];
  Vec w;
  const bool around = detour(grid, config, s, approach, all, n, frame, w);
  const DriveCommand command = around
      ? drive_to(self, s, w, config, false, Mode::AVOID, Mode::AVOID, Mode::AVOID)
      : drive_to(self, s, approach, config, false, Mode::APPROACH, Mode::APPROACH_ALIGN, Mode::AT_APPROACH);
  return commit(separateBeforeTurning(grid, config, self, s, obstacles, obstacle_count, frame, command));
}

// Guarda final de superficie: ninguna orden recta saca el centro del margen.
DriveCommand guardSurface(const Grid& grid, const NavigationConfig& config, const Pose& self, Vec s,
                          DriveCommand command) {
  if (!leavesSurface(grid, config, self, s, command)) return command;
  DriveCommand hold;
  hold.mode = Mode::YIELD;
  return hold;
}

}  // namespace

DriveCommand go_to(const Grid& grid, const NavigationConfig& config, const Pose& self,
                   const Point& goal, const Obstacle* obstacles, uint8_t obstacle_count, Mode mode) {
  if (grid.cell_mm <= 0.0F) return {};
  const Frame frame{grid.cell_mm};
  const Vec s = frame.mm({self.col, self.row});
  return guardSurface(grid, config, self, s, guardContact(self, s, obstacles, obstacle_count, frame,
                      goToUnguarded(grid, config, self, goal, obstacles, obstacle_count, mode)));
}

DriveCommand navigate(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                      const NavigationConfig& config, const Pose& self, const Cube& cube,
                      const Point& depot, const Obstacle* obstacles, uint8_t obstacle_count,
                      PushState* state) {
  if (state == nullptr || grid.cell_mm <= 0.0F) return {};
  const Frame frame{grid.cell_mm};
  const Vec s = frame.mm({self.col, self.row});
  DriveCommand command = guardSurface(grid, config, self, s, guardContact(self, s, obstacles, obstacle_count, frame,
                                      navigateUnguarded(grid, depot_size, cube_side_cells, config, self, cube,
                                                        depot, obstacles, obstacle_count, state)));
  command.pushing = state->stage == PushState::PUSH;
  return command;
}

// Interfaz anterior (Webots): `pushing` solo distingue empuje de aproximación.
DriveCommand navigate(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                      const NavigationConfig& config, const Pose& self, const Cube& cube,
                      const Point& depot, bool* pushing) {
  if (pushing == nullptr) return {};
  PushState state;
  if (*pushing) {
    state.stage = PushState::PUSH;
    const Vec u = unit(Vec{depot.col - cube.position.col, depot.row - cube.position.row});
    state.ux = u.x; state.uy = u.y;
  }
  const DriveCommand command = navigate(grid, depot_size, cube_side_cells, config, self, cube, depot,
                                        nullptr, 0, &state);
  *pushing = state.stage == PushState::PUSH || state.stage == PushState::ALIGN;
  return command;
}

Point parking_spot(const Grid& grid, const Point* keep_clear, uint8_t count) {
  const float inset = 5.0F;
  const Point corners[4] = {{inset, inset}, {grid.cols - inset, inset},
                            {inset, grid.rows - inset}, {grid.cols - inset, grid.rows - inset}};
  Point best = corners[0];
  float best_score = -1.0F;
  for (const Point& corner : corners) {
    float score = 1e9F;
    for (uint8_t i = 0; i < count; ++i) score = std::fmin(score, distance(corner, keep_clear[i]));
    if (score > best_score) { best_score = score; best = corner; }
  }
  return best;
}

bool rover_pushing(const Pose& rover, const Point* cubes, uint8_t count, float cell_mm,
                   float contact_mm) {
  if (cell_mm <= 0.0F) return false;
  const Frame frame{cell_mm};
  const Vec s = frame.mm({rover.col, rover.row});
  for (uint8_t i = 0; i < count; ++i) {
    const Vec p = local(rover, s, frame.mm(cubes[i]));
    if (p.x > contact_mm - 25.0F && p.x < contact_mm + 15.0F && std::fabs(p.y) < 40.0F) return true;
  }
  return false;
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::CUBE_DELIVERED: return "CUBE_DELIVERED";
    case Mode::APPROACH_ALIGN: return "APPROACH_ALIGN";
    case Mode::APPROACH: return "APPROACH";
    case Mode::AT_APPROACH: return "AT_APPROACH";
    case Mode::PUSH_ALIGN: return "PUSH_ALIGN";
    case Mode::PUSH: return "PUSH";
    case Mode::PUSH_DONE: return "PUSH_DONE";
    case Mode::AVOID: return "AVOID";
    case Mode::BACKOFF: return "BACKOFF";
    case Mode::PARK: return "PARK";
    case Mode::UNREACHABLE: return "UNREACHABLE";
    case Mode::YIELD: return "YIELD";
    default: return "WAIT_TARGET";
  }
}

void plan_assignment(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                     const TaskRover* rovers, uint8_t rover_count, const TaskCube* cubes,
                     uint8_t cube_count, const TaskDepot* depots, uint8_t depot_count,
                     const Color* previous, const AssignConfig& config, Color* assigned) {
  if (assigned == nullptr) return;
  for (uint8_t r = 0; r < rover_count; ++r) assigned[r] = Color::NONE;
  if (rover_count > 2 || cube_count > 3) return;
  // Costo en mm de cada par rover→cubo→depósito; negativo = no disponible.
  float cost[2][3];
  for (uint8_t r = 0; r < rover_count; ++r) {
    for (uint8_t c = 0; c < cube_count; ++c) {
      cost[r][c] = -1.0F;
      if (rovers[r].age_ms > config.max_rover_age_ms || cubes[c].color == Color::NONE ||
          cubes[c].cube.age_ms > config.max_cube_age_ms) continue;
      const TaskDepot* depot = nullptr;
      for (uint8_t d = 0; d < depot_count; ++d) if (depots[d].color == cubes[c].color) depot = &depots[d];
      if (depot == nullptr || cube_done(grid, depot_size, cube_side_cells, cubes[c].cube.position,
                                        depot->position, config.settle_margin_mm)) continue;
      float mm = (distance({rovers[r].pose.col, rovers[r].pose.row}, cubes[c].cube.position) +
                  distance(cubes[c].cube.position, depot->position)) * grid.cell_mm;
      if (previous != nullptr && previous[r] == cubes[c].color) mm -= config.hysteresis_mm;
      cost[r][c] = mm < 0 ? 0 : mm;
    }
  }
  // Corredor de empuje de cada cubo (cubo → depósito), en mm.
  Vec from[3], to[3];
  for (uint8_t c = 0; c < cube_count; ++c) {
    from[c] = {cubes[c].cube.position.col * grid.cell_mm, cubes[c].cube.position.row * grid.cell_mm};
    to[c] = from[c];
    for (uint8_t d = 0; d < depot_count; ++d)
      if (depots[d].color == cubes[c].color)
        to[c] = {depots[d].position.col * grid.cell_mm, depots[d].position.row * grid.cell_mm};
  }
  // Búsqueda exhaustiva: a lo sumo 2 rovers × 3 cubos. -1 = sin tarea. Dos
  // empujes cuyos corredores se cruzan no van a la vez: uno espera.
  int best[2] = {-1, -1};
  int best_count = 0;
  float best_cost = 0;
  for (int a = -1; a < cube_count; ++a) {
    for (int b = -1; b < (rover_count > 1 ? cube_count : 0); ++b) {
      if (a >= 0 && b >= 0 && a == b) continue;
      if ((a >= 0 && cost[0][a] < 0) || (b >= 0 && cost[1][b] < 0)) continue;
      if (a >= 0 && b >= 0 && corridorsConflict(from[a], to[a], from[b], to[b], config.corridor_clearance_mm))
        continue;
      const int count = (a >= 0) + (b >= 0);
      const float total = (a >= 0 ? cost[0][a] : 0) + (b >= 0 ? cost[1][b] : 0);
      if (count > best_count || (count == best_count && count > 0 && total < best_cost)) {
        best[0] = a; best[1] = b; best_count = count; best_cost = total;
      }
    }
  }
  for (uint8_t r = 0; r < rover_count; ++r)
    if (best[r] >= 0) assigned[r] = cubes[best[r]].color;
}

void assign_tasks(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                  const TaskRover* rovers, uint8_t rover_count, const TaskCube* cubes,
                  uint8_t cube_count, const TaskDepot* depots, uint8_t depot_count,
                  uint32_t max_rover_age_ms, uint32_t max_cube_age_ms, Color* assigned) {
  AssignConfig config;
  config.max_rover_age_ms = max_rover_age_ms;
  config.max_cube_age_ms = max_cube_age_ms;
  plan_assignment(grid, depot_size, cube_side_cells, rovers, rover_count, cubes, cube_count,
                  depots, depot_count, nullptr, config, assigned);
}

bool should_yield(const Pose& self, uint8_t self_id, const Pose& other, uint8_t other_id,
                  uint32_t other_age_ms, float cell_mm, float separation_mm,
                  uint32_t max_pose_age_ms) {
  if (other_age_ms > max_pose_age_ms || cell_mm <= 0.0F || self_id <= other_id) return false;
  return distance({self.col, self.row}, {other.col, other.row}) * cell_mm < separation_mm;
}
}  // namespace rover_logic
