// Simulador 2D de ronda: ejecuta el firmware real de cada rover (fw_10.so,
// fw_11.so) contra una cancha cinemática, una visión con latencia/ruido y un
// árbitro independiente del código bajo prueba. Unidades internas: mm, s, rad.
// Marco: x = col·cell (derecha), y = row·cell (abajo), theta antihorario como
// en el contrato v2: dirección (cos θ, −sin θ).
#include <dlfcn.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "firmware_api.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kSqrt2 = 1.41421356237309504880;

// ---------------------------------------------------------------- parámetros
// Fuente: rover_control/diagnostico/SESION_2026-09-19.md (rover 1, sin carga).
//  - 20/20 durante 3,5 s ≈ 20–25 cm con curva a la derecha → ~64 mm/s.
//  - 15/15 arranca; 18/20 todavía curva levemente a la derecha.
//  - Giro en sitio ±18/20 no arranca; ±25 gira ~46–62 °/s; residual 2–3°.
// Lo no medido (empuje, fricción del cubo, latencias) queda marcado SUPUESTO.
struct Physics {
  double k_mm_s = 427;          // mm/s por unidad de PWM sobre la zona muerta
  double deadband = 0.05;       // avance
  double pivot_static = 0.20;   // giro en sitio: no arranca por debajo
  double pivot_kinetic = 0.16;  // giro en sitio ya en marcha
  double gain_left = 1.00;
  double gain_right = 0.85;     // curva derecha a 20/20
  double tau_s = 0.06;          // residual ~2–3° al cortar a ~50 °/s
  double track_mm = 88;         // Webots: ruedas a ±44 mm
  double push_factor = 0.85;    // SUPUESTO: pérdida de velocidad empujando
  double cube_mu = 0.3;         // SUPUESTO: arrastre tangencial del cubo
  double gyro_bias_dps = 0.42;  // GZERO medido 0,40–0,43 °/s
  double gyro_noise_dps = 0.08; // spread 0,15–0,17 °/s en 200 muestras
  double imu_hz = 104;          // ODR configurado en lsm6ds3_gyro.h
};

struct Vision {
  double cam_fps = 30;
  double pub_hz = 20;             // config_vision.json
  double proc_ms = 35;            // SUPUESTO: captura → estado listo
  double net_min_ms = 3, net_max_ms = 25;  // SUPUESTO: Wi-Fi local
  double pos_noise_mm = 1.5;      // SUPUESTO
  double theta_noise_deg = 1.0;   // SUPUESTO
  double rover_drop_p = 0.02;     // SUPUESTO: cuadros sin detectar el marcador
};

// Rover: caja 120×100 mm con paletas delanteras (Webots). Eje de giro al centro.
constexpr double kRear = 60, kFront = 60, kHalfWidth = 50;
constexpr double kBumperFront = 77, kBumperHalf = 38;
constexpr double kCubeRadius = 30;       // cubo de 60 mm como disco
constexpr double kSurfaceMargin = 70;    // superficie física ~1 m vs cancha lógica

// --vision-delay-ms: latencia extra captura → estado (prueba de estrés).
double g_extra_latency_ms = 0;

struct Rng {
  std::mt19937_64 engine;
  explicit Rng(uint64_t seed) : engine(seed) {}
  double uniform(double a, double b) { return std::uniform_real_distribution<double>(a, b)(engine); }
  double normal(double sigma) { return sigma <= 0 ? 0 : std::normal_distribution<double>(0, sigma)(engine); }
  bool chance(double p) { return uniform(0, 1) < p; }
};

// ----------------------------------------------------------------- firmware
struct Firmware {
  void* handle = nullptr;
  FwVoid setup = nullptr, loop = nullptr;
  FwSetTime set_time = nullptr;
  FwGetTime get_time = nullptr;
  FwTcpPush tcp_push = nullptr;
  FwSetFlag tcp_online = nullptr, imu_ok = nullptr;
  FwImu imu_sample = nullptr;
  FwMotors motors = nullptr;
  FwStatusFn status = nullptr;
  FwSerialCapture serial_capture = nullptr;
  FwSerialTake serial_take = nullptr;
  FwSerialTake espnow_take = nullptr;
  FwTcpPush espnow_deliver = nullptr;
  FwSerialInput serial_input = nullptr;
  FwSetFlag button = nullptr;
  FwSetFlag line = nullptr;

  template<class T> bool bind(T& target, const char* name) {
    target = reinterpret_cast<T>(dlsym(handle, name));
    if (!target) std::fprintf(stderr, "símbolo ausente %s\n", name);
    return target != nullptr;
  }
  bool load(const std::string& file) {
    path = file;
    // RTLD_LOCAL + visibilidad oculta: cada rover con su propio estado global.
    handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle) { std::fprintf(stderr, "%s\n", dlerror()); return false; }
    return bind(setup, "fw_setup") && bind(loop, "fw_loop") && bind(set_time, "fw_set_time") && bind(get_time, "fw_get_time") &&
           bind(tcp_push, "fw_tcp_push") && bind(tcp_online, "fw_tcp_online") &&
           bind(imu_ok, "fw_imu_ok") && bind(imu_sample, "fw_imu_sample") &&
           bind(motors, "fw_motors") && bind(status, "fw_status") &&
           bind(serial_capture, "fw_serial_capture") && bind(serial_take, "fw_serial_take") &&
           bind(espnow_take, "fw_espnow_take") && bind(espnow_deliver, "fw_espnow_deliver") &&
           bind(serial_input, "fw_serial_input") && bind(button, "fw_button") &&
           bind(line, "fw_line");
  }
  std::string path;
  void unload() {
    if (!handle) return;
    dlclose(handle); handle = nullptr;
    // Si la biblioteca siguiera cargada, la próxima corrida heredaría sus globales.
    if (void* still = dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD)) {
      std::fprintf(stderr, "%s no se descargó; abortando para no mezclar corridas\n", path.c_str());
      dlclose(still); std::exit(4);
    }
  }
};

// ---------------------------------------------------------------- escenario
struct RoverSpec { int id; double col, row, theta_deg; };
struct CubeSpec { std::string color; double col, row; };
struct DepotSpec { std::string color; double col, row; };

enum class EventType { CUBE_HIDE, ROVER_HIDE, TCP_DOWN, IMU_FAIL, CUBE_HIDE_NEAR, MOTOR_FAIL, ESPNOW_DOWN, ESTOP, BUTTON,
                       VISION_BIAS };
struct Event {
  EventType type;
  std::string target;   // color o id de rover
  double t0_ms = 0, t1_ms = 0;  // relativos al inicio de RUNNING
  double radius_mm = 0;         // CUBE_HIDE_NEAR: se oculta al acercarse un rover
  double duration_ms = 0;
  double bias_x_mm = 0;         // VISION_BIAS: la cámara informa al rover corrido en x
};

struct Scenario {
  std::string name, purpose;
  std::vector<RoverSpec> rovers;
  std::vector<CubeSpec> cubes;
  std::vector<DepotSpec> depots = {{"green", 21.5, 3.75}, {"red", 39.25, 21.5}, {"blue", 21.5, 39.25}};
  std::vector<Event> events;
  uint16_t cols = 43, rows = 43;
  double cell_mm = 20, depot_length = 10, depot_depth = 7.5, cube_side = 3;
  double start_col = 3.75, start_row = 21.5;
  double round_ms = 600000, prep_ms = 3000;
  int required = -1;            // -1: todos los cubos
  uint64_t time_offset_us = 0;  // p. ej. para cruzar el desborde de micros()
  bool random_cubes = false;
};

std::vector<RoverSpec> officialStart(bool both = true) {
  // Salida compartida (3.75, 21.5), uno a cada lado a 200 mm entre centros:
  // más juntos, el primer giro en sitio (radio de barrido ~92 mm) los choca.
  if (!both) return {{10, 3.75, 21.5, 0}};
  return {{10, 3.75, 16.5, 0}, {11, 3.75, 26.5, 0}};
}

std::vector<Scenario> catalog() {
  std::vector<Scenario> list;
  auto add = [&](Scenario s) { list.push_back(std::move(s)); };
  Scenario s;

  s = {}; s.name = "un_cubo"; s.purpose = "Un rover, un cubo en línea con su depósito";
  s.rovers = officialStart(false); s.cubes = {{"red", 22, 21.5}}; add(s);

  s = {}; s.name = "un_cubo_diagonal"; s.purpose = "Un rover, cubo fuera de eje: giro, aproximación y empuje en diagonal";
  s.rovers = officialStart(false); s.cubes = {{"green", 14, 14}}; add(s);

  s = {}; s.name = "nominal_3_cubos"; s.purpose = "Disposición de Webots: dos rovers, tres cubos";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}, {"blue", 15, 29}, {"red", 33, 26}}; add(s);

  s = {}; s.name = "lado_equivocado"; s.purpose = "El rover está entre el cubo y su depósito: no debe atravesar el cubo";
  s.rovers = {{10, 30, 21.5, 0}}; s.cubes = {{"red", 20, 21.5}}; add(s);

  s = {}; s.name = "cubo_en_borde"; s.purpose = "Cubo a 4,5 celdas del borde opuesto a su depósito: el empuje directo no cabe";
  s.rovers = officialStart(false); s.cubes = {{"green", 21.5, 38.5}}; add(s);

  s = {}; s.name = "rover_sin_tarea"; s.purpose = "Un cubo para dos rovers: el que queda sin tarea no debe bloquear al otro";
  s.rovers = officialStart(); s.cubes = {{"blue", 8, 30}}; add(s);

  s = {}; s.name = "cruce_de_rutas"; s.purpose = "Tareas que se cruzan en el centro";
  s.rovers = {{10, 6, 8, 0}, {11, 6, 35, 0}};
  s.cubes = {{"blue", 17, 17}, {"green", 17, 26}}; add(s);

  s = {}; s.name = "cubo_oculto_en_empuje"; s.purpose = "El cubo deja de verse 2,5 s cuando un rover se le acerca";
  s.rovers = officialStart(false); s.cubes = {{"red", 22, 21.5}};
  s.events = {{EventType::CUBE_HIDE_NEAR, "red", 0, 0, 140, 2500}}; add(s);

  s = {}; s.name = "pose_propia_perdida"; s.purpose = "El rover 10 no se ve durante 1,5 s en plena ronda";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}, {"blue", 15, 29}};
  s.events = {{EventType::ROVER_HIDE, "10", 20000, 21500}}; add(s);

  s = {}; s.name = "tcp_intermitente"; s.purpose = "El rover 11 pierde TCP 3 s y reconecta";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}, {"blue", 15, 29}};
  s.events = {{EventType::TCP_DOWN, "11", 15000, 18000}}; add(s);

  s = {}; s.name = "falla_imu"; s.purpose = "La IMU del rover 10 falla a los 10 s: el rover 11 debe terminar solo";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}, {"blue", 15, 29}};
  s.events = {{EventType::IMU_FAIL, "10", 10000, 1e12}}; add(s);

  // Motores muertos (rueda suelta, driver quemado): la cámara lo sigue viendo
  // quieto junto al punto de empuje de un cubo. El otro debe terminar solo.
  // Posición real donde quedó el rover en la falla de IMU del 23/09 (ritmo 200/700).
  s = {}; s.name = "rover_detenido"; s.purpose = "ABIERTO: rover 10 sin motores detrás del cubo verde (posición real de la falla del 23/09)";
  s.rovers = {{10, 22.6, 17.3, -2}, {11, 3.75, 26.5, 0}}; s.cubes = {{"green", 26, 10}, {"blue", 15, 29}};
  s.events = {{EventType::MOTOR_FAIL, "10", 0, 1e12}}; add(s);

  s = {}; s.name = "rover_detenido_en_linea"; s.purpose = "ABIERTO: rover 10 sin motores justo detrás del cubo, en la línea al depósito";
  s.rovers = {{10, 21.5, 19.5, 90}, {11, 3.75, 26.5, 0}}; s.cubes = {{"green", 21.5, 12}, {"red", 30, 30}};
  s.events = {{EventType::MOTOR_FAIL, "10", 0, 1e12}}; add(s);

  // Un cubo, asignado al rover 10, que entra en paro de emergencia a los 3 s.
  // Con ESP-NOW el rover 11 lo sabe al instante; sin enlace, solo al cabo de
  // 20 s sin verlo moverse.
  s = {}; s.name = "companero_en_falla"; s.purpose = "Rover 10 en paro de emergencia con la tarea; ESP-NOW avisa al 11";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}};
  s.events = {{EventType::ESTOP, "10", 3000, 1e12}}; add(s);

  s = {}; s.name = "companero_en_falla_sin_espnow"; s.purpose = "Igual, con ESP-NOW caído: el 11 lo detecta por visión";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}};
  s.events = {{EventType::ESTOP, "10", 3000, 1e12}, {EventType::ESPNOW_DOWN, "10", 0, 1e12},
              {EventType::ESPNOW_DOWN, "11", 0, 1e12}}; add(s);

  // Con VRC_REQUIRE_START_BUTTON=1 el rover 10 se arma a los 2 s y el 11 nunca:
  // solo el 10 debe moverse. Con 0 (por defecto) el botón no cambia nada.
  s = {}; s.name = "boton_arranque"; s.purpose = "Solo el rover que presiona el botón se mueve (si se exige botón)";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}};
  s.events = {{EventType::BUTTON, "10", 2000, 2200}}; add(s);

  s = {}; s.name = "fin_de_ronda"; s.purpose = "La ronda termina a los 20 s con maniobras en curso: todo debe detenerse";
  s.rovers = officialStart(); s.cubes = {{"green", 26, 10}, {"blue", 15, 29}, {"red", 33, 26}};
  s.round_ms = 20000; s.required = 0; add(s);

  s = {}; s.name = "desborde_micros"; s.purpose = "micros() se desborda (≈71,6 min) en plena ronda";
  s.rovers = officialStart(false); s.cubes = {{"red", 22, 21.5}};
  s.time_offset_us = 4294967296ULL - 12000000ULL; add(s);

  // La cámara informa al rover 10 corrido 200 mm (error de calibración). Con
  // guardia IR o sin él, entrega sin salirse: el empuje se corrige contra el cubo.
  s = {}; s.name = "calibracion_corrida"; s.purpose = "Visión informa al rover 200 mm corrido en x: ¿entrega sin salirse?";
  s.rovers = officialStart(false); s.cubes = {{"red", 30, 21.5}};
  { Event e{EventType::VISION_BIAS, "10", 4000, 1e12}; e.bias_x_mm = -200; s.events = {e}; } add(s);

  s = {}; s.name = "aleatorio"; s.purpose = "Tres cubos en posiciones aleatorias según la semilla";
  s.rovers = officialStart(); s.random_cubes = true; add(s);

  s = {}; s.name = "aleatorio_un_rover"; s.purpose = "Tres cubos aleatorios con un solo rover: aísla navegación de coordinación";
  s.rovers = officialStart(false); s.random_cubes = true; add(s);
  return list;
}

void placeRandomCubes(Scenario& s, Rng& rng) {
  s.cubes.clear();
  const char* colors[3] = {"green", "red", "blue"};
  for (int i = 0; i < 3; ++i) {
    for (int attempt = 0; attempt < 1000; ++attempt) {
      const double c = rng.uniform(6, s.cols - 6), r = rng.uniform(6, s.rows - 6);
      bool ok = std::hypot(c - s.start_col, r - s.start_row) > 10;
      for (const auto& d : s.depots) ok = ok && std::hypot(c - d.col, r - d.row) > 9;
      for (const auto& o : s.cubes) ok = ok && std::hypot(c - o.col, r - o.row) > 6;
      if (ok) { s.cubes.push_back({colors[i], c, r}); break; }
    }
  }
}

// ------------------------------------------------------------------- mundo
struct Body { double x = 0, y = 0, th = 0, vl = 0, vr = 0, v = 0, w = 0; };

struct Msg { double deliver_ms; std::string line; double self_age_ms; double capture_ms; bool self_seen; };

struct Rover {
  int id = 0;
  Firmware fw;
  bool estop_sent = false;
  Body body;
  float cmd_l = 0, cmd_r = 0;
  FwStatus status{};
  bool contact_cube = false;
  double next_imu_us = 0;
  std::deque<Msg> queue;
  double last_delivery_ms = -1;
  // Lo que el rover sabe de sí mismo según el último mensaje entregado.
  double known_self_age_ms = 1e9, known_at_ms = 0;
  double last_seen_capture_ms = -1e9;
  double seen_x = 0, seen_y = 0, seen_th = 0;
  double travelled_mm = 0, moving_ms = 0, still_run_ms = 0, longest_still_ms = 0;
  std::string first_fault;
  FILE* serial_log = nullptr;
  bool edge_backoff = false;
};

struct Cube {
  std::string color;
  double x = 0, y = 0;
  double last_seen_capture_ms = -1e9, seen_x = 0, seen_y = 0;
  double hide_until_ms = -1; bool near_hide_used = false;
  double entered_ms = -1;   // instante en que entró por completo a su zona
  bool ever_delivered = false; int removed_after_delivery = 0;
  bool lost = false;
};

struct Frame {
  double capture_ms = 0;
  struct R { int id; bool seen; double x, y, th; };
  std::vector<R> rovers;
  std::vector<bool> cube_seen;
  std::vector<double> cube_x, cube_y;
};

struct Result {
  std::string scenario; uint64_t seed = 0;
  bool pass = false; std::vector<std::string> reasons;
  int delivered = 0, required = 0; double completion_ms = -1;
  int collisions = 0, off_surface = 0; double min_center_gap_mm = 1e9;
  int edge_backoffs = 0;  // órdenes cambiadas por el guardia de borde IR
  double max_out_mm = 0;  // cuánto llegó a salir una esquina más allá de la superficie física
  std::map<int, std::string> rover_notes;
};

double wrapPi(double a) { while (a > kPi) a -= 2 * kPi; while (a <= -kPi) a += 2 * kPi; return a; }

struct Vec { double x, y; };
Vec forward(double th) { return {std::cos(th), -std::sin(th)}; }
Vec leftOf(double th) { return {-std::sin(th), -std::cos(th)}; }
Vec toLocal(const Body& b, double x, double y) {
  const Vec f = forward(b.th), l = leftOf(b.th);
  const double dx = x - b.x, dy = y - b.y;
  return {dx * f.x + dy * f.y, dx * l.x + dy * l.y};
}
Vec toWorldDir(const Body& b, Vec local) {
  const Vec f = forward(b.th), l = leftOf(b.th);
  return {local.x * f.x + local.y * l.x, local.x * f.y + local.y * l.y};
}

// Penetración de un disco en un rectángulo local [x0,x1]×[−h,h]; normal local.
bool discInRect(Vec p, double r, double x0, double x1, double h, Vec& normal, double& depth) {
  const double cx = std::clamp(p.x, x0, x1), cy = std::clamp(p.y, -h, h);
  const double dx = p.x - cx, dy = p.y - cy, d = std::hypot(dx, dy);
  if (d > 1e-9) {
    if (d >= r) return false;
    normal = {dx / d, dy / d}; depth = r - d; return true;
  }
  // Centro dentro: salir por el lado más cercano.
  const double opts[4] = {p.x - x0, x1 - p.x, p.y + h, h - p.y};
  const Vec dirs[4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  const int k = static_cast<int>(std::min_element(opts, opts + 4) - opts);
  normal = dirs[k]; depth = opts[k] + r; return true;
}

std::vector<Vec> corners(const Body& b) {
  std::vector<Vec> out;
  for (auto [lx, ly] : std::vector<std::pair<double, double>>{
           {kBumperFront, kHalfWidth}, {kBumperFront, -kHalfWidth}, {-kRear, -kHalfWidth}, {-kRear, kHalfWidth}}) {
    const Vec d = toWorldDir(b, {lx, ly});
    out.push_back({b.x + d.x, b.y + d.y});
  }
  return out;
}

bool rectsOverlap(const Body& a, const Body& b) {
  const auto ca = corners(a), cb = corners(b);
  for (const Body* body : {&a, &b}) {
    for (const Vec axis : {forward(body->th), leftOf(body->th)}) {
      double amin = 1e18, amax = -1e18, bmin = 1e18, bmax = -1e18;
      for (auto p : ca) { const double s = p.x * axis.x + p.y * axis.y; amin = std::min(amin, s); amax = std::max(amax, s); }
      for (auto p : cb) { const double s = p.x * axis.x + p.y * axis.y; bmin = std::min(bmin, s); bmax = std::max(bmax, s); }
      if (amax < bmin || bmax < amin) return false;
    }
  }
  return true;
}

class Simulation {
 public:
  Simulation(Scenario scenario, uint64_t seed, bool randomize, const std::string& fw_dir,
             const std::string& trace_path, const std::string& serial_dir)
      : s_(std::move(scenario)), rng_(seed) {
    result_.scenario = s_.name; result_.seed = seed;
    if (s_.random_cubes) placeRandomCubes(s_, rng_);
    vis_.proc_ms += g_extra_latency_ms;
    if (randomize) perturb();
    for (const auto& spec : s_.rovers) {
      Rover r; r.id = spec.id;
      r.body.x = spec.col * s_.cell_mm; r.body.y = spec.row * s_.cell_mm; r.body.th = spec.theta_deg * kDeg;
      rovers_.push_back(std::move(r));
    }
    for (auto& r : rovers_) {
      const std::string path = fw_dir + "/fw_" + std::to_string(r.id) + ".so";
      if (!r.fw.load(path)) { ok_ = false; return; }
      if (!serial_dir.empty()) {
        r.fw.serial_capture(1);
        r.serial_log = std::fopen((serial_dir + "/" + s_.name + "_" + std::to_string(seed) + "_rover" +
                                   std::to_string(r.id) + ".log").c_str(), "w");
      }
    }
    for (const auto& c : s_.cubes) {
      Cube cube; cube.color = c.color; cube.x = c.col * s_.cell_mm; cube.y = c.row * s_.cell_mm;
      cubes_.push_back(cube);
    }
    if (!trace_path.empty()) trace_ = std::fopen(trace_path.c_str(), "w");
  }
  ~Simulation() {
    for (auto& r : rovers_) { r.fw.unload(); if (r.serial_log) std::fclose(r.serial_log); }
    if (trace_) std::fclose(trace_);
  }
  bool ok() const { return ok_; }

  Result run() {
    const double run0 = 1000 + s_.prep_ms;
    const double run_end = run0 + s_.round_ms;
    // setup() avanza su propio reloj (delay); la simulación continúa desde ahí.
    uint64_t first_step = 0;
    for (auto& r : rovers_) {
      setTime(r, 0); r.fw.setup();
      const uint64_t used_ms = (r.fw.get_time() - s_.time_offset_us) / 1000 + 1;
      first_step = std::max(first_step, used_ms);
    }
    double finished_at = -1;
    writeTraceHeader();
    for (step_ = first_step;; ++step_) {
      const double t = step_ * kDtMs;
      phase_ = t < 1000 ? "IDLE" : (t < run0 ? "READY" : "RUNNING");
      if (finished_at >= 0 || t >= run_end) {
        if (finished_at < 0) finished_at = std::min(t, run_end);
        phase_ = "FINISHED";
      }
      if (finished_at >= 0 && t > finished_at + 2500) break;
      phase_elapsed_ = phase_ == std::string("IDLE") ? t : phase_ == std::string("READY") ? t - 1000
                     : phase_ == std::string("RUNNING") ? t - run0 : t - finished_at;
      phase_total_ = phase_ == std::string("READY") ? s_.prep_ms : phase_ == std::string("RUNNING") ? s_.round_ms : 0;
      run_t_ = t - run0;
      applyEvents(t);
      visionStep(t);
      for (auto& r : rovers_) firmwareStep(r, t);
      physicsStep();
      if (phase_ == std::string("RUNNING")) {
        const double done = refereeStep(t);
        if (done >= 0) { finished_at = t; result_.completion_ms = done - run0; }
      }
      checks(t, run0, finished_at);
      if (trace_ && step_ % 50 == 0) writeTrace(t);
    }
    finalize(run0);
    return result_;
  }

 private:
  static constexpr double kDtMs = 1.0;
  Scenario s_;
  Rng rng_;
  Physics phys_;
  Vision vis_;
  std::vector<Rover> rovers_;
  std::vector<Cube> cubes_;
  std::deque<Frame> frames_;
  Frame ready_frame_;
  bool have_frame_ = false, ok_ = true;
  double next_capture_ms_ = 0, next_pub_ms_ = 0;
  uint32_t seq_ = 0;
  uint64_t step_ = 0;
  const char* phase_ = "IDLE";
  double phase_elapsed_ = 0, phase_total_ = 0, run_t_ = 0;
  double all_still_since_ = -1, running_ms_ = 0;
  bool stall_reported_ = false;
  Result result_;
  FILE* trace_ = nullptr;
  bool colliding_ = false;

  void perturb() {
    // Dominio aleatorio ±10 %: la lógica no debe depender de un número exacto.
    auto j = [&](double& v, double spread) { v *= 1 + rng_.uniform(-spread, spread); };
    j(phys_.k_mm_s, 0.10); j(phys_.gain_right, 0.05); j(phys_.tau_s, 0.2);
    j(phys_.push_factor, 0.10); j(vis_.proc_ms, 0.3);
  }

  void setTime(Rover& r, double t_ms) { r.fw.set_time(s_.time_offset_us + static_cast<uint64_t>(t_ms * 1000)); }

  Rover* roverById(const std::string& id) {
    for (auto& r : rovers_) if (std::to_string(r.id) == id) return &r;
    return nullptr;
  }
  Cube* cubeByColor(const std::string& color) {
    for (auto& c : cubes_) if (c.color == color) return &c;
    return nullptr;
  }

  bool eventActive(EventType type, const std::string& target) const {
    for (const auto& e : s_.events)
      if (e.type == type && e.target == target && run_t_ >= e.t0_ms && run_t_ < e.t1_ms &&
          phase_ == std::string("RUNNING")) return true;
    return false;
  }

  void applyEvents(double t) {
    for (auto& r : rovers_) {
      const bool down = eventActive(EventType::TCP_DOWN, std::to_string(r.id));
      r.fw.tcp_online(!down);
      if (down) r.queue.clear();
      r.fw.imu_ok(!eventActive(EventType::IMU_FAIL, std::to_string(r.id)));
    }
    for (const auto& e : s_.events) {
      if (e.type != EventType::CUBE_HIDE_NEAR || phase_ != std::string("RUNNING")) continue;
      Cube* c = cubeByColor(e.target);
      if (!c || c->near_hide_used) continue;
      for (const auto& r : rovers_)
        if (std::hypot(r.body.x - c->x, r.body.y - c->y) < e.radius_mm) {
          c->near_hide_used = true; c->hide_until_ms = t + e.duration_ms;
        }
    }
  }

  bool cubeVisible(const Cube& c, double t) {
    if (c.lost || t < c.hide_until_ms) return false;
    if (eventActive(EventType::CUBE_HIDE, c.color)) return false;
    for (const auto& r : rovers_) {  // un chasis encima tapa el cubo
      const Vec p = toLocal(r.body, c.x, c.y);
      if (p.x > -kRear && p.x < kFront && std::fabs(p.y) < kHalfWidth) return false;
    }
    return true;
  }

  void visionStep(double t) {
    if (t >= next_capture_ms_) {
      next_capture_ms_ += 1000.0 / vis_.cam_fps;
      Frame f; f.capture_ms = t;
      for (auto& r : rovers_) {
        const bool seen = !eventActive(EventType::ROVER_HIDE, std::to_string(r.id)) && !rng_.chance(vis_.rover_drop_p);
        double bias = 0;
      for (const auto& e : s_.events)
        if (e.type == EventType::VISION_BIAS && eventActive(e.type, std::to_string(r.id))) bias = e.bias_x_mm;
      f.rovers.push_back({r.id, seen, r.body.x + bias + rng_.normal(vis_.pos_noise_mm),
                            r.body.y + rng_.normal(vis_.pos_noise_mm),
                            r.body.th + rng_.normal(vis_.theta_noise_deg * kDeg)});
      }
      for (auto& c : cubes_) {
        f.cube_seen.push_back(cubeVisible(c, t));
        f.cube_x.push_back(c.x + rng_.normal(vis_.pos_noise_mm));
        f.cube_y.push_back(c.y + rng_.normal(vis_.pos_noise_mm));
      }
      frames_.push_back(std::move(f));
    }
    while (!frames_.empty() && frames_.front().capture_ms + vis_.proc_ms <= t) {
      const Frame& f = frames_.front();
      for (size_t i = 0; i < f.rovers.size(); ++i) if (f.rovers[i].seen) {
        rovers_[i].last_seen_capture_ms = f.capture_ms;
        rovers_[i].seen_x = f.rovers[i].x; rovers_[i].seen_y = f.rovers[i].y; rovers_[i].seen_th = f.rovers[i].th;
      }
      for (size_t i = 0; i < cubes_.size(); ++i) if (f.cube_seen[i]) {
        cubes_[i].last_seen_capture_ms = f.capture_ms;
        cubes_[i].seen_x = f.cube_x[i]; cubes_[i].seen_y = f.cube_y[i];
      }
      ready_frame_ = f; have_frame_ = true;
      frames_.pop_front();
    }
    if (t >= next_pub_ms_ && have_frame_) {
      next_pub_ms_ += 1000.0 / vis_.pub_hz;
      publish(t);
    }
  }

  void publish(double t) {
    const double cap = ready_frame_.capture_ms;
    char buf[4096];
    int n = std::snprintf(buf, sizeof(buf),
        "{\"v\":2,\"seq\":%u,\"ts_ms\":%llu,\"phase\":\"%s\",\"clock\":{\"elapsed_ms\":%u,\"remaining_ms\":%u,\"total_ms\":%u},"
        "\"grid\":{\"cols\":%u,\"rows\":%u,\"cell_mm\":%g},\"start\":{\"col\":%g,\"row\":%g},"
        "\"depots\":[", ++seq_, 1758600000000ULL + static_cast<unsigned long long>(cap), phase_,
        static_cast<unsigned>(phase_elapsed_), static_cast<unsigned>(std::max(0.0, phase_total_ - phase_elapsed_)),
        static_cast<unsigned>(phase_total_), s_.cols, s_.rows, s_.cell_mm, s_.start_col, s_.start_row);
    for (size_t i = 0; i < s_.depots.size(); ++i)
      n += std::snprintf(buf + n, sizeof(buf) - n, "%s{\"color\":\"%s\",\"col\":%g,\"row\":%g}", i ? "," : "",
                         s_.depots[i].color.c_str(), s_.depots[i].col, s_.depots[i].row);
    n += std::snprintf(buf + n, sizeof(buf) - n,
        "],\"depot_size\":{\"length\":%g,\"depth\":%g},\"cube_side\":%g,\"rovers\":[",
        s_.depot_length, s_.depot_depth, s_.cube_side);
    bool first = true;
    for (const auto& r : rovers_) {
      if (r.last_seen_capture_ms < -1e8) continue;  // nunca visto: no se publica
      double th = std::fmod(r.seen_th / kDeg, 360.0); if (th < 0) th += 360;
      n += std::snprintf(buf + n, sizeof(buf) - n, "%s{\"id\":%d,\"col\":%.3f,\"row\":%.3f,\"theta\":%.2f,\"age_ms\":%u}",
                         first ? "" : ",", r.id, r.seen_x / s_.cell_mm, r.seen_y / s_.cell_mm, th,
                         static_cast<unsigned>(cap - r.last_seen_capture_ms));
      first = false;
    }
    n += std::snprintf(buf + n, sizeof(buf) - n, "],\"cubes\":[");
    first = true;
    for (const auto& c : cubes_) {
      if (c.last_seen_capture_ms < -1e8) continue;
      n += std::snprintf(buf + n, sizeof(buf) - n, "%s{\"color\":\"%s\",\"col\":%.3f,\"row\":%.3f,\"age_ms\":%u}",
                         first ? "" : ",", c.color.c_str(), c.seen_x / s_.cell_mm, c.seen_y / s_.cell_mm,
                         static_cast<unsigned>(cap - c.last_seen_capture_ms));
      first = false;
    }
    n += std::snprintf(buf + n, sizeof(buf) - n, "],\"obstacles\":[]}\n");
    for (auto& r : rovers_) {
      if (eventActive(EventType::TCP_DOWN, std::to_string(r.id))) continue;
      // TCP conserva el orden: nunca se entrega antes que el mensaje anterior.
      const double deliver = std::max(r.last_delivery_ms, t + rng_.uniform(vis_.net_min_ms, vis_.net_max_ms));
      r.last_delivery_ms = deliver;
      const bool self_seen = r.last_seen_capture_ms > -1e8;
      r.queue.push_back({deliver, std::string(buf, n), self_seen ? cap - r.last_seen_capture_ms : 1e9, cap, self_seen});
    }
  }

  // Difusión ESP-NOW: lo que envía un rover llega a los demás (5 % de pérdida).
  void routeEspNow(Rover& from, double t) {
    char packet[64];
    for (size_t n; (n = from.fw.espnow_take(packet, sizeof(packet))) > 0;) {
      if (eventActive(EventType::ESPNOW_DOWN, std::to_string(from.id))) continue;
      for (auto& to : rovers_) {
        if (&to == &from || eventActive(EventType::ESPNOW_DOWN, std::to_string(to.id)) || rng_.chance(0.05)) continue;
        setTime(to, t);  // el receptor sella con su propio millis()
        to.fw.espnow_deliver(packet, n);
      }
    }
  }

  // IR del borde sobre el tablero de ajedrez físico (celdas de 20 mm, que se
  // extiende kSurfaceMargin más allá de la cancha lógica); afuera, lona blanca.
  // Posiciones SUPUESTAS hasta medirlas: S1/S2 adelante izq/der, S3/S4 atrás.
  int lineMask(const Body& b) {
    static const double kSensors[4][2] = {{55, -30}, {55, 30}, {-55, -30}, {-55, 30}};  // adelante, derecha
    const double w = s_.cols * s_.cell_mm, h = s_.rows * s_.cell_mm;
    int mask = 0;
    for (int i = 0; i < 4; ++i) {
      const double f = kSensors[i][0], l = kSensors[i][1];
      const double x = b.x + f * std::cos(b.th) + l * std::sin(b.th) + rng_.normal(1.0);
      const double y = b.y - f * std::sin(b.th) + l * std::cos(b.th) + rng_.normal(1.0);
      bool black = false;
      if (x > -kSurfaceMargin && y > -kSurfaceMargin && x < w + kSurfaceMargin && y < h + kSurfaceMargin)
        black = (static_cast<long>(std::floor((x + kSurfaceMargin) / 20)) +
                 static_cast<long>(std::floor((y + kSurfaceMargin) / 20))) & 1;
      if (!black) mask |= 1 << i;  // LINE_BLACK_IS_LOW: blanco = HIGH
    }
    return mask;
  }

  void firmwareStep(Rover& r, double t) {
    while (!r.queue.empty() && r.queue.front().deliver_ms <= t) {
      const Msg& m = r.queue.front();
      r.fw.tcp_push(m.line.data(), m.line.size());
      if (m.self_seen) { r.known_self_age_ms = m.self_age_ms; r.known_at_ms = t; }
      r.queue.pop_front();
    }
    const double t_us = t * 1000;
    if (t_us >= r.next_imu_us) {
      r.next_imu_us += 1e6 / phys_.imu_hz;
      // Signo del perfil 1 (GSIGN 1): positivo = giro a la izquierda = θ crece.
      r.fw.imu_sample(static_cast<float>(r.body.w / kDeg + phys_.gyro_bias_dps + rng_.normal(phys_.gyro_noise_dps)));
    }
    setTime(r, t);
    // Paro de emergencia local ('!' por serie): el rover queda detenido y lo anuncia.
    if (!r.estop_sent && eventActive(EventType::ESTOP, std::to_string(r.id))) {
      r.fw.serial_input("!"); r.estop_sent = true;
    }
    r.fw.button(eventActive(EventType::BUTTON, std::to_string(r.id)));
    r.fw.line(lineMask(r.body));
    r.fw.loop();
    routeEspNow(r, t);
    r.fw.motors(&r.cmd_l, &r.cmd_r);
    r.fw.status(&r.status);
    const bool backoff = r.status.mode && std::string(r.status.mode) == "EDGE_BACK_OFF";
    if (backoff && !r.edge_backoff) ++result_.edge_backoffs;
    r.edge_backoff = backoff;
    if (r.serial_log) {
      char chunk[4096];
      for (size_t k; (k = r.fw.serial_take(chunk, sizeof(chunk))) > 0;) {
        std::fwrite(chunk, 1, k, r.serial_log);
      }
    }
  }

  double wheelTarget(double u, bool pivot, bool rotating) const {
    const double a = std::fabs(u);
    if (a < 1e-6) return 0;
    double d = phys_.deadband;
    if (pivot) {
      if (!rotating && a < phys_.pivot_static) return 0;
      d = phys_.pivot_kinetic;
    }
    return (u > 0 ? 1 : -1) * phys_.k_mm_s * std::max(0.0, a - d);
  }

  void physicsStep() {
    const double dt = kDtMs / 1000.0;
    std::vector<Body> before;
    for (auto& r : rovers_) before.push_back(r.body);
    for (auto& r : rovers_) {
      Body& b = r.body;
      if (eventActive(EventType::MOTOR_FAIL, std::to_string(r.id))) { b.v = b.w = b.vl = b.vr = 0; continue; }
      const bool pivot = r.cmd_l * r.cmd_r < 0;
      const bool rotating = std::fabs(b.w) > 0.05;
      const double load = r.contact_cube ? phys_.push_factor : 1.0;
      const double tl = phys_.gain_left * load * wheelTarget(r.cmd_l, pivot, rotating);
      const double tr = phys_.gain_right * load * wheelTarget(r.cmd_r, pivot, rotating);
      const double alpha = std::min(1.0, dt / phys_.tau_s);
      b.vl += (tl - b.vl) * alpha; b.vr += (tr - b.vr) * alpha;
      b.v = (b.vl + b.vr) / 2; b.w = (b.vr - b.vl) / phys_.track_mm;
      b.th = wrapPi(b.th + b.w * dt);
      b.x += b.v * std::cos(b.th) * dt; b.y -= b.v * std::sin(b.th) * dt;
      r.travelled_mm += std::fabs(b.v) * dt;
    }
    if (rovers_.size() == 2 && rectsOverlap(rovers_[0].body, rovers_[1].body)) {
      if (!colliding_) ++result_.collisions;
      colliding_ = true;
      for (size_t i = 0; i < rovers_.size(); ++i) {
        rovers_[i].body = before[i];
        rovers_[i].body.vl = rovers_[i].body.vr = rovers_[i].body.v = rovers_[i].body.w = 0;
      }
    } else {
      colliding_ = false;
    }
    for (auto& r : rovers_) {
      r.contact_cube = false;
      for (auto& c : cubes_) {
        if (c.lost) continue;
        const Vec p = toLocal(r.body, c.x, c.y);
        Vec n{}; double depth = 0;
        bool hit = discInRect(p, kCubeRadius, -kRear, kFront, kHalfWidth, n, depth);
        Vec n2{}; double depth2 = 0;
        if (discInRect(p, kCubeRadius, kFront, kBumperFront, kBumperHalf, n2, depth2) && (!hit || depth2 > depth)) {
          hit = true; n = n2; depth = depth2;
        }
        if (!hit) continue;
        r.contact_cube = true;
        const Vec wn = toWorldDir(r.body, n);
        // Velocidad del punto de contacto (v + ω×r), parte tangencial con arrastre.
        const Vec contact = toWorldDir(r.body, {p.x - n.x * kCubeRadius, p.y - n.y * kCubeRadius});
        const Vec f = forward(r.body.th);
        const double vx = r.body.v * f.x + r.body.w * contact.y;
        const double vy = r.body.v * f.y - r.body.w * contact.x;
        const double vn = vx * wn.x + vy * wn.y;
        const double tx = vx - vn * wn.x, ty = vy - vn * wn.y;
        c.x += wn.x * depth + tx * phys_.cube_mu * dt;
        c.y += wn.y * depth + ty * phys_.cube_mu * dt;
      }
    }
    for (size_t i = 0; i < cubes_.size(); ++i)
      for (size_t j = i + 1; j < cubes_.size(); ++j) {
        const double dx = cubes_[j].x - cubes_[i].x, dy = cubes_[j].y - cubes_[i].y, d = std::hypot(dx, dy);
        if (d < 2 * kCubeRadius && d > 1e-9) {
          const double push = (2 * kCubeRadius - d) / 2;
          cubes_[i].x -= dx / d * push; cubes_[i].y -= dy / d * push;
          cubes_[j].x += dx / d * push; cubes_[j].y += dy / d * push;
        }
      }
    const double w = s_.cols * s_.cell_mm, h = s_.rows * s_.cell_mm;
    for (auto& c : cubes_)
      if (!c.lost && (c.x < 0 || c.y < 0 || c.x > w || c.y > h)) c.lost = true;
  }

  // Árbitro con la regla del contrato (sección cube_side), sobre la verdad física.
  bool inDepot(const Cube& c, const DepotSpec& d) const {
    const double w = s_.cols, h = s_.rows;
    const double nearest = std::min({d.col, w - d.col, d.row, h - d.row});
    const bool horizontal = nearest == d.row || nearest == h - d.row;
    const double margin = s_.cube_side * kSqrt2 / 2;
    const double half_col = (horizontal ? s_.depot_length : s_.depot_depth) / 2 - margin;
    const double half_row = (horizontal ? s_.depot_depth : s_.depot_length) / 2 - margin;
    return std::fabs(c.x / s_.cell_mm - d.col) <= half_col && std::fabs(c.y / s_.cell_mm - d.row) <= half_row;
  }

  double refereeStep(double t) {
    bool all = !cubes_.empty();
    double last_entry = -1;
    int inside_count = 0;
    for (auto& c : cubes_) {
      const DepotSpec* depot = nullptr;
      for (const auto& d : s_.depots) if (d.color == c.color) depot = &d;
      const bool inside = depot && !c.lost && inDepot(c, *depot);
      if (inside && c.entered_ms < 0) c.entered_ms = t;
      if (!inside && c.entered_ms >= 0) {
        if (c.ever_delivered) ++c.removed_after_delivery;
        c.entered_ms = -1;
      }
      if (inside && t - c.entered_ms >= 1000) c.ever_delivered = true;
      if (inside) { ++inside_count; last_entry = std::max(last_entry, c.entered_ms); }
      all = all && inside && t - c.entered_ms >= 1000;  // permanencia de 1 s
    }
    result_.delivered = inside_count;
    return all ? last_entry : -1;
  }

  void fault(Rover& r, const std::string& what) {
    if (r.first_fault.empty()) {
      r.first_fault = what;
      result_.reasons.push_back("rover " + std::to_string(r.id) + ": " + what);
    }
  }

  void checks(double t, double run0, double finished_at) {
    const double w = s_.cols * s_.cell_mm, h = s_.rows * s_.cell_mm;
    bool all_still = true;
    for (auto& r : rovers_) {
      const bool moving_cmd = r.cmd_l != 0 || r.cmd_r != 0;
      if (moving_cmd && t < run0) fault(r, "movimiento antes de RUNNING");
      if (moving_cmd && finished_at >= 0 && t > finished_at + 300) fault(r, "movimiento después de FINISHED");
      const double self_age = r.known_self_age_ms + (t - r.known_at_ms);
      if (moving_cmd && self_age > 300 + 50) fault(r, "movimiento con pose propia vencida");
      for (auto p : corners(r.body))
        result_.max_out_mm = std::max({result_.max_out_mm, -kSurfaceMargin - p.x, -kSurfaceMargin - p.y,
                                       p.x - w - kSurfaceMargin, p.y - h - kSurfaceMargin});
      for (auto p : corners(r.body))
        if (p.x < -kSurfaceMargin || p.y < -kSurfaceMargin || p.x > w + kSurfaceMargin || p.y > h + kSurfaceMargin) {
          if (r.first_fault.find("superficie") == std::string::npos) { ++result_.off_surface; fault(r, "salida de la superficie"); }
          break;
        }
      if (phase_ == std::string("RUNNING")) {
        if (moving_cmd) { r.moving_ms += kDtMs; r.still_run_ms = 0; }
        else { r.still_run_ms += kDtMs; r.longest_still_ms = std::max(r.longest_still_ms, r.still_run_ms); }
      }
      all_still = all_still && !moving_cmd;
    }
    if (rovers_.size() == 2)
      result_.min_center_gap_mm = std::min(result_.min_center_gap_mm,
          std::hypot(rovers_[0].body.x - rovers_[1].body.x, rovers_[0].body.y - rovers_[1].body.y));
    if (phase_ == std::string("RUNNING")) running_ms_ += kDtMs;
    if (phase_ == std::string("RUNNING")) {
      if (!all_still) all_still_since_ = -1;
      else if (all_still_since_ < 0) all_still_since_ = t;
      else if (t - all_still_since_ > 30000 && !stall_reported_) {
        stall_reported_ = true;
        char msg[96]; std::snprintf(msg, sizeof(msg), "todos quietos más de 30 s desde t=%.1f s", (all_still_since_ - run0) / 1000);
        result_.reasons.push_back(msg);
      }
    }
  }

  void finalize(double run0) {
    (void)run0;
    result_.required = s_.required >= 0 ? s_.required : static_cast<int>(cubes_.size());
    for (const auto& c : cubes_) {
      if (c.lost) result_.reasons.push_back("cubo " + c.color + " fuera de la cancha");
      if (c.removed_after_delivery) result_.reasons.push_back("cubo " + c.color + " sacado de su zona tras entregarse");
    }
    if (result_.collisions) result_.reasons.push_back("contacto entre rovers x" + std::to_string(result_.collisions));
    if (result_.delivered < result_.required)
      result_.reasons.push_back("entregas " + std::to_string(result_.delivered) + "/" + std::to_string(result_.required));
    bool critical = result_.collisions || result_.off_surface;
    for (auto& r : rovers_) {
      char note[256];
      std::snprintf(note, sizeof(note), "recorrido=%.0fmm en_mov=%.0f%% quieto_max=%.1fs motion=%s mission=%s",
                    r.travelled_mm, 100 * r.moving_ms / std::max(1.0, running_ms_), r.longest_still_ms / 1000,
                    r.status.motion_reason ? r.status.motion_reason : "?", r.status.mission ? r.status.mission : "?");
      result_.rover_notes[r.id] = note;
      critical = critical || !r.first_fault.empty();
    }
    bool lost = false; for (const auto& c : cubes_) lost = lost || c.lost || c.removed_after_delivery;
    result_.pass = !critical && !lost && result_.delivered >= result_.required;
  }

  void writeTraceHeader() {
    if (!trace_) return;
    std::fprintf(trace_, "# %s | cancha %ux%u celdas de %g mm | depósitos", s_.name.c_str(), s_.cols, s_.rows, s_.cell_mm);
    for (const auto& d : s_.depots) std::fprintf(trace_, " %s:%g:%g", d.color.c_str(), d.col, d.row);
    std::fprintf(trace_, "\nt_ms,phase");
    for (const auto& r : rovers_) std::fprintf(trace_, ",r%d_x,r%d_y,r%d_th,r%d_l,r%d_r,r%d_mode,r%d_motion,r%d_target",
                                               r.id, r.id, r.id, r.id, r.id, r.id, r.id, r.id);
    for (const auto& c : cubes_) std::fprintf(trace_, ",%s_x,%s_y,%s_seen", c.color.c_str(), c.color.c_str(), c.color.c_str());
    std::fprintf(trace_, "\n");
  }
  void writeTrace(double t) {
    std::fprintf(trace_, "%.0f,%s", t, phase_);
    for (const auto& r : rovers_)
      std::fprintf(trace_, ",%.1f,%.1f,%.1f,%.3f,%.3f,%s,%s,%d", r.body.x, r.body.y, r.body.th / kDeg, r.cmd_l, r.cmd_r,
                   r.status.mode ? r.status.mode : "", r.status.motion_reason ? r.status.motion_reason : "", r.status.target);
    for (const auto& c : cubes_)
      std::fprintf(trace_, ",%.1f,%.1f,%d", c.x, c.y, c.last_seen_capture_ms > -1e8 ? static_cast<int>(t - c.last_seen_capture_ms) : -1);
    std::fprintf(trace_, "\n");
  }
};

void printResult(const Result& r, bool json) {
  if (json) {
    std::printf("{\"scenario\":\"%s\",\"seed\":%llu,\"pass\":%s,\"delivered\":%d,\"required\":%d,\"completion_ms\":%.0f,"
                "\"collisions\":%d,\"off_surface\":%d,\"min_center_gap_mm\":%.0f,\"edge_backoffs\":%d,\"max_out_mm\":%.0f,\"reasons\":[",
                r.scenario.c_str(), static_cast<unsigned long long>(r.seed), r.pass ? "true" : "false", r.delivered,
                r.required, r.completion_ms, r.collisions, r.off_surface, r.min_center_gap_mm, r.edge_backoffs, r.max_out_mm);
    for (size_t i = 0; i < r.reasons.size(); ++i) std::printf("%s\"%s\"", i ? "," : "", r.reasons[i].c_str());
    std::printf("]}\n");
    return;
  }
  std::printf("%-22s seed=%-4llu %s  entregas=%d/%d", r.scenario.c_str(), static_cast<unsigned long long>(r.seed),
              r.pass ? "PASS" : "FAIL", r.delivered, r.required);
  if (r.completion_ms >= 0) std::printf("  tiempo=%.1fs", r.completion_ms / 1000);
  if (r.min_center_gap_mm < 1e8) std::printf("  sep_min=%.0fmm", r.min_center_gap_mm);
  std::printf("\n");
  for (const auto& reason : r.reasons) std::printf("    - %s\n", reason.c_str());
  for (const auto& [id, note] : r.rover_notes) std::printf("    rover %d: %s\n", id, note.c_str());
}

void usage() {
  std::printf(
      "Uso: vrc_sim [--fw-dir DIR] [--scenario NOMBRE|all] [--seed N] [--seeds K]\n"
      "             [--exact] [--trace archivo.csv] [--serial-dir DIR] [--json] [--list]\n"
      "             [--vision-delay-ms MS]\n"
      "  --seeds K  corre K semillas consecutivas desde --seed (campaña)\n"
      "  --exact    sin perturbar la física (±10 %%) entre semillas\n"
      "  --vision-delay-ms MS  latencia de visión extra (estrés)\n");
}
}  // namespace

int main(int argc, char** argv) {
  std::string fw_dir = ".", scenario = "all", trace, serial_dir;
  uint64_t seed = 1, seeds = 1;
  bool json = false, exact = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { if (i + 1 >= argc) { usage(); std::exit(2); } return argv[++i]; };
    if (a == "--fw-dir") fw_dir = next();
    else if (a == "--scenario") scenario = next();
    else if (a == "--seed") seed = std::stoull(next());
    else if (a == "--seeds") seeds = std::stoull(next());
    else if (a == "--trace") trace = next();
    else if (a == "--serial-dir") serial_dir = next();
    else if (a == "--json") json = true;
    else if (a == "--exact") exact = true;
    else if (a == "--vision-delay-ms") g_extra_latency_ms = std::stod(next());
    else if (a == "--list") {
      for (const auto& s : catalog()) std::printf("%-22s %s\n", s.name.c_str(), s.purpose.c_str());
      return 0;
    } else { usage(); return a == "--help" ? 0 : 2; }
  }
  int runs = 0, passed = 0;
  for (const auto& s : catalog()) {
    if (scenario != "all" && s.name != scenario) continue;
    for (uint64_t k = 0; k < seeds; ++k) {
      Simulation sim(s, seed + k, !exact, fw_dir, seeds == 1 ? trace : "", serial_dir);
      if (!sim.ok()) return 3;
      const Result r = sim.run();
      printResult(r, json);
      ++runs; passed += r.pass;
    }
  }
  if (!runs) { std::fprintf(stderr, "escenario desconocido: %s (use --list)\n", scenario.c_str()); return 2; }
  if (!json) std::printf("\n%d/%d corridas PASS\n", passed, runs);
  return passed == runs ? 0 : 1;
}
