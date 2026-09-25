#pragma once

// Núcleo portable: no depende de Arduino, Wi-Fi, Webots ni JSON. El firmware y
// el controlador de simulación deben adaptarle sus propios datos de entrada.

#include <cstdint>

namespace rover_logic {

struct Pose {
  float col = 0.0F;
  float row = 0.0F;
  float theta_deg = 0.0F;
};

struct Point {
  float col = 0.0F;
  float row = 0.0F;
};

struct Grid {
  uint16_t cols = 0;
  uint16_t rows = 0;
  float cell_mm = 0.0F;
};

struct DepotSize {
  float length_cells = 0.0F;
  float depth_cells = 0.0F;
};

struct Cube {
  Point position;
  uint32_t age_ms = 0;
};

enum class Color : uint8_t { NONE = 0, GREEN = 1, BLUE = 2, RED = 3 };
struct TaskRover { uint8_t id = 0; Pose pose; uint32_t age_ms = 0; };
struct TaskCube { Color color = Color::NONE; Cube cube; };
struct TaskDepot { Color color = Color::NONE; Point position; };

// Los valores existentes no cambian: Webots los recibe como enteros.
enum class Mode : uint8_t {
  WAIT_TARGET,
  CUBE_DELIVERED,
  APPROACH_ALIGN,
  APPROACH,
  AT_APPROACH,
  PUSH_ALIGN,
  PUSH,
  PUSH_DONE,
  AVOID,        // rodeando un cubo o el otro rover
  BACKOFF,      // retroceso recto: cubo entregado delante u otro rover cerca
  PARK,         // sin tarea: despejar la cancha
  UNREACHABLE,  // ninguna dirección de empuje cabe dentro de la cancha
  YIELD,        // cede el paso al otro rover
};

struct DriveCommand {
  float left = 0.0F;
  float right = 0.0F;
  bool active = false;
  bool pushing = false;
  Mode mode = Mode::WAIT_TARGET;
  float heading_error_deg = 0;
  bool reverse = false;     // retroceso recto, sin girar
  float distance_mm = 0;    // cuánto falta: acorta el tramo cerca del objetivo
};

// Distancias en mm, medidas sobre el CenfoBot (caja 120×100, paletas a 77 mm
// del eje). Revisar contra la cancha: `push_contact_offset_mm` decide dónde
// queda el cubo dentro de la zona (ventana de fondo ±32,6 mm).
struct NavigationConfig {
  uint32_t max_cube_age_ms = 1200;
  uint32_t max_push_cube_age_ms = 3000;  // durante el empuje el cubo se tapa
  float drive_speed = 0.34F;
  float push_speed = 0.26F;
  float turn_speed = 0.28F;
  float steer_gain = 0.16F;
  float turn_in_place_deg = 20.0F;
  float goal_tolerance_mm = 35.0F;
  float slowdown_mm = 140.0F;
  float approach_standoff_mm = 150.0F;   // centro rover → centro cubo al alinear
  float approach_tolerance_mm = 40.0F;
  float push_contact_max_mm = 190.0F;    // más lejos: se perdió el contacto
  float push_contact_offset_mm = 107.0F; // paleta 77 + medio cubo 30
  float push_aim_mm = 60.0F;             // mira delante del cubo, sobre su línea
  float push_lateral_abort_mm = 28.0F;   // el cubo resbala del frente
  float push_angle_abort_deg = 25.0F;
  float settle_margin_mm = 12.0F;        // entrar a la zona con margen, no al filo
  float cube_clearance_mm = 140.0F;      // camino: siempre > cube_turn_clearance_mm, se puede girar en cualquier punto
  // Entre rovers (centro ↔ centro). Tiene paso el que empuja; si ambos o
  // ninguno, el de marcador menor. Los dos lo deducen igual desde la cámara.
  float rover_clearance_mm = 240.0F;       // rodear a quien tiene paso
  float rover_clearance_low_mm = 200.0F;   // rodear a quien me cede el paso
  float rover_turn_clearance_mm = 200.0F;  // girar en sitio barre ~92 mm por rover
  float cube_turn_clearance_mm = 135.0F;   // esquina del rover 92 + media diagonal del cubo 42
  float rover_yield_mm = 260.0F;           // quien tiene paso y se mueve: esperar
  float rover_stop_mm = 160.0F;            // contacto inminente delante: detenerse
  float arena_margin_mm = 60.0F;         // centro del rover respecto del borde lógico
  float backoff_mm = 80.0F;
};

struct Obstacle {
  Point center;
  float radius_mm = 0;
  bool rover = false;
  bool pushing = false;   // rover con un cubo contra las paletas (visto por la cámara)
  bool priority = false;  // ese rover tiene paso sobre mí
  bool moving = false;    // se desplazó hace poco (quieto = obstáculo a rodear)
  float theta_deg = 0;    // orientación del otro rover (para la guarda de contacto)
};

// Sin radio entre rovers, la cámara delata quién empuja: un cubo apoyado en
// las paletas de ese rover.
bool rover_pushing(const Pose& rover, const Point* cubes, uint8_t count, float cell_mm,
                   float contact_mm);

struct AssignConfig {
  uint32_t max_rover_age_ms = 300;
  uint32_t max_cube_age_ms = 1200;
  float hysteresis_mm = 500.0F;   // compromiso: una tarea en curso solo cambia por una mejora grande
  float settle_margin_mm = 12.0F; // mismo criterio de "entregado" que la navegación
  float corridor_clearance_mm = 150.0F;  // corredores cubo→depósito que no se cruzan
};

// Criterio oficial del contrato: centro a media diagonal de cada borde.
bool cube_delivered(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                    const Point& cube, const Point& depot);
// Igual que cube_delivered, con un margen extra en mm para no quedar en el filo.
bool cube_settled(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                  const Point& cube, const Point& depot, float margin_mm);

// Terminado: bien asentado, o dentro y pasado del centro (no se puede mejorar).
bool cube_done(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
               const Point& cube, const Point& depot, float margin_mm);

// Etapas del empuje de un cubo. La dirección se congela al llegar al punto
// de alineación: recalcularla con cada pose ruidosa alterna las etapas.
struct PushState {
  enum Stage : uint8_t { GOTO, ALIGN, PUSH, RETREAT };
  uint8_t stage = GOTO;
  float ux = 1, uy = 0;              // dirección de empuje congelada (mm)
  float retreat_x = 0, retreat_y = 0;  // dónde empezó el retroceso (mm)
  float gx = 0, gy = 0;              // destino del tramo congelado (mm)
  bool staging = false;              // el tramo termina en el punto de preparación
};

// Aproximación y empuje de un cubo, rodeando obstáculos. `state` persiste
// entre llamadas y se reinicia al cambiar de cubo.
DriveCommand navigate(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                      const NavigationConfig& config, const Pose& self, const Cube& cube,
                      const Point& depot, const Obstacle* obstacles, uint8_t obstacle_count,
                      PushState* state);
DriveCommand navigate(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                      const NavigationConfig& config, const Pose& self, const Cube& cube,
                      const Point& depot, bool* pushing);
// Ir a un punto (estacionar) rodeando obstáculos.
DriveCommand go_to(const Grid& grid, const NavigationConfig& config, const Pose& self,
                   const Point& goal, const Obstacle* obstacles, uint8_t obstacle_count, Mode mode);
// Esquina interior más alejada de todo lo que hay que dejar libre.
Point parking_spot(const Grid& grid, const Point* keep_clear, uint8_t count);
const char* mode_name(Mode mode);

// Asigna a lo sumo un cubo por rover: primero cubrir la mayor cantidad de
// rovers, luego el menor costo total (rover→cubo→depósito). `previous` (puede
// ser nulo) da ventaja a las tareas en curso para que no oscilen.
void plan_assignment(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                     const TaskRover* rovers, uint8_t rover_count, const TaskCube* cubes,
                     uint8_t cube_count, const TaskDepot* depots, uint8_t depot_count,
                     const Color* previous, const AssignConfig& config, Color* assigned);
void assign_tasks(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                  const TaskRover* rovers, uint8_t rover_count, const TaskCube* cubes,
                  uint8_t cube_count, const TaskDepot* depots, uint8_t depot_count,
                  uint32_t max_rover_age_ms, uint32_t max_cube_age_ms, Color* assigned);
bool should_yield(const Pose& self, uint8_t self_id, const Pose& other, uint8_t other_id,
                  uint32_t other_age_ms, float cell_mm, float separation_mm,
                  uint32_t max_pose_age_ms = 300);

}  // namespace rover_logic
