# Auditoría de competencia y plan de corrección/presentación — 23/09/2026

Rama `webots-simulation` sobre `40d4c25`, con 25 archivos modificados y ~20
rutas sin versionar. Complementa [PLAN_ACTUAL.md](PLAN_ACTUAL.md) (21/09) y
el registro histórico [MAPA_MENTAL_Y_AUDITORIA.md](MAPA_MENTAL_Y_AUDITORIA.md).
Revisión estática + pruebas de host; **no hubo prueba física ni corrida Webots
en esta auditoría.**

## 1. Veredicto

La base de ingeniería es buena: seguridad por capas, contrato v2 validado,
núcleo de decisión compartido entre firmware y Webots, pruebas de host y
bitácoras honestas. Pero **hoy el equipo no puede completar una ronda**:

1. **Solo un rover puede moverse.** La IMU del rover 2 falla (prueba A/B del
   22/09) y `ArenaMotion` entra en `FAULT` sin IMU; además su perfil está
   `initially_validated=false`.
2. **Nunca se ha empujado un cubo** con el controlador integrado, ni físico
   ni en Webots con la misma lógica de movimiento.
3. **La estrategia tiene fallos geométricos** que harán fallar entregas aunque
   el movimiento funcione (sección 3, P0-3 a P0-6).

La prioridad es: rover 2 móvil → un cubo entregado de punta a punta → dos
rovers sin bloquearse → ronda completa → paquete de presentación.

## 2. Evidencia ejecutada hoy

| Comprobación | Resultado |
| --- | --- |
| `rover_logic_core` + `tests/test_rover_logic_core.cpp` | OK |
| `tests/run_arena.sh` (movimiento + `setup()/loop()` reales, red on/off) | OK |
| `tests/run_firmware_safety.sh` | OK |
| `pytest webots/tests` | 22 passed |
| `pytest rover_control/tests/test_serial_console.py` | 2 passed |
| `arduino-cli` | No instalado: no se recompiló para ESP32 hoy |

Advertencia no bloqueante: `StaticJsonDocument` está deprecado en ArduinoJson 7.

## 3. Hallazgos priorizados

### P0 — impiden competir

| # | Hallazgo | Evidencia | Corrección |
| --- | --- | --- | --- |
| P0-1 | **Rover 2 inmovilizado.** IMU defectuosa confirmada por intercambio A/B; sin IMU el controlador no arranca. | `imu_comparacion/SESION_2026-09-22.md`; `motion/rover_profile.h:21`; `arena_motion.h:17` | (a) Pedir **hoy** a la organización el reemplazo de la IMU/cable (el reglamento prohíbe sustituirla ustedes). (b) En paralelo, modo **solo-visión** sin IMU: giro por pulsos cortos cerrando el lazo con `theta` de la cámara, avance a PWM fijo. Vale como plan B para cualquier rover si la IMU falla en la ronda. |
| P0-2 | **Empuje no validado.** PWM 18/20 (~6–7 cm/s sin carga) nunca probado con cubo. `TURN_STALL`/`TURN_TIMEOUT` son probables con carga. | `SESION_2026-09-19.md:40`; README "su capacidad con carga todavía debe probarse" | Prueba de banco: 10 empujes rectos de 30 cm con cubo, registrar desvío y pérdida de cubo. Si no empuja, subir PWM de empuje en perfil separado. |
| P0-3 | **La aproximación atraviesa el cubo.** `drive_to(approach)` va en línea recta; si el rover está entre el cubo y el depósito, cruza el cubo y lo empuja en sentido contrario. Tampoco esquiva otros cubos ni cubos ya entregados. | `rover_logic_core.cpp:84-94` | Si el segmento rover→punto de aproximación pasa a menos de ~(radio rover + medio cubo) del cubo, insertar un punto de rodeo lateral. Prueba unitaria con el rover "del lado equivocado". |
| P0-4 | **Punto de aproximación fuera de la cancha.** Con el cubo cerca del borde opuesto al depósito, `approach` queda a 115 mm detrás del cubo, fuera de la cancha; la protección IR está desactivada. Salir de la superficie se penaliza. | `rover_logic_core.cpp:84-86`; `config.example.h` `LINE_STOP_ON_DETECTION false` | Limitar objetivos al área con margen de medio chasis. Si el cubo no es empujable directo, maniobra de 2 tramos: sacarlo primero del borde en ángulo. Prueba con cubo a 1 celda del borde. |
| P0-5 | **Bloqueo por cesión de paso.** El ID mayor se detiene si el otro está a <200 mm, **aunque el otro esté quieto**. Ambos salen del mismo `start`. Si el ID menor queda sin tarea (último cubo asignado al otro) o detenido junto al depósito, el ID mayor queda parado toda la ronda. El ID menor nunca esquiva. | `rover_logic_core.cpp:140-146`; `vision_navigator.cpp:89-95` | Ceder solo si el otro se mueve hacia mí (o su ruta cruza la mía) y con tiempo máximo. El rover sin tarea debe **despejar**: ir a una zona de estacionamiento fuera de rutas/depósitos. Prueba: 1 cubo, rover 10 sin tarea al lado. |
| P0-6 | **Asignación sin memoria.** `assign_tasks` se recalcula en cada vuelta sin histéresis: al moverse los rovers y el cubo, los costos cambian y la tarea puede intercambiarse a mitad de empuje (oscilación, doble persecución de un cubo cuando cada rover ve un mensaje distinto). | `vision_navigator.cpp:85`; `rover_logic_core.cpp:110` | Mantener la tarea mientras sea válida; reasignar solo por entrega, cubo perdido > N s o ganancia de costo > umbral. Idealmente confirmar reserva vía ESP-NOW (P1-1). |
| P0-7 | **Fallas enclavadas = rover muerto en la ronda.** `IMU_STALE`, `TURN_STALL`, `TURN_TIMEOUT` enclavan hasta reiniciar, pero el reglamento prohíbe reiniciar selectivamente durante la ronda. | `arena_motion.h:30,98`; reglamento §8 | Separar fallas: `TURN_STALL`/`TIMEOUT` → retroceso corto y replanificar; IMU caída → degradar a modo solo-visión (P0-1b). Enclavar solo lo que sea peligroso. |

### P1 — reducen mucho la probabilidad de éxito

| # | Hallazgo | Corrección |
| --- | --- | --- |
| P1-1 | **No hay comunicación rover↔rover.** La coordinación es implícita (misma telemetría, mismo algoritmo). Es defendible, pero el README oficial dice que los rovers "deben comunicarse", y el jurado evaluará coordinación. | ESP-NOW mínimo: latido + tarea reservada + fase (APPROACH/PUSH/IDLE). Usarlo para reservas (P0-6) y cesión (P0-5). Si falla el enlace, seguir con el acuerdo implícito actual. |
| P1-2 | **Contradicción del arranque.** El firmware arranca con `phase=RUNNING`; el reglamento dice juez + botón de la IdeaBoard. Pendiente con la organización. | Opción `VRC_REQUIRE_START_BUTTON`: armar con botón **y** exigir `RUNNING`. Sirve con cualquier interpretación. Preguntar por escrito. |
| P1-3 | **Después de entregar no retrocede.** Queda en contacto con el cubo y gira en sitio para el siguiente objetivo; puede sacar el cubo del depósito antes del segundo de permanencia. | Tras `CUBE_DELIVERED`: retroceso recto ~80 mm antes de girar. |
| P1-4 | **Throughput bajo.** 350 ms de avance + 500 ms de reposo + nueva captura ≈ 40 % del tiempo en movimiento, ~2,5 cm/s efectivos. Con 3 cubos cabe en 600 s si nada falla, pero sin margen para reintentos. | Tras validar P0: tramos más largos cuando el error es pequeño y la distancia grande; reposo según el giro. Medir cronómetro real por cubo. |
| P1-5 | **Webots no prueba el controlador físico.** `rover_agent.py` fija velocidades continuas; no ejecuta `ArenaMotion` (paradas, giros por IMU). Solo existe `nominal_3_cubos`, con `physics_calibrated: false`; los otros 7 escenarios de `VALIDACION.md` no existen. | Portar el ciclo parar-observar a Webots o un simulador 2D sencillo en C++ sobre el núcleo; implementar `cruce_de_rutas`, `cubo_oculto_en_empuje`, `fin_de_ronda` y los casos de P0-3/4/5. |
| P1-6 | **Sensores locales sin uso.** IR de borde desactivado (polaridad/posición sin confirmar); ultrasonido midió 0,9 cm contra 20 cm de referencia. | Validar IR en la cancha real y activar parada de borde; el sonar puede quedar desactivado si no es fiable, documentándolo. |
| P1-7 | **Cubo oculto > 1,2 s = rover detenido.** `MAX_CUBE_AGE_MS 1200`; el reglamento pide "continuar razonablemente" con la última posición conocida. | Durante `PUSH` permitir edad mayor (p. ej. 3 s) con avance limitado; fuera de `PUSH`, detenerse. |

### P2 — calidad, entrega y presentación

- **Trabajo sin versionar:** 25 modificados + carpetas nuevas (`motion/`,
  `calibracion_rover2/`, `imu_comparacion/`, pruebas). Hacer commits por tema
  y abrir PR `webots-simulation → main`. Añadir `__pycache__/` a `.gitignore`.
- `diagnostico_rover.ino` borrado y `diagnostico.ino` nuevo: confirmar que
  el renombre es intencional (Arduino exige que el `.ino` y la carpeta coincidan).
- Tres documentos de estado (`MAPA_...`, `PLAN_ACTUAL`, este). Al cerrar P0,
  dejar un solo documento vivo.
- Falta un README del **equipo** (arquitectura, cómo compilar, cómo correr
  una ronda). `README.md` raíz es de la organización; su enlace
  `Vision-Robotic-Challenge` sigue roto.
- Credenciales: `config.h` ignorado. Correcto, verificar antes de cada push.
- `obstacles[]` del contrato se lee pero la navegación no lo usa.

## 4. Plan de corrección

Fechas sin confirmar: falta la fecha oficial de competencia. Cada fase tiene
criterio de salida verificable; no avanzar sin cumplirlo.

| Fase | Trabajo | Criterio de salida |
| --- | --- | --- |
| **0. Hoy** | Solicitud escrita a la organización: reemplazo de IMU rover 2 y aclaración del arranque (P1-2), duración de ronda y fecha. Commits del trabajo actual. | Correo enviado; árbol limpio. |
| **1. Rover 2 móvil** | Modo solo-visión (P0-1b) con pruebas de host; calibrar mapa de ruedas del rover 2 con `calibracion_rover2`. | Rover 2 gira ±90° y avanza 50 cm en cancha solo con cámara, 5 de 5. |
| **2. Un cubo de punta a punta** | P0-3, P0-4, P1-3 en el núcleo con pruebas unitarias; prueba de empuje P0-2; recuperación P0-7. | Rover 1 entrega 1 cubo desde 3 posiciones iniciales (incluido "lado equivocado" y cubo a 1 celda del borde), 4 de 5 corridas, con traza serial guardada. |
| **3. Dos rovers** | P0-5, P0-6, P1-1 (ESP-NOW mínimo). Escenarios de cruce y "rover sin tarea". | Dos rovers, 2 cubos, sin contacto y sin bloqueo de más de 10 s, 3 de 3. |
| **4. Ronda completa** | 3 cubos en configuración oficial; P1-4, P1-6, P1-7. | 3/3 cubos dentro de 600 s en 3 de 5 corridas; tiempos registrados. |
| **5. Congelar** | Etiqueta `v-competencia`, binarios guardados por rover, checklist pre-ronda. | Checklist ejecutada en seco dos veces sin fallos. |

Regla de congelamiento: después de la fase 5, solo correcciones de fallos
reproducidos, con prueba que los cubra.

## 5. Plan de presentación

Entregables exigidos (`robot.md` §Responsabilidad): **código fuente,
documentación técnica y demostración autónoma funcional.**

### 5.1 Paquete técnico (en el repo)

1. `README` del equipo: problema → arquitectura → cómo compilar/cargar →
   cómo operar una ronda → resultados.
2. Diagrama de arquitectura: visión oficial → TCP/NDJSON v2 → `TelemetryClient`
   → `WorldState` → `SafetySupervisor` → núcleo (asignación, cesión,
   aproximación/empuje) → `ArenaMotion` (IMU) → motores; ESP-NOW entre rovers.
3. **Matriz de cumplimiento del reglamento**: cada restricción de §4, §6, §7,
   §8 y §12 → cómo la cumple el diseño y la evidencia (p. ej. "sin control
   externo": el firmware decide sin comandos serial; banco de motores y
   `VRC_NETWORK_ONLY` desactivados en la compilación de competencia; la
   laptop solo ejecuta la visión oficial).
4. Evidencia: pruebas automáticas (cómo correrlas), bitácoras de sesión,
   tabla de corridas de las fases 2–4 con tiempos y fallos.
5. Limitaciones conocidas, dichas con honestidad (sin encoders, IMU solo en
   tramos, sonar no fiable). Al jurado le da confianza que se reconozcan.

### 5.2 Presentación oral (≈10 min, 8–10 láminas)

1. El reto en una lámina · 2. Arquitectura · 3. Seguridad por capas (datos
viejos, pose perdida, FINISHED, fallas) · 4. Estrategia: asignación, cesión,
empuje · 5. Control de movimiento con IMU (datos de calibración reales) ·
6. Simulación y pruebas (Webots + host) · 7. Resultados medidos · 8.
Problemas encontrados y cómo se resolvieron (IMU del rover 2 con A/B es una
buena historia de ingeniería) · 9. Trabajo futuro.

### 5.3 Demo

- Video de respaldo grabado de la mejor corrida de la fase 4 (y una de Webots).
- Guion: encender quietos (cero IMU) → confirmar `wifi=ok tcp=ok` → colocar
  en salida → señal del juez → manos fuera.
- Plan B en vivo: si un rover falla, el otro debe poder completar solo
  (la asignación ya lo permite si la pose del otro vence; verificarlo en fase 3).

### 5.4 Checklist pre-ronda (borrador)

Batería cargada · firmware `v-competencia` en ambos · `ROVER_ID` 10/11 y
`VRC_PHYSICAL_ROVER` correctos · `VISION_HOST` verificado (IP DHCP) · banco
de motores y `NETWORK_ONLY` en 0 · cero IMU con rover quieto (`READY`) · pose
de ambos visible con edad < 300 ms · cable USB desconectado.

## 6. Decisiones pendientes del equipo

- ¿Se implementa ESP-NOW (P1-1) o se defiende la coordinación implícita?
  Recomiendo el mínimo: bajo costo, y responde directamente a lo que se evalúa.
- ¿Se prioriza el modo solo-visión aunque llegue la IMU nueva? Recomiendo sí:
  también cubre una falla de IMU durante la ronda.

## 7. Avance del 23/09: simulador y correcciones de lógica

Nuevo `rover_control/sim/` ([README](rover_control/sim/README.md)): ejecuta el
`setup()/loop()` real de los dos rovers contra física, visión y árbitro
simulados. Con él se confirmaron y corrigieron en el núcleo compartido:

| Hallazgo | Estado | Cómo |
| --- | --- | --- |
| P0-3 atraviesa el cubo | Corregido | Planificador A* en la grilla (cubos y rover inflados) |
| P0-4 fuera de la cancha | Corregido | Área segura; empuje en diagonal para despegar el cubo del borde |
| P0-5 bloqueo por cesión | Corregido | Paso para quien empuja (visto por cámara); el rover sin tarea se estaciona |
| P0-6 asignación sin memoria | Mejorado | Histéresis, corredores en conflicto no simultáneos; falta ESP-NOW |
| P0-7 fallas enclavadas | Corregido parcial | Giro trabado se reintenta (4 seguidos enclavan); IMU sigue enclavando |
| P1-3 no retrocede | Corregido | Retroceso comprometido de 80 mm tras entregar o abortar |
| P1-7 cubo oculto | Corregido | 3 s de tolerancia solo durante el empuje |
| **Nuevo:** geometría de empuje | Corregido | Contacto real a 107 mm (antes 80): el cubo quedaba fuera de la ventana de ±32 mm |
| **Nuevo:** oscilaciones de decisión | Corregido | Máquina de estados IR→ALINEAR→EMPUJAR→RETIRARSE con dirección y tramo congelados |
| **Nuevo:** giro barre cubos/rovers | Corregido | No girar en sitio a <135 mm de un cubo ni <200 mm de un rover |
| **Nuevo:** guarda de contacto | Agregado | Predice el chasis tras cada orden; ninguna orden que toque otro rover sale |

Resultado (10 semillas por escenario, física perturbada ±10 %): **125/150
corridas**, con **0 contactos, 0 salidas y 0 movimientos indebidos**. Pasan
10/10 un cubo, lado equivocado, cubo en borde, rover sin tarea, pérdida de
pose, TCP intermitente, cubo oculto, fin de ronda, desborde de `micros()` y
`nominal_3_cubos` (mediana 204 s). Débiles: `aleatorio` (2–3/10, 20/30 cubos),
`aleatorio_un_rover` (4/10) y `falla_imu` (5/10).

**Lectura honesta.** La seguridad está sólida; la productividad con 3 cubos
aleatorios todavía no. Las fallas restantes son atascos locales (rover
encerrado entre cubos cercanos) y cubos que quedan pegados a un borde. No
seguir ajustando umbrales a ciegas: medir primero en la arena (velocidad
empujando, arrastre del cubo, latencia) y recalibrar el simulador.

Cambios de configuración: `NAV_PUSH_CONTACT_OFFSET_MM 107`,
`NAV_APPROACH_STANDOFF_MM 150`, `NAV_APPROACH_TOLERANCE_MM 40`,
`MAX_PUSH_CUBE_AGE_MS 3000`, `NAV_ARENA_MARGIN_MM 60`. Un `config.h` con el
contacto anterior (80) ya no compila (`static_assert`). Los rovers deben salir
con ≥200 mm entre centros. Las pruebas existentes se actualizaron a los
comportamientos nuevos y pasan. Compilación ESP32 (core 3.3.11, `arduino-cli`
del Arduino IDE en `/opt/arduino-ide/resources/app/lib/backend/resources/`):
970 683 bytes de flash (74 %) y 97 132 bytes de RAM global (29 %; el A* suma
~45 KB). No se cargó a ningún rover.

Pendiente del plan: modo solo-visión sin IMU, ESP-NOW mínimo, arranque con
botón, y la hoja de mediciones para el viernes.

## 8. Ritmo parar-observar (23/09, tarde)

Primer punto de la hoja de ruta de `ESTADO_DEL_ARTE.md`. La pausa tras cada
maniobra y el tramo máximo ahora se configuran con `ARENA_SETTLE_MS` y
`ARENA_MAX_DRIVE_MS` (`hardware_config.h`). Los valores por defecto pasan de
500/350 ms a **150/1000 ms**. Para volver al ritmo anterior basta
`#define ARENA_SETTLE_MS 500U` y `#define ARENA_MAX_DRIVE_MS 350U` en `config.h`.

Se eligió comparando cinco ritmos, con 10 semillas × 15 escenarios cada uno y
visión nominal. Además se repitió todo con 100 ms extra de latencia de visión
(`--vision-delay-ms 100`), porque una pausa corta decide con una pose más vieja.

| Ritmo (ms) | PASS (nominal / +100 ms) | 1 cubo | Diagonal | 3 cubos, 2 rovers | Cruce |
| --- | --- | --- | --- | --- | --- |
| 500/350 (anterior) | 131 / 135 | 34,6 s | 51,9 s | 153,9 s | 179,2 s |
| **150/1000 (nuevo)** | **130 / 130** | **13,4 s** | **21,4 s** | **71,1 s** | **112,7 s** |

(Tiempos: medianas con visión nominal.)

Todas las corridas tuvieron **0 contactos y 0 salidas de superficie**. Las
fallas restantes son las mismas familias de antes: `aleatorio`,
`aleatorio_un_rover` y `falla_imu`.

Dos hallazgos que destapó el ritmo nuevo:

- **El rover seguía un cubo que resbalaba por la pared hasta salir de la
  superficie** (`aleatorio` semilla 8, también con 200/700 y 100/1000). Se
  agregó una guarda de superficie: en el empuje, el rover se retira si el tramo
  lo saca del margen, y como red final ninguna orden recta sale del margen
  hacia afuera. Hay pruebas unitarias, y se verificó que fallan sin la guarda.
- **`falla_imu` bajó de 5/10 a 0/10, pero no por el ritmo.** A los 10 s el
  rover averiado ya avanzó más y queda detenido justo detrás del cubo verde,
  en el punto desde donde hay que empujarlo. Con el ritmo anterior y la falla a
  los 16 s también da 0/10. La debilidad real: **un rover detenido junto al
  punto de empuje bloquea ese cubo.** Queda pendiente, y es independiente del
  ritmo. Un intento de permitir empujes diagonales junto a un rover detenido
  (chequeo de chasis en vez de círculo) no alcanzó y se revirtió.

**Riesgo para el viernes.** El simulador no modela el desenfoque de la imagen
en movimiento ni el deslizamiento del cubo a mayor ritmo. En la arena hay que
correr una prueba A/B con 500/350 contra 150/1000 midiendo el tiempo y el
error de entrega. Si la pose llega tarde (realineos repetidos, `PUSH_ALIGN`
oscilando), subir primero `ARENA_SETTLE_MS` a 200–300.

Herramientas nuevas del simulador:

- `SIM_DEFINES="-DARENA_SETTLE_MS=200U ..." bash sim/build.sh` compila con
  otros parámetros sin tocar la configuración.
- `--vision-delay-ms` agrega latencia de visión para pruebas de estrés.

Compilación ESP32: 971 267 bytes (74 %) y 97 140 bytes de RAM (29 %).

## 9. Cuatro frentes sin arena (23/09, noche)

Campaña de 10 semillas: en los mismos 15 escenarios de antes pasan **140/150**
(antes 130), con 0 contactos y 0 salidas de la superficie. Con los 5
escenarios nuevos son 170/200. `nominal_3_cubos` tiene una mediana de 66,5 s.

**1. Modo solo-visión sin IMU (`ARENA_VISION_ONLY_FALLBACK 1`).** Si el
giroscopio deja de responder, o no está al arrancar, el rover ya no queda en
FAULT. Sigue con maniobras cortas medidas por tiempo:

- giros a 25 % de hasta 45°, calculados con 60 °/s (la tasa medida más alta,
  para quedarse corto);
- tramos de hasta 500 ms;
- la cámara corrige tras cada maniobra, con la misma pausa y exigencia de
  captura nueva de siempre.

Un perfil sin validar sigue sin moverse, y con la opción en 0 se enclava FAULT
como antes. Resultado: `falla_imu` pasa de 0/10 a **10/10** (mediana ~48 s).
Hay pruebas del paso a solo-visión, del giro a tiempo, del tope de 45°, del
tramo de 500 ms y de la IMU ausente al arrancar.

**2. Rover detenido junto al punto de empuje: sigue abierto.** Se agregaron un
evento `MOTOR_FAIL` y dos escenarios (`rover_detenido`, en la posición real
de la falla, y `rover_detenido_en_linea`). Los dos dan 0/10. Se probaron
tres cosas:

- revisar el chasis a lo largo de todo el tramo de empuje;
- correr el cubo de lado;
- empujar solo hasta donde el tramo está libre, con geometría de caja real y
  margen de 8 mm para rovers detenidos.

El caso en línea llegó a mover el cubo de lado, pero en 20 semillas el
conjunto no mejoró (276 contra 274 corridas) y **se revirtió**. Hallazgo: en
la posición real, el rover muerto queda bajo la zona de entrada del depósito
verde, y ningún empuje hacia el norte cabe entre su chasis y la ventana
lateral del depósito. Resolverlo exige planificar varias etapas (reubicar el
cubo y entrar en diagonal), no ajustar umbrales.

**3. ESP-NOW mínimo (`peer_link`, `VRC_ENABLE_ESPNOW 1`).** Cada rover
difunde a 10 Hz un paquete de 10 bytes:

- firma, versión y equipo (`ESPNOW_TEAM_ID`);
- id y número de secuencia;
- cubo reclamado y etapa;
- si puede moverse (sin FAULT ni paro de emergencia).

El paquete lleva además una suma de verificación. Es **solo informativo**:
uno que no puede moverse suelta sus cubos al instante, y el cubo que otro
está empujando no se le disputa. Los datos vencen a los 500 ms, y sin
paquetes todo funciona como antes. Los paquetes no van cifrados (son de
difusión): uno falso solo empeora el reparto de cubos, nunca mueve un rover.

En el escenario `companero_en_falla` (paro de emergencia con la tarea a los
3 s), el compañero entrega en **44 s** de mediana con enlace y en 57 s sin
él. El simulador transporta los paquetes con 5 % de pérdida, y hay pruebas
del protocolo, del eco propio, del vencimiento y del ritmo de envío. La
consola serie muestra `[espnow] compañero=…`.

**4. Botón de arranque (`VRC_REQUIRE_START_BUTTON 0` por defecto).** Con 1,
el rover solo se mueve si alguien presionó BOOT (IO0, con antirrebote de
50 ms) **y** la visión está en RUNNING. El armado dura hasta FINISHED.
Escenario `boton_arranque` y pruebas incluidos. Queda pendiente preguntar a
la organización cómo interpretan el arranque.

**Hoja del viernes:** `rover_control/diagnostico/HOJA_MEDICIONES_VIERNES.md`,
con el registrador `diagnostico/registro_vision.py`. Graba la telemetría y
resume velocidad, giros, latencia y distancia de contacto; se validó contra
el publicador simulado y con datos sintéticos.

Compilación ESP32: 979 875 bytes (74 %) y 97 292 bytes de RAM (29 %). No se
cargó a ningún rover.

