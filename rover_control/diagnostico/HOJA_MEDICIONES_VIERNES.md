# Hoja de mediciones — viernes 25/09

Objetivo: reemplazar los supuestos del simulador por números medidos y
decidir cuánto se puede acelerar. Cada prueba dice qué valor ajusta.
Orden = prioridad: si el tiempo no alcanza, se cortan las últimas.

**Antes de empezar**

- [ ] Visión corriendo y cancha calibrada; `!` por serie detiene al rover (probarlo primero).
- [ ] En la PC de visión, un registro por prueba, con un nombre de archivo por prueba:
      `python3 rover_control/diagnostico/registro_vision.py grabar --host 127.0.0.1 --salida <prueba>.csv`
- [ ] Después de cada prueba: `... registro_vision.py resumen <prueba>.csv --rover <id>`
- [ ] Anotar qué rover físico (1 o 2) y qué marcador (10 u 11) se usó en cada una.

## 1. Velocidad contra PWM, sin cubo  → `k_mm_s` del simulador, `kNominalMmPerS`

Firmware con `VRC_ENABLE_MOTOR_BENCH 1`. El banco permite hasta 35 %.
Por serie: `ARM` y luego `MOTOR <izq> <der> 1500`; `ARM` otra vez antes de cada orden.

| PWM izq/der | Rover | v media (mm/s) | Giro total (°) en 1,5 s | Notas |
| --- | --- | --- | --- | --- |
| 18 / 20 | | | | referencia del 19/09: ~64 mm/s |
| 25 / 27 | | | | |
| 30 / 33 | | | | |
| 35 / 35 | | | | |

¿Se sostiene la recta? Si el giro total pasa de ~5°, ajustar la relación izq/der.

## 2. Lo mismo empujando un cubo  → `push_factor`

Cubo apoyado en las paletas antes de arrancar.

| PWM | v con cubo (mm/s) | v sin cubo (fila de arriba) | Factor = con/sin | ¿El cubo se despegó? |
| --- | --- | --- | --- | --- |
| 18 / 20 | | | | |
| 25 / 27 | | | | |
| 30 / 33 | | | | |

## 3. Curva con cubo: ¿hasta dónde no resbala?  → `cube_mu`, empuje curvo

`MOTOR <izq> <der> 1500` con el cubo en las paletas. Anotar si el cubo sigue
centrado, se corre de lado (cuántos mm) o se sale.

| Izq / der | Giro (°/s, del resumen) | Corrimiento lateral del cubo | ¿Se salió? |
| --- | --- | --- | --- |
| 25 / 22 | | | |
| 25 / 18 | | | |
| 30 / 20 | | | |

## 4. Giro en sitio a 25 %  → `kBlindTurnDps` (modo sin IMU, hoy 60 °/s)

`MOTOR -25 25 1000` y `MOTOR 25 -25 1000`, tres veces cada uno.

| Sentido | °/s (1) | °/s (2) | °/s (3) | Residual al cortar (°) |
| --- | --- | --- | --- | --- |
| Izquierda | | | | |
| Derecha | | | | |

`kBlindTurnDps` debe ser **la tasa más alta medida**: así el giro a ciegas
queda corto y la cámara lo completa, en vez de pasarse.

## 5. Latencia de la visión  → `proc_ms` del simulador; decide `ARENA_SETTLE_MS`

De cualquier registro: la línea `latencia captura→recepción` del resumen.
Mediana: ______ ms   p95: ______ ms.
Si la p95 pasa de ~150 ms, la pausa de 150 ms decide con una pose del
movimiento anterior: subir `ARENA_SETTLE_MS`.

## 6. Distancia de contacto rover–cubo  → `NAV_PUSH_CONTACT_OFFSET_MM` (hoy 107)

Cubo apoyado en las paletas, rover quieto, 10 s de registro. Repetir girando el rover 90°.
`registro_vision.py contacto <prueba>.csv --rover <id> --color <color>`

| Toma | Mediana (mm) | Mín | Máx |
| --- | --- | --- | --- |
| 1 | | | |
| 2 (girado 90°) | | | |

Esto decide si el cubo queda dentro de la ventana de ±32 mm de la zona.

## 7. Ritmo: 500/350 contra 150/1000  → `ARENA_SETTLE_MS`, `ARENA_MAX_DRIVE_MS`

Un rover, un cubo, misma posición inicial. Tres rondas con cada ritmo. Para
el ritmo anterior, en `config.h`: `#define ARENA_SETTLE_MS 500U` y
`#define ARENA_MAX_DRIVE_MS 350U`.

| Ritmo | Tiempo (s) 1 | 2 | 3 | ¿Cubo entregado? | Error final en la zona | ¿Realineos repetidos? |
| --- | --- | --- | --- | --- | --- | --- |
| 500 / 350 | | | | | | |
| 150 / 1000 | | | | | | |

Simulador: 34,6 s contra 13,4 s. Si con 150/1000 aparecen realineos en
`PUSH_ALIGN`, probar 250/1000.

## 8. Modo sin IMU

Con el rover **apagado**, desconectar el cable de la IMU; luego encender.
La consola debe mostrar `imu=IMU_ABSENT`. En RUNNING con un cubo:

- [ ] Se mueve con maniobras cortas y entrega el cubo.   Tiempo: ______ s
- [ ] No choca ni sale de la cancha.
- [ ] Reconectar la IMU (apagado) y confirmar que vuelve a `imu=ok`.

## 9. ESP-NOW entre los dos rovers

Ambos encendidos y conectados a la visión.

- [ ] La consola de cada uno muestra `[espnow] compañero=<id> objetivo=… etapa=… puede_moverse=1`.
- [ ] Con un cubo en RUNNING, `!` en el rover que lo tiene: el otro muestra
      `puede_moverse=0` y toma el cubo en pocos segundos (sin enlace, ~20 s).
- [ ] Si dice `sin datos (enlace=1)`, revisar que ambos estén en el mismo AP (mismo canal).

## 10. Botón de arranque (solo si se decide usarlo)

`#define VRC_REQUIRE_START_BUTTON 1` en `config.h`.

- [ ] En RUNNING sin presionar: no se mueve y la consola muestra `[arranque] armado=0`.
- [ ] Presionar BOOT: `[arranque] botón presionado: armado` y arranca.
- [ ] Al terminar la ronda se desarma; la siguiente exige presionar otra vez.

## Después: cargar los números

| Medición | Dónde se carga |
| --- | --- |
| 1 | `sim/sim.cpp` `Physics::k_mm_s`; `kNominalMmPerS` en `motion/arena_motion.h` |
| 2 | `Physics::push_factor` |
| 3 | `Physics::cube_mu` |
| 4 | `kBlindTurnDps` en `motion/arena_motion.h` |
| 5 | `Vision::proc_ms` y `net_*_ms`; `ARENA_SETTLE_MS` |
| 6 | `NAV_PUSH_CONTACT_OFFSET_MM` en `hardware_config.h` |
| 7 | `ARENA_SETTLE_MS` / `ARENA_MAX_DRIVE_MS` |

Con los números cargados, volver a correr la campaña del simulador
(`./vrc_sim --seeds 10`) antes de la siguiente sesión en la arena.
