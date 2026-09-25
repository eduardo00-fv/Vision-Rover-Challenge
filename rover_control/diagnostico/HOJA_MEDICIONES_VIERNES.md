# Hoja de mediciones — viernes 25/09

Objetivo: reemplazar los supuestos del simulador por números medidos y
decidir cuánto se puede acelerar. Cada prueba dice qué valor ajusta.
Orden = prioridad: si el tiempo no alcanza, se cortan las últimas.

**Modo asistido (recomendado).** Con la visión corriendo en esta laptop y el
rover 1 por USB:

    python3 rover_control/diagnostico/sesion_viernes.py

Detecta IP, puerto y marcador; carga el firmware de banco, prueba `!`, corre
1→4 y 6 (y calcula 5), la prueba IR (sección 11), carga el autónomo y corre la
7 con los dos ritmos (3 rondas cada uno). El USB solo se usa para cargar; las
pruebas van por Wi-Fi. Solo IR: `sesion_viernes.py ir`. Solo pide lo físico (cubo en las paletas, reubicar, `r` en
la visión). Ctrl+C detiene el rover; al volver a correrlo sigue donde quedó.
Todo queda en `diagnostico/sesiones/<fecha>/RESULTADOS.md` con las tablas de
abajo llenas. 8, 9 y 10 siguen siendo manuales.

**Antes de empezar (modo manual)**

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

## 11. IR sobre el tablero de ajedrez  → `LINE_*` en `config.h`

La cancha es un tablero de ajedrez: leer negro no es borde. `line_guard.h`
declara un sensor fuera cuando se alejó más de `LINE_EDGE_MM` del lugar donde
cambió de color por última vez (sobre la cancha cambia cada 20-28 mm). Cerca de
las direcciones 45° y 26,6°/63,4° el umbral se duplica: ahí un sensor que roza
esquinas ve rachas largas de un color. Es una red de seguridad que detecta
tarde (el sensor ya pasó 40-85 mm el borde, según el ensayo), no una parada
fina: la que evita salir es el margen de navegación con la cámara.

El asistente lo hace en tres partes:

1. **Quieto** (3 pedidos): sobre el tablero, sobre la lona y en el aire → polaridad.
2. **Recorrido automático** (~40 s): rectas en 3 rumbos y giros. Con la pose
   de la cámara ubica cada sensor en el chasis, detecta sensores trabados y
   mide la racha más larga de un color → `LINE_EDGE_MM`.
3. **Borde** (2 pedidos): rover a ~10 cm del borde mirando hacia afuera,
   derecho y en diagonal; avanza de a 2-3 cm hasta que el guardia detecta y
   vuelve solo. **Quedarse cerca para agarrarlo.**

Deja en `RESULTADOS.md` el bloque de `config.h` listo para copiar. Solo
sugiere `LINE_STOP_ON_DETECTION true` si detectó el borde en las dos pasadas.

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
| 11 | `LINE_STOP_ON_DETECTION`, `LINE_FRONT_MASK`, `LINE_REAR_MASK`, `LINE_EDGE_MM`, `LINE_SENSOR_RADIUS_MM` en `config.h` |

Con los números cargados, volver a correr la campaña del simulador
(`./vrc_sim --seeds 10`) antes de la siguiente sesión en la arena.
