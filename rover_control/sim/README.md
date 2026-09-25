# Simulador 2D de ronda

Ejecuta el **firmware real** de cada rover (`setup()`/`loop()` de
`rover_control.ino`, sin cambios) contra una cancha cinemática, una visión
simulada y un árbitro independiente. Sirve para probar lógica, coordinación y
fallos sin la arena: una ronda de 600 s tarda ~1 s.

No reemplaza la arena: la física del empuje y las latencias son supuestos
(ver abajo). Lo que el simulador demuestra es que **las decisiones** del
firmware son correctas frente a situaciones concretas y reproducibles.

## Uso

```sh
bash rover_control/sim/build.sh              # ArduinoJson en ~/Arduino/libraries por defecto
cd rover_control/sim/build
./vrc_sim --list                             # catálogo de escenarios
./vrc_sim --exact                            # todos, física nominal, semilla 1
./vrc_sim --scenario aleatorio --seeds 20    # campaña con física perturbada ±10 %
./vrc_sim --scenario nominal_3_cubos --trace ronda.csv --serial-dir logs/
python3 ../trace_viewer.py ronda.csv         # ronda.html: repetición animada
./vrc_sim --seeds 10 --vision-delay-ms 100   # estrés: visión 100 ms más lenta
```

Para probar otros parámetros del firmware sin tocar `config.example.h`:

```sh
SIM_DEFINES="-DARENA_SETTLE_MS=500U -DARENA_MAX_DRIVE_MS=350U" \
  bash rover_control/sim/build.sh "$HOME/Arduino/libraries/ArduinoJson/src" /tmp/sim_lento
```

Sale con código 0 solo si todas las corridas pasan. `--json` imprime una línea
por corrida para procesar resultados.

## Cómo funciona

- `build.sh` compila el firmware dos veces (ROVER_ID 10 y 11, a partir de
  `config.example.h`) como bibliotecas compartidas con visibilidad oculta:
  cada rover tiene sus propios globales dentro del mismo proceso.
- `host/` reemplaza Arduino, Wi-Fi/TCP, I²C (LSM6DS3 a 104 Hz) y PWM.
- `sim.cpp` integra la física a 1 ms, publica telemetría v2 a 20 Hz con
  latencia, ruido y oclusiones, y arbitra con la regla del contrato (media
  diagonal, 1 s de permanencia) sobre la **verdad física**.
- Ambos rovers usan el perfil de movimiento 1 hasta calibrar el rover 2.

## Eventos de los escenarios

`CUBE_HIDE`, `CUBE_HIDE_NEAR`, `ROVER_HIDE`, `TCP_DOWN`, `IMU_FAIL`,
`MOTOR_FAIL` (el rover no se mueve aunque lo ordene), `ESTOP` (`!` por serie),
`ESPNOW_DOWN` y `BUTTON` (BOOT presionado; solo importa con
`VRC_REQUIRE_START_BUTTON=1`). ESP-NOW se transporta entre rovers con 5 % de
pérdida. Los escenarios marcados ABIERTO documentan un problema sin resolver.

## Qué cuenta como falla

- **Faltas críticas:** contacto entre rovers, salida de la superficie,
  movimiento antes de `RUNNING`, después de `FINISHED` o con la pose propia
  vencida.
- **Faltas de resultado:** entregas incompletas, cubo fuera de la cancha o
  cubo sacado de su zona después de entregado.

## Modelo físico y supuestos

Medido (rover 1, `diagnostico/SESION_2026-09-19.md`): ~64 mm/s a 20/20, curva
a la derecha, giro en sitio que no arranca con 18/20 y que va a 46–62 °/s con
25 %, y un residual de 2–3° al cortar.

**SUPUESTO** (medir en la arena y actualizar `Physics`/`Vision` en `sim.cpp`):

- velocidad empujando un cubo (`push_factor`);
- arrastre lateral del cubo contra las paletas (`cube_mu`);
- latencias de visión y Wi-Fi, y ruido de pose.

El chasis es la caja de Webots (120×100 mm, paletas a 77 mm del eje); el cubo
se modela como disco de 30 mm.

## Colocación de salida

Los rovers deben salir con **≥200 mm entre centros**. Más juntos, el primer
giro en sitio (barrido de ~92 mm por rover) los hace chocar.
