# Comparación de controladores I²C

Diagnóstico mínimo de banco para Arduino ESP32 3.3.11 y 2.0.17, placa
`esp32:esp32:esp32`. Usa el mismo `Lsm6ds3Gyro` del proyecto. No requiere
FastLED ni inicializa PWM; mantiene GPIO 12/14/13/15 en LOW.

Serial 115200. `INFO` identifica core, SDK y chip. `RUN` detecta/configura la
IMU, espera 250 ms y recoge 30 s de muestras nuevas. `!` cancela. Se detiene
al primer fallo I²C o si pasan más de 100 ms sin datos nuevos. `RESULT OK`
significa únicamente que completó esa ventana: no certifica calibración,
exactitud ni funcionamiento bajo movimiento. Media/mínimo/máximo solo tienen
sentido como medición de reposo si el operador mantuvo quieto el conjunto.

Condiciones iguales para ambas compilaciones: SDA 21, SCL 22, 100 kHz,
timeout 25 ms, gyro 104 Hz y ±250 dps, lectura de STATUS y XYZ, sondeo cada
2 ms. El timeout de esta comparación es más permisivo que los 5 ms del
firmware normal; no se trasladó al controlador del rover.

Comparar el mismo sensor, cable, placa y alimentación. Desconectar USB y
batería entre variantes y confirmar apagado completo antes de cada corrida.
La instalación 2.0.17 de esta sesión vive en `/tmp/rover-imu-core2`, separada
de la instalación habitual. Cambiar de core compara también su SDK y
compilador; una diferencia de resultado no identifica por sí sola un bug
concreto del driver.

Al compilar con core 2, pasar `--build-property compiler.cpp.extra_flags=-I<RUTA_ABSOLUTA>/rover_control/imu_comparacion`
para resolver el encabezado compartido desde la carpeta temporal del build.

Restauración preparada con el código de calibración disponible al inicio:
`/tmp/rover-imu-restore` (core 3.3.11). Es una recompilación, no una copia
binaria extraída de la placa. Los archivos en `/tmp` son temporales.
