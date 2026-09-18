# Rover Control — ESP32 / Arduino C++

Base de control para cada CenfoBot. Esta primera entrega implementa:

- Conexión Wi‑Fi al hotspot de visión.
- Cliente TCP no bloqueante al puerto oficial `2026`.
- Reconstrucción de NDJSON: una línea JSON por mensaje.
- Validación de telemetría v2 y actualización atómica de `WorldState`.
- Control diferencial de los motores N20, sin encoders.
- Navegación visual en lazo cerrado: cada PWM se recalcula desde la pose fresca
  de la cámara, sin integrar distancia local.
- Lectura de cuatro sensores de línea, ultrasónico y color reflectivo.
- FSM mínima que entra en parada segura con visión vieja, pose propia vencida o
  ronda finalizada.

El control físico todavía requiere calibrar sentido de ruedas, PWM mínimo y
márgenes de aproximación en ambos rovers antes de usarlo en una ronda.

## Preparación

1. Instalar **esp32 by Espressif Systems** en Arduino IDE.
2. Instalar las bibliotecas **ArduinoJson** y **FastLED** desde el administrador
   de bibliotecas.
3. Copiar `config.example.h` como `config.h`.
4. Configurar el hotspot, IP de la laptop y `ROVER_ID` (`10` o `11`).
5. Abrir `rover_control.ino`, elegir la placa/puerto del IdeaBoard y cargar.

`config.h` está ignorado por Git para no guardar contraseñas ni la configuración
de una cancha específica.

## Arquitectura del firmware

El firmware se organiza por módulos, sin copiar el código de referencias
externas:

- `telemetry_client.*` recibe y valida el contrato oficial de visión.
- `world_state.*` representa una foto inmutable del mundo recibido.
- `safety_supervisor.*` es la autoridad que permite o bloquea movimiento.
- `mission_controller.*` traduce las fases oficiales a estados de misión.

`motor_controller`, `sensors` y `vision_navigator` completan la capa local.
Ningún módulo podrá emitir PWM sin que `safety_supervisor` haya autorizado
movimiento.

## Núcleo portable de navegación

[`rover_logic_core.h`](rover_logic_core.h) y
[`rover_logic_core.cpp`](rover_logic_core.cpp) contienen la geometría de
aproximación/empuje y el criterio de entrega sin depender de Arduino, Wi-Fi ni
Webots. Es la pieza que se reutilizará en el controlador de simulación y que el
adaptador del firmware consumirá después. La regla de depósito usa la media
diagonal del cubo, igual que el contrato v2.

Se puede verificar sin placa ni IDE:

```bash
g++ -std=c++17 rover_logic_core.cpp tests/test_rover_logic_core.cpp -o /tmp/test_rover_logic_core
/tmp/test_rover_logic_core
```

## Cableado definido por el firmware

| Componente | GPIO |
| --- | --- |
| Motor izquierdo | M1 interno: 12 / 14 |
| Motor derecho | M2 interno: 13 / 15 |
| Línea (frente izq., frente der., atrás izq., atrás der.) | 36, 39, 34, 35 |
| Ultrasónico | TRIG 25, ECHO 26 |
| Color reflectivo | NeoPixel iluminador 4, analógico 32 |

El pin `ECHO` de un HC-SR04 típico es de 5 V: use un divisor de voltaje antes
de GPIO26, que es de 3.3 V. Antes de cargar cada rover, ajuste `ROVER_ID`,
`ROVER_TARGET_COLOR` y los cuatro umbrales de línea en `config.h`.

### Contrato de visión que consume

Se programa contra `vision-system/contrato/CONTRATO.md`, actualmente protocolo
v2: TCP, puerto 2026, NDJSON y comunicación de solo lectura desde la visión.
Las posiciones `col` y `row` llegan en celdas; cualquier control físico deberá
convertir con `world.grid.cell_mm`, nunca asumir 20 mm. `row` aumenta hacia
abajo, por lo que el vector de avance usa `drow = -sin(theta)`.

Antes de añadir o modificar navegación, ejecutar en VS Code la tarea
`Vision: revisar contrato vigente`. Muestra el último cambio confirmado del
contrato y los cambios locales pendientes bajo `vision-system`.

## VS Code

El repositorio incluye tareas en `.vscode/tasks.json`. Requieren que
[`arduino-cli`](https://arduino.github.io/arduino-cli/latest/installation/) esté
instalado y disponible en el `PATH`, con el core `esp32` y la biblioteca
**ArduinoJson** instalados.

Desde **Terminal → Run Task** (o `Ctrl+Shift+B`) usa:

- `Rover: compilar` para verificar el sketch.
- `Rover: compilar y cargar` para compilar y subirlo a la IdeaBoard.
- `Rover: monitor serial (115200)` para ver sus mensajes.

Las tareas solicitan el FQBN y el puerto. El valor inicial del FQBN es
`esp32:esp32:esp32`; cámbialo si tu IdeaBoard identifica otro modelo. En Linux,
el puerto suele ser `/dev/ttyUSB0` o `/dev/ttyACM0`.

## Prueba inicial

1. Levantar el simulador desde `vision-system/contrato`:

   ```bash
   python3 mock_publisher.py
   ```

2. Conectar laptop y ESP32 al mismo hotspot/red. En `config.h`, `VISION_HOST`
   debe ser la IP de la laptop, nunca `127.0.0.1`.
3. Abrir el monitor serial a 115200 baud. Deben aparecer conexión Wi‑Fi/TCP,
   la pose del rover configurado y contadores de mensajes.

## Banco de motores sin cámara

Para una prueba mecánica aislada, en una copia local de `config.h` cambie
`VRC_ENABLE_MOTOR_BENCH` a `1` y cargue **el mismo** `rover_control.ino`. Este
modo no inicia Wi-Fi, visión ni misión; solo habilita el motor después de
escribir `ARM` en el monitor serial. Cada orden queda limitada a ±35 % PWM y
1500 ms:

```text
ARM
MOTOR 20 20 300      # avance breve
MOTOR 20 -20 300     # giro breve
STOP
DISARM
```

Pruebe un rover a la vez, con espacio libre y una persona lista para cortar
alimentación. Vuelva `VRC_ENABLE_MOTOR_BENCH` a `0` antes de cualquier ensayo
con visión.

## Regla de seguridad actual

Fuera del modo de banco, el programa conserva `SAFE_STOP`: si no hay una foto
válida de visión de menos de 1500 ms, si el rover propio no aparece o si la
ronda terminó, los motores se detienen.
