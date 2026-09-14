# Rover Control — ESP32 / Arduino C++

Base de control para cada CenfoBot. Esta primera entrega implementa:

- Conexión Wi‑Fi al hotspot de visión.
- Cliente TCP no bloqueante al puerto oficial `2026`.
- Reconstrucción de NDJSON: una línea JSON por mensaje.
- Validación de telemetría v2 y actualización atómica de `WorldState`.
- FSM mínima que entra en parada segura con visión vieja, ausencia de pose propia
  o ronda finalizada.

No controla motores todavía. Esa capa se agregará tras medir la electrónica y
calibrar ambos rovers.

## Preparación

1. Instalar **esp32 by Espressif Systems** en Arduino IDE.
2. Instalar la biblioteca **ArduinoJson** desde el administrador de bibliotecas.
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

Los próximos módulos serán `motor_controller`, `sensors` y `navigator`. Ningún
módulo podrá emitir PWM sin que `safety_supervisor` haya autorizado movimiento.

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

## Regla de seguridad actual

El programa no enviará movimiento porque aún no hay driver de motores. Cuando se
incorpore, deberá conservar `SAFE_STOP`: si no hay una foto válida de visión de
menos de 1500 ms, si el rover propio no aparece o si la ronda terminó, los
motores se detienen.
