# Rover Control — ESP32 / Arduino C++

Base de control para cada CenfoBot. Esta primera entrega implementa:

- Conexión Wi‑Fi al hotspot de visión.
- Cliente TCP al puerto oficial `2026`, lectura acotada por vuelta y conexión
  con timeout de 50 ms (motores detenidos antes de reconectar).
- Reconstrucción de NDJSON: una línea JSON por mensaje.
- Validación de telemetría v2 y actualización atómica de `WorldState`.
- Control diferencial de los motores N20, sin encoders.
- Navegación visual por maniobras cortas con control local de rumbo por IMU.
- Lectura de cuatro sensores de línea; sonar/color deshabilitados por defecto.
- FSM mínima que entra en parada segura con visión vieja, pose propia vencida o
  ronda finalizada.

La calibración inicial de movimiento del rover 1 está cerrada. El rover 2 tiene
un [sketch guiado](calibracion_rover2/README.md) preparado para probar después.
El firmware autónomo y el diagnóstico comparten lector LSM6DS3TR-C, PD y
perfil de giros. `motion/arena_motion.h` integra estos módulos con la visión y
las paradas. El perfil del rover 2 permanece bloqueado por falta de calibración.
Prioridades y pendientes: [plan actual](../PLAN_ACTUAL.md).

## Primera integración para arena

Seleccionar `VRC_PHYSICAL_ROVER 1` en `config.h`, además del marcador real en
`ROVER_ID`: son identificaciones distintas. Un archivo de configuración antiguo
sin selección física queda bloqueado. Mantener el robot quieto al encender:
el cero IMU recoge 200 muestras tras 250 ms de estabilización, con motores
detenidos. Si falla, corregir la causa y reiniciar; no se recalibra en movimiento.

La visión elige objetivo y error angular. Para errores mayores de 10° se usa
un giro relativo con el perfil probado; para el resto, avance de hasta 350 ms
con bases 18/20 y PD 0,8/0,08, limitado a 4 puntos PWM. Cada maniobra termina
con 500 ms de reposo y espera de una captura nueva. Es una primera navegación
con pausas, pendiente de validación física; no estima distancia con la IMU ni
fusiona orientaciones absolutas. Los parámetros visuales de velocidad no
determinan el PWM de este controlador físico. El empuje usa también 18/20:
su capacidad con carga todavía debe probarse.

El permiso de visión se comprueba en cada vuelta, también durante los giros.
IMU sin muestras durante más de 100 ms, error de lectura, giro atascado o timeout
de giro enclavan una parada hasta reiniciar. `!` por serial también enclava
la parada. No hace falta enviar comandos serial para ejecutar la misión.
La protección IR de borde sigue deshabilitada hasta confirmar posición y
polaridad; las primeras pruebas deben ser supervisadas en el centro de la arena.
Sonar y color siguen pospuestos (`VRC_ENABLE_SONAR/COLOR 0`).

Pruebas sin placa, incluyendo `setup()`/`loop()` reales con periféricos simulados:

```sh
bash rover_control/tests/run_arena.sh /ruta/ArduinoJson/src
```

## Preparación

Para comprobar solamente la red, definir `VRC_NETWORK_ONLY 1` en `config.h`.
Recibe y valida telemetría sin inicializar la IMU ni permitir movimiento,
incluso si llega `RUNNING`. No se puede combinar con el banco de motores.
La laptop puede conservar internet por Wi-Fi y servir la visión por Ethernet:
`VISION_HOST` debe ser la IP Ethernet, y el rover debe conectarse al Wi-Fi
del router de esa misma LAN (sin aislamiento entre clientes y Ethernet).
La IP obtenida por DHCP debe comprobarse antes de cada sesión.
Para probar transporte sin cámara, ejecutar el publicador oficial con
`python3 vision-system/contrato/mock_publisher.py --host <IP-Ethernet>`.
Esta prueba comprueba recepción, no visión real ni navegación física.

1. Instalar **esp32 by Espressif Systems** en Arduino IDE.
2. Instalar las bibliotecas **ArduinoJson** y **FastLED** desde el administrador
   de bibliotecas.
3. Copiar `config.example.h` como `config.h`.
4. Configurar hotspot, IP, marcador real `ROVER_ID` y `VRC_PHYSICAL_ROVER`.
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
Webots. Ambos adaptadores consumen esa geometría, la asignación automática de
tareas y la cesión de paso por prioridad de ID a menos de 200 mm. Esto no es
un planificador de rutas ni garantiza evitar colisiones o bloqueos. Webots usa
los parámetros de navegación predeterminados; los ajustes físicos del firmware
todavía deben trasladarse y calibrarse en el simulador. La regla de depósito usa la media
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
| IR S1, S2, S3, S4 (lectura digital) | 4, 5, 18, 19 |
| Ultrasónico | TRIG 27, ECHO 33 |
| Color reflectivo | NeoPixel DI 23, analógico A0 32 |

El pin `ECHO` de un HC-SR04 típico es de 5 V: use un divisor de voltaje antes
de GPIO33, que es de 3.3 V. Antes de cargar cada rover, ajuste `ROVER_ID`
y el sentido de motores en `config.h`. La asignación de cubos es automática. `hardware_config.h`
incluye esa configuración en cada módulo C++, no solo en el sketch.
Los IR se leen digitalmente: GPIO19 no tiene ADC. La polaridad debe medirse;
la parada por línea sigue deshabilitada hasta validar su uso en la cuadrícula.

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
ARM
MOTOR 20 -20 300     # giro breve
STOP
DISARM
```

Pruebe un rover a la vez, con espacio libre y una persona lista para cortar
alimentación. Vuelva `VRC_ENABLE_MOTOR_BENCH` a `0` antes de cualquier ensayo
con visión.

## Regla de seguridad actual

Fuera del modo de banco, se corta movimiento ante desconexión, foto inválida,
foto recibida hace más de 1500 ms, pose propia vencida o fase distinta de RUNNING.
La edad efectiva de pose/cubo suma `age_ms` y el tiempo local desde recepción.
Las publicaciones que repiten una captura no renuevan ese instante.
No se presume sincronización entre el reloj UNIX de visión y `millis()`:
todavía no se mide el retraso absoluto de transporte de la primera captura.
El corte se evalúa cooperativamente; no constituye un watchdog independiente
del microcontrolador. Antes de reconectar TCP se apagan los motores.

Prueba de regresión en la laptop (usa ArduinoJson instalado, sin placa):

```bash
bash rover_control/tests/run_firmware_safety.sh /ruta/Arduino/libraries/ArduinoJson/src
```
