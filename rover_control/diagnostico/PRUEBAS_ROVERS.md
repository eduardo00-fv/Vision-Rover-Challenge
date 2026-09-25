# Pruebas independientes de los rovers

Para ejecutar el rover 2 con menos comandos, usar el sketch guiado
[`calibracion_rover2.ino`](../calibracion_rover2/calibracion_rover2.ino) y su
[menú de pruebas](../calibracion_rover2/README.md). La calibración inicial de
movimiento del rover 1 quedó cerrada por decisión del usuario; las pruebas
de integración pendientes están en [el plan actual](../../PLAN_ACTUAL.md).

Se carga la misma carpeta `diagnostico` (Arduino C++, ESP32 3.3.11) a cada
placa. No utiliza cámara. No copiar el sesgo del gyro de una placa a otra:
`GZERO` lo mide de nuevo. Los parámetros ajustados por comandos se pierden al
reiniciar. La etiqueta de consola rover 1/2 identifica el registro, no cambia
la configuración del firmware ni equivale a los IDs de competencia 10/11.

## Estado del rover 1

- M1 izquierda y M2 derecha; signo positivo hacia adelante, reversa comprobada.
- Paro `!`, corte por tiempo y desarme comprobados.
- Avance inicial: bases 18/20, `HCONFIG 0.8 0.08 4`; sesgo se recalibra quieto.
- IMU: giro izquierdo positivo, `GSIGN 1` verificado con giros manuales.
- Giros de ±45° y ±90° dentro de ±3° según IMU en las pruebas registradas.
- ±180°: consultar los resultados nuevos en `SESION_2026-09-19.md`.

Esto es un ajuste inicial de movimiento. Para cerrar la calibración de uso
real faltan comparación con una referencia externa, repetición en las
condiciones previstas de batería/piso/carga y prueba con empuje de cubos.
Faltan también mapa físico/polaridad de IR y la integración de sus paros.
Ultrasonido y color siguen pendientes, pospuestos por el operador.

## Carga y consola

En Arduino IDE: abrir `diagnostico.ino`, ESP32 Dev Module, core Espressif
3.3.11, seleccionar el puerto correcto y cargar. Requiere FastLED. Cerrar la
consola/monitor antes de cargar. Alternativa CLI desde la raíz del repositorio:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 rover_control/diagnostico
arduino-cli upload --port /dev/ttyUSB0 --fqbn esp32:esp32:esp32 rover_control/diagnostico
python3 rover_control/diagnostico/serial_console.py --port /dev/ttyUSB0 --rover 2
```

Para rover 1 usar `--rover 1`. Los registros son `/tmp/cenfobot-rover1.ndjson`
y `/tmp/cenfobot-rover2.ndjson`; contienen fecha, sesión, TX y RX. `/tmp` es
temporal: copiar los resultados importantes antes de reiniciar la computadora.
Solo una aplicación debe abrir el puerto. El asistente puede leer el archivo
mientras vos escribís en esta consola.

## Rover 2: arranque sin asumir la calibración del primero

Mandar una orden a la vez; esperar cada resultado. No pegar una lista entera:
un comando recibido mientras hay movimiento o medición cancela la operación.

1. `HELP`, `STATUS`, `I2C`. Verificar motores desarmados.
2. Con ruedas levantadas: `ARM`, luego `MOTOR 15 0 1000`; comprobar rueda y
   sentido. Repetir con `MOTOR 0 15 1000` y después reversa. Si no coincide
   M1 izquierda/M2 derecha/positivo adelante, no usar `STRAIGHT` o `TURN`
   hasta corregir la correspondencia: ese convenio está incorporado al control.
3. Comprobar `!` durante una prueba breve. Levantar las ruedas es también
   necesario al comprobar por primera vez los sentidos; ARM no los valida.
4. `GZERO` quieto, esperar `OK`. `GYRO 10000`: girar manualmente a izquierda
   ~90°, observar signo y magnitud; repetir derecha. Configurar `GSIGN 1` o
   `GSIGN -1` según el signo del giro izquierdo. Si Z no mide ese giro, revisar
   montaje antes de mover con corrección.
5. En piso, comprobar arranque de avance y giro. Los valores 18/20 y 25 % de
   giro son referencias del rover 1, no medidas del rover 2. El controlador
   actual TURN arranca al 25 % y reduce hacia 22 %; si no funciona en rover 2,
   registrar el resultado y ajustar antes de ampliar recorridos.
6. Comparar avance recto corregido a 1 s y 3,5 s. Medir distancia y desviación
   lateral; registrar voltaje de batería, piso y carga. Ajustar ganancias solo
   con resultados comparables; no se guardan automáticamente.
7. Giros ±45°, luego ±90°, después ±180°. Tres repeticiones por sentido y
   ángulo, observando además una referencia marcada en el piso. Si una falla,
   revisar el resultado antes de repetir; no hay reintentos automáticos.

## Comandos para repetir sin el asistente

En rover 1, antes de una prueba (cada línea tras terminar la anterior):

```text
GZERO
GSIGN 1
HCONFIG 0.8 0.08 4
ARM
TURN 180
```

Esperar `TURN RESULT`. Para otra prueba, hacer `GZERO` quieto, esperar `OK`,
mandar `ARM` y elegir **solo una** orden:

| Orden | Prueba |
| --- | --- |
| `TURN -180` | Media vuelta derecha |
| `TURN 90` / `TURN -90` | Cuarto de vuelta |
| `TURN 45` / `TURN -45` | Giro corto |
| `STRAIGHT 18 20 3500` | Avance corregido, bases del rover 1 |

La calibración vale 60 s. `TURN` termina por ángulo o protección; `STRAIGHT`
termina por tiempo, no por distancia. `OK` indica tolerancia de ±3° según IMU,
no exactitud respecto a la cancha. `TIMEOUT`, `UNSETTLED`,
`OUTSIDE_TOLERANCE` o `ERROR` requieren revisar antes de seguir.

Registrar por prueba: rover, comando, batería/piso/carga, ángulo IMU, ángulo
externo, error, desplazamiento lateral y observación de tirones/oscilaciones.
Para pasar a autonomía faltan además comunicación/visión, pérdida de señal,
límites de cancha y coordinación entre los dos rovers bajo carga real.
