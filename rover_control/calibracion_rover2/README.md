# Calibración guiada del rover 2

Abrir **calibracion_rover2.ino**, ESP32 Dev Module, core Espressif **3.3.11**,
FastLED instalada, monitor serial **115200**, terminación nueva línea.
Conservar la carpeta dentro del repositorio: comparte el código de
`../diagnostico/` para usar los mismos controles y cortes ya probados.
Firmware Arduino **C++**, sin cámara ni Wi-Fi. No necesita script de PC.

Al encender imprime el menú y permanece desarmado. Escribir **una orden a la
vez**, esperar el resultado y observar el rover. Cada comando de movimiento
de este menú autoriza una prueba; no hace falta escribir `ARM` por separado.
`!` para sin Enter si la terminal lo transmite por tecla; con el monitor
Arduino utilizar su mecanismo de envío. Cualquier comando completo durante
una prueba la cancela: no se encadenan movimientos automáticamente.

| Comando | Acción |
| --- | --- |
| `MENU` | Reimprimir instrucciones |
| `CHECK` | I2C, IMU y lecturas IR, sin mover |
| `IMUBUS` | Diagnóstico de registros I²C con motores apagados: compara lecturas cortas/largas, STOP y timeout 5/25 ms; invalida el cero y restaura timeout 5 ms |
| `M1` | Solo M1, +15 %, 3,5 s; ruedas levantadas |
| `M2` | Solo M2, +15 %, 3,5 s; ruedas levantadas |
| `REV` | Ambos −15 %, 3,5 s; ruedas levantadas |
| `MAPA OK` | Confirmar M1 izquierda, M2 derecha y positivo adelante |
| `CERO` | Calibrar IMU quieto; esperar `GZERO OK` |
| `OBSERVAR` | 10 s de gyro, motores apagados; girar a mano a izquierda ~90° |
| `SIGNO 1` / `SIGNO -1` | Configurar el signo observado para el giro izquierdo |
| `BASE 20 20` | Bases PWM independientes (10..25); inicialmente 20/20 |
| `GANANCIAS 0.8 0.08 4` | Kp, Kd y límite de corrección; punto de partida provisional |
| `RECTO 1000` / `RECTO 3500` | Avance con IMU, duración en ms |
| `GIRO 45` / `GIRO -45` | Izquierda/derecha por ángulo; luego ±90 y ±180 |
| `RESUMEN` | Mostrar ajustes actuales para copiarlos al registro |
| `STOP` / `!` | Detener y desarmar |

## Recorrido rápido

1. `CHECK`, luego `M1`, `M2`, `REV`, observando cada prueba con ruedas
   levantadas. Si corresponde, `MAPA OK`. Si no corresponde, corregir la
   identificación/sentido antes del control por IMU.
2. `CERO`, esperar OK; `OBSERVAR` y girar manualmente a izquierda. Configurar
   `SIGNO` según lectura; comprobar también sentido contrario.
3. En piso: `CERO`, esperar OK, `RECTO 1000`. Si arranca bien, probar
   `RECTO 3500`. Ajustar `BASE` y/o `GANANCIAS` solo según observación.
4. `GIRO 45`, `GIRO -45`; luego ±90 y ±180, esperando cada resultado.
   Repetir `CERO` cuando venza (60 s). Ante un error, revisar antes de seguir.
5. `RESUMEN` y conservar configuración/resultados propios del rover 2.

No se asume automáticamente el signo ni el mapa de ruedas del rover 1.
Los ajustes viven en RAM: vuelven a sus valores iniciales al reiniciar.
Los comandos originales del diagnóstico siguen disponibles con sus propias
condiciones (`ARM` para `MOTOR`, `TURN` y `STRAIGHT`). `MAPA OK` protege los
atajos guiados; no es una detección física ni bloquea los comandos originales.
El resultado `OK` en giros significa ±3° según IMU, no según referencia externa.

Los umbrales de PWM/giro son iniciales para comparar rovers, no una garantía
de que el segundo responda igual. No se autotunean ni se aprueba el rover solo
por haber emitido los comandos. `RESUMEN` exporta ajustes, no una certificación.

Opcionalmente, desde la raíz, usar la consola que registra los resultados:

```bash
python3 rover_control/diagnostico/serial_console.py --port /dev/ttyUSB0 --rover 2
```

El operador manda las órdenes en su terminal y el asistente puede leer
`/tmp/cenfobot-rover2.ndjson`. Cerrar el monitor Arduino antes de abrirla.
Para compilar sin cargar:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 rover_control/calibracion_rover2
```
