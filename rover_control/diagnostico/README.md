# Diagnóstico físico CenfoBot — C++

Este sketch es para el CenfoBot/IdeaBoard de la competencia. No usa Wi-Fi,
cámara ni telemetría. Requiere `FastLED`.

Abra `diagnostico.ino`, compile con ESP32 de Espressif 3.3.11 y abra serial a
115200 con terminación de línea. La carpeta y el sketch ahora tienen el mismo nombre.
Empiece por:

```text
HELP
STATUS
I2C
LINE 10
SONAR 10
COLOR 10
IMU 30
```

Para movimiento, un rover a la vez y con espacio libre:

```text
ARM
MOTOR 15 15 300
ARM
MOTOR 15 -15 300
STOP
```

La potencia de `MOTOR` está limitada a ±35 % y cada orden a 3500 ms. Use las duraciones
cortas de los ejemplos para las primeras pruebas; reserve 3500 ms para medir
recorridos en piso despejado, con suficiente holgura del cable USB. Cada movimiento
consume su autorización `ARM`. `STOP` y `DISARM` apagan ambos motores; `!`
también lo hace sin esperar Enter. Durante movimiento, cualquier comando
completo cancela la prueba y debe reenviarse con el rover detenido. Así las
mediciones de sensores, que son bloqueantes, nunca prolongan el movimiento.
Una línea serial incompleta no bloquea el temporizador de corte.

## Consola persistente

No hace falta abrir y cerrar el puerto para cada prueba. Con Python y
`pyserial` instalados, desde esta carpeta:

```bash
python3 serial_console.py --port /dev/ttyUSB0 --rover 1
```

La consola mantiene una única conexión a 115200, muestra el serial en tiempo
real y guarda cada transmisión/recepción en `/tmp/cenfobot-rover1.ndjson`.
Podés escribir `GZERO`, `GSIGN 1`, `ARM`, `TURN 45`, `TURN -45`, `STATUS`, etc.
directamente. `!` se manda sin Enter. Al cerrar con Ctrl-C o Ctrl-D envía `!`.
Si el puerto cambia, usá `--port`; para otro archivo, `--log /ruta/sesion.ndjson`.
Para el segundo rover, `--rover 2` separa el registro. La etiqueta la declara
el operador: no identifica automáticamente qué placa está enchufada ni aplica
ganancias. El programa de la placa sigue siendo C++; Python solo transporta
comandos y guarda el serial en la computadora.

Abrila en **tu propia terminal** para escribir los comandos vos mismo. El
asistente puede leer el NDJSON sin abrir el USB. No abras también el monitor
Arduino: el puerto debe tener un solo dueño. La consola envía paro al abrir
y al cerrar, no calibra ni arma automáticamente, no reintenta movimientos y
no se reconecta sola tras una desconexión. `!` se procesa por tecla, sin Enter.
Una consola abierta dentro de las herramientas del asistente no es una
terminal interactiva accesible por el operador; transferir el uso requiere
cerrar esa conexión primero.

La secuencia de validación y preparación del rover 2 está en
[PRUEBAS_ROVERS.md](PRUEBAS_ROVERS.md).

Cableado confirmado por el usuario:

| Señal | GPIO |
| --- | --- |
| M1 / M2 (driver IdeaBoard) | 12/14 y 13/15 |
| IR S1, S2, S3, S4 | 4, 5, 18, 19 |
| Ultrasonido TRIG / ECHO | 27 / 33 |
| Color DI / A0 | 23 / 32 |

`LINE` informa cuántas muestras fueron HIGH por sensor. Se usa lectura
digital: GPIO19 no dispone de ADC. Compare claro y oscuro para verificar la
polaridad y si la salida de los módulos es apropiada para lectura digital.
No se asigna una posición física a S1–S4 sin comprobarla.

## Calibración

Los ejemplos oficiales del CenfoBot no declaran encoders: no se mide PPR ni se
configura un PI de velocidad por pulsos. Los mandos `MOTOR` permiten comparar
arranque y desvío, pero una compensación PWM fija no mantiene el rumbo.
Para medir giro durante el movimiento, use el modo `STRAIGHT` descrito abajo;
`IMU 30` solo informa velocidad angular durante una medición en reposo, no
reconstruye el giro de un recorrido anterior.

Para ultrasonido mida una pared plana a 10, 20, 30, 50, 80 y 100 cm, diez
muestras por punto. Use el cableado confirmado arriba; confirme que ECHO sea
seguro para 3.3 V.

Para color, registre suelo, rojo, verde y azul a igual distancia e iluminación.
El iluminador usa DI en GPIO23 y la entrada analógica A0 en GPIO32. Los valores
crudos se muestran en escala ADC Arduino; la clasificación por canal mínimo
es provisional hasta medir muestras reales.

Primero valide los comandos sin alimentación de motores. Después eleve las
ruedas y pruebe `ARM`, `MOTOR 15 0 300`, espere el corte y repita con M2.
Durante otra prueba, envíe `COLOR 50`: debe cancelar el motor sin iniciar la
medición. Reenvíe `COLOR 50` ya detenido. Pruebe también `!` y una línea sin
Enter. Registre sentido, duración observada y respuesta de parada antes de
pasar al piso. La compilación no sustituye estas comprobaciones físicas.

## Avance con corrección de rumbo — experimental

Solo C++, sin cámara ni encoders. Control **PD de rumbo relativo**: integra
gyro Z después de restar su sesgo y aplica una corrección diferencial a las
bases PWM. No regula velocidad/distancia, no recupera posición lateral y no
elimina la deriva en recorridos largos. Sus ganancias todavía necesitan
validación física; no es un piloto autónomo aprobado.

Preparación sin movimiento de motores (enviar cada comando al terminar el anterior):

1. `GZERO`: mantener el rover inmóvil. Descarta el arranque del sensor y estima
   sesgo con 200 muestras nuevas. Debe responder `GZERO OK`; caduca en 60 s.
   Rechaza sesgo mayor de 5 °/s o variación mayor de 1 °/s. Esto detecta ruido
   y algunos movimientos, pero no demuestra inmovilidad: no tocar el rover.
2. `GYRO 5000`: motores apagados. Girar a mano unos 90° hacia la **izquierda**,
   visto desde arriba, y comprobar signo y magnitud de `yaw_raw`. Si Z apenas
   cambia o el ángulo no corresponde, revisar montaje/eje antes de continuar.
3. `GSIGN 1` si ese giro produjo ángulo positivo; `GSIGN -1` si fue negativo.
   Confirmarlo también girando hacia la derecha. El firmware no adivina el
   montaje; al reiniciar vuelve a signo desconocido y bloquea `STRAIGHT`.
4. Recolocar, hacer otro `GZERO` quieto y esperar `OK`.

Primera prueba autorizada, despejada y corta:

```text
ARM
STRAIGHT 18 20 1000
```

Las bases 18/20 son provisionales para el rover probado el 19/09, no una
configuración universal. `STRAIGHT` acepta bases de 10 a 25 %, únicamente hacia
adelante, y duración de 1 a 3500 ms. No enviar comandos durante la prueba salvo
para detenerla. Después se puede ampliar el tiempo con el operador listo.

`HCONFIG 0.6 0.08 4` fija los valores iniciales de Kp, Kd y límite de corrección
en puntos porcentuales PWM. Rangos: Kp 0..2, Kd 0..0.5, límite 0..5. No hay
integral ni ganancias guardadas en flash. `STATUS` muestra la configuración.
Las líneas `H` informan ángulo, tasa, corrección y mandos L/R durante la prueba;
se descartan si el serial no tiene espacio, para no bloquear el control.

Protecciones: `ARM` de un solo uso, `!` sin Enter, tiempo máximo, corte por
error I2C, por intervalo entre muestras superior a 100 ms, por giro acumulado
mayor de 20° o tasa mayor de 150 °/s. El corte es cooperativo, no un watchdog
independiente; cada operación I2C tiene timeout de 5 ms. Cualquier comando
completo durante calibración, observación o movimiento cancela la tarea y
debe reenviarse. `IMU` cambia la configuración del sensor e invalida GZERO.

La configuración de registros (104 Hz, BDU, auto-incremento, datos nuevos y
sensibilidad de gyro) se basa en la [ficha del LSM6DS3TR-C de ST](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf).

Pruebas sin placa, desde la raíz:

```bash
g++ -std=c++17 rover_control/tests/test_heading_control.cpp -o /tmp/vrc-heading-test
/tmp/vrc-heading-test
g++ -std=c++17 -I rover_control/tests/diagnostic_host rover_control/tests/test_diagnostic_heading.cpp -o /tmp/vrc-diagnostic-heading-test
/tmp/vrc-diagnostic-heading-test
```

Cubren signo de corrección, saturación, sesgo, rollover, falta de muestras,
error I2C, calibración/signo ausentes, calibración vencida, argumentos inválidos,
`!`, cancelación por comando y corte aun con una línea serial incompleta.
Los buses y el reloj son simulados: falta comprobar todo en hardware.

## Giro por tiempo con medición IMU

`TURNTEST -25 25 1000` aplica M1=-25 % y M2=+25 % durante 1000 ms, registra
el giro relativo y apaga PWM. Continúa midiendo 500 ms **sin motores** para
incluir parte del giro residual. Esperar `TURNTEST RESULT` antes de enviar
otro comando: informa `yaw` al final de esa ventana, `yaw_cut` (última muestra
antes de apagar PWM) y su diferencia `coast`. No garantiza que el robot haya
terminado de asentarse al acabar la ventana; hay que observarlo.

Requiere `GZERO` reciente, `GSIGN` comprobado y `ARM` de una sola orden.
Rangos: ruedas con signos opuestos, hasta ±35 %, 1..3500 ms. El signo del
resultado es positivo hacia la izquierda con la configuración ya verificada.
No usa las ganancias de `STRAIGHT` ni corrige hacia un ángulo objetivo.
Conserva `!`, cancelación por comando, corte por error I2C o falta de muestras
por más de 100 ms. Aborta y descarta el resultado si supera 200 °/s o 180°.
Estos umbrales son para la prueba de giro, no sustituyen el límite de 20°
del control de avance. Una cancelación no publica un resultado completo.

Secuencia (esperar a que termine cada operación):

```text
GZERO
GSIGN 1
ARM
TURNTEST -25 25 1000
```

`MOTOR` continúa siendo una prueba sin registro de ángulo. Ninguno de estos
comandos puede reconstruir el giro de una prueba pasada sin muestras.

## Giro a un ángulo relativo — experimental

`TURN 45` gira a la izquierda; `TURN -45`, a la derecha. Admite magnitudes de
10 a 180°, con `GZERO` reciente, `GSIGN` verificado y `ARM`. El rumbo inicial
de cada orden se toma como cero. No es orientación absoluta ni depende de PPR.

Arranca al 25 % y reduce progresivamente hacia 22 % en los últimos 15°.
Anticipa el corte usando la velocidad angular actual, con coeficientes
provisionales de 0,04 s a izquierda y 0,05 s a derecha, más 1° de margen;
la predicción por velocidad se limita a 6°. Son hipótesis basadas en dos
pruebas, no una calibración definitiva. No aumenta por encima de 25 % si
se atasca ni invierte motores para corregir después del corte.

Se detiene si no progresa al menos 1° durante más de 750 ms, gira más de 5°
en sentido contrario o agota el tiempo: 3500 ms para magnitudes de hasta 90°,
7000 ms para magnitudes mayores, hasta 180°. Aborta si el ángulo medido
supera la magnitud del objetivo en más de 15°. Conserva los cortes IMU y `!` de
`TURNTEST`. Tras apagar PWM observa 500 ms adicionales y reporta:

- `OK`: error final dentro de ±3° y tasa inferior a 3 °/s durante los últimos
  200 ms, según IMU. No prueba exactitud absoluta ni reemplaza observación física.
- `OUTSIDE_TOLERANCE`: quedó fuera del margen.
- `UNSETTLED`: aún no cumple el criterio de reposo.
- `TIMEOUT`: se agotó el tiempo; nunca se presenta como objetivo alcanzado.

Las fallas y cancelaciones no publican un resultado completo. Una nueva orden
siempre necesita otro `ARM`. Las ganancias de avance no intervienen en `TURN`.

Prueba unitaria del controlador:

```bash
g++ -std=c++17 rover_control/tests/test_turn_control.cpp -o /tmp/vrc-turn-test
/tmp/vrc-turn-test
```

La prueba de integración `test_diagnostic_heading.cpp` también cubre ambos
sentidos, reducción/corte, atasco, giro contrario, timeout, cancelación,
falla I2C, falta de muestras y clasificación del resultado después del corte.
