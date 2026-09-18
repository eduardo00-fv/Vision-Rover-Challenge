# Diagnóstico físico CenfoBot — C++

Este sketch es para el CenfoBot/IdeaBoard de la competencia. No usa Wi-Fi,
cámara ni telemetría. Requiere `FastLED`.

Abra `diagnostico_rover.ino`, compile para el ESP32 y abra serial a 115200.
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
MOTOR 15 -15 300
STOP
```

La potencia está limitada a ±35 % y cada orden a 1500 ms. `STOP` y `DISARM`
apagan ambos motores.

## Calibración

Los ejemplos oficiales del CenfoBot no declaran encoders: no se mide PPR ni se
configura un PI de velocidad por pulsos. Para compensar motores, mida el desvío
en tres recorridos a 15, 20, 25, 30 y 35 % PWM; después use el promedio de
`IMU 30` en reposo como deriva y compare el giro relativo de cada recorrido.

Para ultrasonido mida una pared plana a 10, 20, 30, 50, 80 y 100 cm, diez
muestras por punto. El ejemplo oficial usa TRIG `IO25`, ECHO `IO26` y requiere
el jumper `SELECT`--`Vin`; confirme que ECHO sea seguro para 3.3 V.

Para color, registre suelo, rojo, verde y azul a igual distancia e iluminación.
Los ejemplos oficiales usan NeoPixel `IO4` y lectura `IO39`. El ejemplo de IR
también enumera `IO39` como frontal derecho: confirme físicamente esa posible
colisión antes de calibrar ambos como sensores independientes.
