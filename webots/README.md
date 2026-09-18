# Simulación Webots — Vision Rover Challenge

Proyecto de simulación de la competencia. Reproduce la cancha efectiva de
860 × 860 mm, dos rovers (IDs 10 y 11), cubos de 60 mm y zonas de acopio de
200 × 150 mm. El supervisor publica el contrato TCP v2 en el puerto 2026.

Abra `worlds/vision_rover_challenge.wbt` con Webots. El servidor queda en
`0.0.0.0:2026`; el ESP32 puede conectarse usando la IP de esa computadora.

La tecla `R` reinicia el mundo y solicita `READY` al árbitro de visión; solo se
acepta cuando la cámara ya ve la cancha. `F` solicita detener una ronda en
curso e `I` aborta preparación. `S` no inicia una ronda: `READY → RUNNING` lo
decide exclusivamente el reloj del árbitro.

## Evolución a prueba de sistema

El mundo actual es un prototipo de integración: publica telemetría y ejecuta
dos agentes de referencia. No debe confundirse todavía con una validación
física de la competencia. El contrato de aceptación, las métricas y el orden
de trabajo para convertirlo en una prueba legítima están en
[VALIDACION.md](VALIDACION.md).

## Escenarios

El escenario nominal está en `scenarios/nominal_3_cubos.json` y referencia la
configuración contractual, que es la única fuente para posiciones, tamaños,
tiempos, semilla y patologías. Al presionar `R`, el supervisor vuelve a colocar
rovers y cubos según ese escenario, además de reiniciar la fase `READY`.

Para cargar otro escenario sin tocar el controlador, inicie Webots con
`VRC_SCENARIO` apuntando a su JSON. Una ruta relativa se resuelve desde la raíz
del repositorio; por ejemplo:

```bash
VRC_SCENARIO=webots/scenarios/nominal_3_cubos.json webots worlds/vision_rover_challenge.wbt
```

## Trazas de corrida

Cada ejecución escribe un NDJSON en `/tmp/vision-rover-webots` por defecto. El
archivo incluye metadatos, telemetría publicada, comandos y verdad física como
oráculo; esta última no se entrega a los rovers. Cambie el destino con
`VRC_RUN_DIR=/ruta/de/salidas`. Al terminar la ronda se agrega un resumen con
las entregas evaluadas mediante la geometría oficial. Mientras no exista el
oráculo de contactos y límites, el resultado se declara `INCONCLUSIVE`, nunca
un `PASS` engañoso.

La clasificación compara únicamente corridas completas y sin faltas: gana el
menor `completion_ms` del reloj oficial. Una entrega requerida ausente es
`FAIL`, igual que una colisión o salida de cancha; no recibe un tiempo válido.

El oráculo actual usa el rectángulo orientado de cada chasis, con dimensiones y
distancia preventiva declaradas por escenario. Contacto rover--rover o salida
de cancha son `FAIL` automáticos y quedan registrados en la traza. Una corrida
sin esos incidentes sigue siendo `INCONCLUSIVE` hasta que visión y física estén
en el lazo.

## Cámara y marcadores

El mundo incluye una cámara cenital `overhead_camera` y texturas ArUco
`DICT_4X4_50` para las cuatro esquinas y los dos rovers. Las imágenes se
derivan de los marcadores oficiales del repositorio y se verifican mediante
OpenCV antes de usarse. El supervisor convierte su buffer BGRA a BGR y lo pasa
por `vision.sistema.procesar`; la telemetría TCP ya proviene de ese pipeline, no
de las poses internas. Si la detección falla, conserva la última foto visual
buena o publica una foto vacía: nunca rellena datos con la verdad del mundo.

## Núcleo del rover

El agente Webots conserva el cliente TCP en Python, pero la navegación ya se
ejecuta en el núcleo C++ compartido con el firmware. Antes de abrir el mundo,
compile la biblioteca local:

```bash
make -C controllers/rover_agent
```

Si no existe `librover_logic.so`, el controlador no arranca: es intencional,
para no volver silenciosamente a una estrategia Python distinta de la del rover.

## Física del modelo

El mundo usa el mismo convenio de física que `AttaBot-Sim`: Webots `ENU`
(cancha en X--Y y altura Z), tracción diferencial con dos ruedas motrices de
alta fricción y dos castors de baja fricción. El `Supervisor` no traslada
rovers ni cubos durante una ronda; las ruedas, los contactos y la gravedad son
la única fuente de movimiento. Las dimensiones del chasis y las paletas siguen
siendo las del Vision Rover Challenge.

La prueba de banco opcional `VRC_ACTUATOR_TEST=izquierda,derecha` aplica un
mando fijo normalizado para medir avance, giro y frenado en una traza. Es un
modo de calibración aislado: no habilita resultados de misión ni cambia el
control normal.
