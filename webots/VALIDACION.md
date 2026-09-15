# Plan de validación Webots — Vision Rover Challenge

Este documento define cuándo una corrida de Webots es una **prueba de sistema**
y no solamente una animación. Aplica a la integración completa:

```text
verdad física de Webots → cámara/base de visión → TCP/NDJSON v2 → rover 10 y 11 → actuadores Webots
```

El `Supervisor` puede conocer la verdad física para crear escenarios y evaluar
la corrida, pero no puede entregársela a la visión ni a los controladores de los
rovers. Los rovers se comunican con la base exclusivamente por el contrato TCP
v2. La lógica de misión y de coordinación se ejecuta por rover.

## Regla de oro

Cada ejecución debe guardar, junto con su resultado:

- identificador del escenario, versión de la cancha y semilla;
- configuración del controlador de cada rover;
- telemetría TCP recibida por ambos rovers;
- comandos de motor, transiciones de misión y eventos de seguridad;
- verdad física, solamente como oráculo de evaluación;
- resultado `PASS`, `FAIL` o `INCONCLUSIVE`, más la causa concreta.

Sin esos artefactos una corrida no cuenta como evidencia de regresión.

## Qué significa aprobar una ronda

Una ronda aprueba si, antes de `FINISHED`, se cumplen todas estas condiciones:

1. Cada cubo requerido queda completamente dentro de su depósito, según
   `depot_size` y `cube_side` del mensaje v2.
2. Ningún rover recibe una pose, cubo, depósito o asignación desde el
   `Supervisor`; solo usa sus sensores locales y TCP/NDJSON v2.
3. No hay contacto rover--rover ni salida del área permitida. Un contacto con
   cubo es válido únicamente durante una maniobra de aproximación o empuje.
4. Ante telemetría vencida, ausencia de pose propia, fase no ejecutable o una
   condición local de seguridad, el rover emite mando cero dentro del límite
   configurado y no vuelve a moverse hasta recuperarse.
5. La decisión de tarea no queda duplicada: dos rovers no pueden tener una
   reserva activa sobre el mismo cubo.
6. El informe se puede reproducir con la misma semilla y configuración.

Una entrega visual no basta: el oráculo calculará la inclusión geométrica
conservadora ya definida por el contrato. Un cubo que toca o sobresale de un
borde no es una entrega.

## Tipos de resultado

| Resultado | Uso |
| --- | --- |
| `PASS` | Cumple todos los criterios del escenario y no activa una falta crítica. |
| `FAIL` | Hay falta crítica, incumplimiento de una expectativa o finaliza sin completar la misión requerida. |
| `INCONCLUSIVE` | Falló la infraestructura de la prueba (por ejemplo, Webots terminó o faltan trazas); no prueba ni descarta el comportamiento. |

Son faltas críticas: fuga de verdad del Supervisor, colisión rover--rover,
salida de cancha, movimiento mientras aplica `SAFE_STOP`, mensaje que viola el
contrato o ausencia de artefactos de la corrida.

## Métricas que se registran

Los umbrales de comportamiento se fijarán después de la calibración física;
no se inventan para hacer pasar la simulación. Desde la primera corrida se
registran estas magnitudes:

| Métrica | Fuente | Propósito |
| --- | --- | --- |
| entregas válidas / requeridas | oráculo físico | resultado de misión |
| tiempo de misión y margen restante | reloj oficial | eficiencia sin reloj local |
| mínimo espaciamiento rover--rover y contactos | física Webots | seguridad |
| error posición/orientación visión vs verdad | visión + oráculo | calidad de la base |
| latencia captura → recepción y huecos de `seq` | TCP de cada rover | calidad del enlace |
| edad máxima de pose/cubo usada para decidir | telemetría + eventos | robustez ante oclusión |
| distancia de frenado y demora de parada | mando + física | seguridad |
| reasignaciones, reservas y bloqueos | eventos de misión | coordinación |

## Catálogo mínimo de escenarios

Cada escenario declara su semilla, duración, configuración y expectativas. Se
agregan casos concretos al runner sin modificar el controlador bajo prueba.

| ID | Situación | Resultado esperado |
| --- | --- | --- |
| `nominal_3_cubos` | Tres cubos, visión y red normales. | Entregas requeridas sin contacto entre rovers. |
| `cruce_de_rutas` | Dos misiones intersectan en el centro. | Un rover cede o replantea; no hay colisión ni doble reserva. |
| `cubo_oculto_en_empuje` | El rover oculta el cubo durante la entrega. | Se respeta `age_ms`; no se empuja a ciegas más allá del límite de la estrategia. |
| `pose_propia_perdida` | Se pierden observaciones de un rover. | El rover afectado se detiene; el otro no lo embiste ni toma decisiones con su pose vencida. |
| `tcp_intermitente` | Corte y reconexión del cliente. | Se frena con datos vencidos y se recupera solo con una foto v2 válida y reciente. |
| `cubo_desplazado` | El cubo cambia de posición durante aproximación/empuje. | Se vuelve a observar y se replantea la maniobra. |
| `limite_deposito` | Cubo en el borde y cubo completamente dentro. | Solo el segundo cuenta como entrega. |
| `fin_de_ronda` | La fase cambia a `FINISHED` durante movimiento. | Mando cero y ninguna acción posterior. |

## Orden de implementación

1. **Contrato y trazabilidad.** Corregir las discrepancias del publicador
   Webots con v2 (`ts_ms`, reloj `IDLE`, transiciones y parámetros de
   configuración), y crear el formato de eventos y reporte de corrida.
2. **Escenarios reproducibles.** Hacer que mundo, controlador y fallos se
   construyan desde una configuración única, sin coordenadas ni constantes
   duplicadas entre `.wbt` y Python.
3. **Base de visión en el lazo.** Sustituir la pose publicada por el Supervisor
   por imágenes de una cámara cenital procesadas por el pipeline de
   `vision-system`; conservar verdad física solo para medir el error.
4. **Control de rover real en el lazo.** Extraer el núcleo portable de la
   navegación/misión del firmware y conectarlo por TCP como haría el ESP32.
5. **Física calibrada.** Modelar chasis, ruedas, paletas, cubos, fricción y
   límites; contrastar velocidad, giro, frenado y empuje con medidas de cancha.
6. **Coordinación y fallos.** Incorporar asignación, reservas, prioridades y
   las inyecciones del catálogo.
7. **Regresión automática.** Ejecutar los escenarios en modo headless y
   publicar reporte, trazas y, cuando falle, evidencia visual.

Las fases 1 y 2 se pueden ejecutar de inmediato. Las fases 3 a 5 no se
consideran terminadas por tener una animación: requieren que la ruta de datos
y la lógica sean las mismas que se usarán en la cancha.

## Restricciones de diseño

- El contrato oficial permanece en `vision-system/contrato/CONTRATO.md`; no se
  crea un protocolo alterno exclusivo para Webots.
- La semilla controla ruido, pérdidas, fallos de red y posición inicial. Una
  semilla fija es obligatoria en regresión; las aleatorias se usan para campaña.
- Los parámetros físicos se documentan con su medición de origen y fecha.
- Las pruebas de visión incluyen tanto exactitud contra la verdad como el caso
  de oclusión y latencia; no basta con detectar imágenes estáticas.
