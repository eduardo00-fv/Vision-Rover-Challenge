# Investigación: `WorldState` para los rovers

> Diseño propuesto, no implementación. Se basa en el contrato oficial v2 y
> preserva la autonomía: la laptop publica observaciones, nunca decisiones.

## Decisión central

El `WorldState` es una **foto inmutable, validada y reciente** del mundo que el
rover puede usar para decidir. No es una copia mutable del buffer TCP ni un
planificador. Cada mensaje NDJSON válido crea una foto nueva; la FSM consulta
una foto completa.

```text
TCP no bloqueante → parser NDJSON → validación → WorldState nuevo
                                                   ↓
                       coordinación rover↔rover → FSM / planificador / control
```

Esto evita que la FSM espere red, mezcle campos de paquetes distintos o use una
posición vieja como si fuese una observación actual.

## Tres estados, tres propietarios

| Tipo | Fuente | Contenido | Regla |
| --- | --- | --- | --- |
| Observado | Visión | Paquete v2: cancha, rovers, cubos, fase y reloj. | Verdad compartida; no se altera. |
| Estimado local | ESP32 propio | Frescura, velocidad, objetivo, ruta, progreso y fallos. | Se invalida ante datos viejos. |
| Coordinado | Rover 10 ↔ Rover 11 | Reserva, intención, latido y época. | Coordina; nunca reemplaza percepción. |

La visión ya resuelve la identidad: rover por `id` ArUco y cubo por `color`.
No hay que implementar asociación de detecciones.

## Modelo propuesto

```text
WorldState
├── meta: protocol_version, seq, captured_at_ms, received_at_ms,
│          transport_age_ms, sequence_gap, phase y clock
├── arena: cols, rows, cell_mm, start, depots[color], depot_size, cube_side
├── observed: rovers[id], cubes[color], obstacles[]
├── derived: frescura, velocidad, ventana de depósito y riesgo de colisión
└── mission_local: self_id, estado FSM, tarea, objetivo, ruta, seguridad,
                   reserva e intención del par
```

`mission_local` no viaja en el contrato de visión. La parte que el otro rover
necesita conocer viaja por el protocolo rover↔rover.

## Ingesta: último valor gana

1. Un bucle de red recibe fragmentos sin bloquear la FSM.
2. Acumula bytes y extrae solo líneas completas terminadas en `\n`.
3. Decodifica JSON y valida versión `v == 2`, campos y tipos.
4. Si llegaron varias líneas, conserva solo la última válida; un salto de `seq`
   se registra como métrica, no se recupera.
5. Construye e intercambia `latest_world` de forma atómica.
6. Si cae la conexión, la última foto envejece; el reconector funciona aparte.

TCP puede partir una línea o entregar varias juntas. No se debe asumir que un
`recv()` equivale a un mensaje ni acumular telemetría para navegar después.

## Validez y frescura

| Medida | Fórmula | Significado |
| --- | --- | --- |
| `object_age_ms` | Campo `age_ms` | Desde cuándo visión no ve el objeto. |
| `transport_age_ms` | `received_at_ms - ts_ms` | Antigüedad de toda la foto al llegar. |

Política inicial, a calibrar con hardware:

| Banda | Foto completa | Objeto | Acción |
| --- | --- | --- | --- |
| Fresca | `<200 ms` | `<200 ms` | Navegar y empujar. |
| Precaución | `200–500 ms` | `200–800 ms` | Reducir velocidad. |
| Degradada | `>500 ms` | `800–1500 ms` | No iniciar empuje fino ni cruces. |
| Insegura | `>1500 ms` | `>1500 ms` | Parada segura y recuperación. |

Un cubo viejo no está entregado, ni desapareció, ni quedó libre: es su última
posición conocida con incertidumbre creciente.

## Derivaciones necesarias

### Pose y velocidad

Conservar solo la foto anterior y estimar `v = Δposición / Δt` y
`ω = Δtheta / Δt`. La diferencia angular se normaliza a `[-180°, 180°]`: pasar
de 359° a 1° son 2°, no −358°. Con pose vieja o `Δt` anómalo, la velocidad es
desconocida; no se predice.

### Geometría segura del depósito

El punto `depot.col,row` es el centro de una zona, no el destino seguro del
cubo. Se deduce el borde más cercano, la orientación y se encoge el rectángulo
por este margen en cada lado:

```text
margin_cells = cube_side × √2 / 2
```

La ventana resultante garantiza que el cubo entero entra para cualquier giro. La
FSM debe escoger un `release_target` con margen adicional —preferiblemente el
centro de la ventana— y no empujar hasta el borde.

### Aproximación al cubo

Para cubo `C` y destino `D`:

```text
u = normalizar(D − C)
approach = C − u × (radio_cubo + radio_rover + margen)
push_goal = centro seguro de la ventana del depósito
```

Primero se alcanza `approach`; luego se alinea el rumbo `C → D` y se habilita
el empuje. Los márgenes reales se calibran con la huella del CenfoBot y el
contacto físico.

### Riesgo / ocupación

El mapa de coste contiene bordes con margen, el otro rover aumentado por su
huella/velocidad, cubos no objetivo, obstáculos futuros del contrato y la zona
de maniobra del cubo objetivo. Hoy `obstacles[]` viene vacío, pero se conserva
la interfaz para futuras ediciones.

## Interfaz de la FSM

La FSM nunca lee JSON; consulta operaciones semánticas:

```text
can_start_round()       self_pose_fresh()       peer_pose_fresh()
cube(color)             depot(color)            task_is_safe(task)
approach_point(task)    delivery_goal(task)     collision_risk(route)
object_is_in_depot(color)
```

Estados iniciales:

```text
WAIT_VISION → IDLE/READY → SELECT_TASK → RESERVE_TASK
→ PLAN_APPROACH → DRIVE_APPROACH → ALIGN_PUSH → PUSH
→ VERIFY_DELIVERY → RELEASE_TASK → SELECT_TASK
                 ↘ RECOVER / YIELD / SAFE_STOP
```

`SAFE_STOP` puede entrar desde cualquier estado por pose propia vieja,
telemetría degradada o guarda local de seguridad.

## Coordinación mínima entre rovers

La visión indica dónde está el par, no qué intenta hacer. Un mensaje pequeño
puede comunicar tarea, reserva, intención y vida:

```json
{"v":1,"from":10,"epoch":42,"state":"DRIVE_APPROACH","task":"green",
 "claim_until_ms":1785012410000,
 "intent":{"target_col":18.2,"target_row":7.0},"heartbeat":918}
```

- Un cubo tiene un único `claim` activo.
- Si reclaman ambos, gana la mayor `epoch`; a igualdad, el ID menor.
- La reserva vence tras varios latidos perdidos o al llegar `claim_until_ms`.
- El perdedor toma otro cubo o espera brevemente; no reclama en bucle.
- La reserva nunca reemplaza las poses oficiales de visión.

Para dos rovers, esto es más depurable que subastas o consenso completo y puede
reemplazarse después sin cambiar el modelo de mundo.

## Invariantes y métricas

- `seq` no retrocede; versión desconocida o mensaje inválido no cambia estado.
- La FSM ve fotos completas, no campos mutables individuales.
- Rovers/cubos se buscan por identidad, nunca por índice de lista.
- Pose vieja no habilita avance rápido ni empuje fino.
- `FINISHED` cancela tareas, detiene motores y libera reservas.
- Un cubo solo se considera entregado con geometría válida y datos frescos; la
  visión/árbitro sigue siendo la fuente final de verdad.
- Medir: paquetes válidos/inválidos, saltos de `seq`, reconexiones, latencia,
  máximos de `age_ms`, paradas, reservas y conflictos.

## Pendientes de prueba física

1. Huella/radio del rover y márgenes de seguridad.
2. Umbrales de frescura del hotspot y la cámara reales.
3. Distancia de aproximación y velocidad máxima de empuje.
4. Transporte rover↔rover: ESP-NOW frente a Wi‑Fi/IP.
5. Señal local que indique progreso de empuje antes de la confirmación oficial.

## Conclusión

El primer código debe ser un adaptador de telemetría no bloqueante y este
`WorldState` inmutable con validación y frescura. La FSM y el planificador se
construyen después sobre esas garantías.
