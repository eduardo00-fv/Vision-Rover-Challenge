# Auditoría de cumplimiento contra el reglamento vigente — 28/09/2026

Rama `webots-simulation` tras fusionar `origin/main` (`d5779dd`, 21/09) en
`a41ffaa`. Complementa [AUDITORIA_2026-09-23.md](AUDITORIA_2026-09-23.md): esa
auditoría evaluó la capacidad de competir; esta revisa **qué cambió en el repo
oficial y si cumplimos el reglamento nuevo**. Revisión estática y pruebas de
host; no hubo prueba física.

## 1. Qué cambió en el repo oficial

La rama partía del 12/09. Desde entonces la organización publicó:

| Archivo | Cambio | Impacto para nosotros |
| --- | --- | --- |
| `reglamento.md` | Reescrito: 27 secciones, tabla de prohibiciones. Arranque **automático** al detectar `READY`; ya no hay botón ni señal del juez. | Ver §3, H2. |
| `torneo.md` (nuevo) | 2 rondas, 2 intentos por ronda, cuenta el mejor intento; top 10 pasa; clasificación por cubos y luego tiempo del último cubo. | Estrategia: asegurar cubos antes que velocidad. |
| `docs/` (nuevo) | [Generador de posición de cubos](https://universidad-cenfotec.github.io/Vision-Rover-Challenge/) con dificultad `D∈[0,1]`. **Salida y acopios fijos; los cubos cambian en cada intento.** | Ver H7. |
| `solucion_simple.md` (nuevo) | Estrategia *greedy* de ejemplo (costo rover→cubo→acopio, prioridad de paso). | Nuestra asignación ya es más completa. |
| `conexiones/sensorcolor.png` (21/09) | Se corrigió el texto: **NeoPixel en IO4, sensor en IO32** (antes decía lo contrario). Las flechas del dibujo no cambiaron. | Ver H5. |
| `archivos_fabricacion/WebCam Base.stl` | Soporte de la cámara oficial. | Ninguno. |

El contrato de telemetría (`vision-system/contrato`) **no cambió**.

Geometría del generador: tablero 50×50 celdas, área efectiva 40×40, cubo de
3×3 celdas, **separación mínima de 2 celdas (40 mm) entre cubos**, acopio rojo
arriba, verde a la izquierda, azul a la derecha y los dos rovers abajo, lado a
lado. Es la misma disposición de `config_vision.json` rotada 90°: los colores
no se contradicen.

## 2. Evidencia ejecutada hoy

| Comprobación | Resultado |
| --- | --- |
| `tests/run_arena.sh` y `tests/run_firmware_safety.sh` | OK (incluye red solo telemetría, ESP-NOW, botón y guarda de borde) |
| `pytest webots/tests rover_control/tests` | 30 passed |
| Fusión de `origin/main` | Sin conflictos |

## 3. Hallazgos nuevos

| # | Prioridad | Hallazgo | Evidencia | Acción |
| --- | --- | --- | --- | --- |
| H1 | **P0 · descalificación** · ✅ corregido | El `config.h` local tiene `VRC_ENABLE_MOTOR_BENCH 1`. Con eso el rover **no ejecuta la misión** y además acepta `MOTOR` por Wi-Fi desde la laptop: control remoto, prohibido por §6.3.7, §11.2.7 y causal de descalificación §26.2.6. | `rover_control/config.h:15`; `rover_control.ino:244,272` | **Hecho:** `VRC_COMPETITION 1` en `config.h` hace fallar la compilación si queda banco, solo-red o botón activos, o sin perfil físico (`hardware_config.h`, al final). Al arrancar imprime `[config] competencia=… banco=… boton=…` para mostrárselo al juez. Prueba en `tests/run_firmware_safety.sh`. Falta: poner `VRC_COMPETITION 1` y `VRC_ENABLE_MOTOR_BENCH 0` en el `config.h` de cada rover antes de la ronda. |
| H2 | P1 · confirmar por escrito | **Nombres de fase distintos.** El reglamento dice `IDLE` 1 min → `READY` = inicio. El contrato y la visión oficial usan `READY` = 60 s de preparación → `RUNNING` = ronda de 600 s. Arrancamos en `RUNNING`, que es lo correcto con el software oficial actual: arrancar en `READY` nos movería durante la preparación. Riesgo: si la organización renombra las fases para ajustarlas al reglamento, el rover nunca arrancaría. | `reglamento.md` §8–9; `CONTRATO.md` §5; `safety_supervisor.cpp:16`; `config_vision.json` `ronda` | Correo a la organización: "¿el inicio es `phase=RUNNING` del contrato v2?". `VRC_REQUIRE_START_BUTTON 0` cumple §9.6; dejarlo así. |
| H3 | P2 · aclarar | **Cableado de sensores distinto al manual.** El equipo conectó los IR en IO4/5/18/19, el sonar en 27/33 y el color en IO23/IO32; la guía de conexiones indica IR en IO36/39/34/35, sonar en 25/26 y NeoPixel en IO4. El firmware coincide con el cableado real, así que funcionalmente está bien. **El reglamento no menciona el cableado ni obliga a seguir la guía de conexiones.** Solo exige la "configuración física y electrónica original" (§1.2) y prohíbe "modificar" componentes y sensores (§1.3, §1.7, §18, §26.2.1). El kit lo arma el propio equipo (`armado/README.md`), así que no hay un cableado "original" definido. Hay un riesgo interpretativo bajo, no un incumplimiento. | `conexiones/README.md`; `codigos/code_4IR.py:17-20`; `code_ultrasonic.py:12`; `hardware_config.h:48-52` | Por decisión del equipo **no se cambian los pines**. Opcional: confirmarlo con la organización en el mismo correo de H2. |
| H4 | P2 | Ultrasónico: midió 0,9 cm con el objeto a 20 cm (P1-6 del 23/09). No se debe a los pines, porque el firmware sigue el cableado real. | `hardware_config.h:49-50` | Sigue desactivado. Revisar el jumper SELECT–Vin que exige el manual. |
| H5 | P2 | Sensor de color desactivado. El reglamento no exige usarlo, porque el contrato ya da el color del cubo. | `hardware_config.h:51-52` | Nada por ahora. |
| H6 | P1 · parcialmente cerrado | **Integridad del hardware (§1.3–1.5, §18, §26.2.2–3).** La IdeaBoard del rover 2 **fue reportada y cambiada por la organización** (confirmado por el equipo el 28/09). Las bitácoras registran además que la IMU del rover 2 se montó en la placa del rover 1 y que se desconectaron los IR. | `calibracion_rover2/SESION_2026-09-21.md:42-60` | Conservar el correo o la constancia del cambio. Antes de la inspección, confirmar que cada IMU está en su rover y que los IR están conectados. |
| H7 | P1 | **Escenarios no representativos.** Los cubos salen del generador oficial con dificultad hasta 1, con 3 cruces de rutas y cubos a 40 mm entre sí; el rover mide unos 100 mm de ancho. En Webots solo existe `nominal_3_cubos`. La campaña del simulador 2D no incluye escenarios del generador ni la salida doble lado a lado. | `webots/scenarios/`; `docs/index.html` (`robots` en x=19 y x=27) | Portar el generador (unas 30 líneas) al simulador 2D y correr D=0,2 / 0,5 / 0,8 / 1,0 con 10 semillas cada una. Medir cubos entregados, tiempo del último cubo y contactos con cubos vecinos. |
| H8 | P1 | **Rover 2 sigue sin perfil validado** (`initially_validated=false`), así que no se mueve. El reglamento exige dos rovers que se coordinen y repartan tareas (§12.2.5–7); competir con uno solo no descalifica, pero arriesga el puntaje y la evaluación. | `motion/rover_profile.h:20-21` | Es la fase 1 del plan del 23/09. No hay resultados de la sesión del 25/09 (`diagnostico/sesiones/2026-09-25/` está vacía). |
| H9 | P2 | La parada de emergencia `!` por serie es una orden humana desde una computadora. Es aceptable antes de `READY`, pero durante el intento no debe haber ningún cable conectado. | `rover_control.ino:281` | Incluirlo en la lista de verificación previa al intento. |

## 4. Matriz de cumplimiento del reglamento vigente

| Regla | Estado | Cómo se cumple / qué falta |
| --- | --- | --- |
| §1, §18 Plataforma sin modificar | ⚠️ | Placa del rover 2 cambiada por la organización. Pendiente: restaurar la IMU y los IR (H6). El cableado propio (H3) no contradice ninguna regla escrita. |
| §4.3 Lógica cargada en los rovers | ✅ | Asignación, A*, cesión de paso y control corren en el ESP32; la laptop solo ejecuta la visión oficial. |
| §6.3, §11.2.7 Sin control externo tras el inicio | ✅ con `VRC_COMPETITION 1` | H1: la compilación de competencia rechaza el banco. `VRC_NETWORK_ONLY` y el banco se deciden al compilar; no hay comandos de movimiento en el modo misión. |
| §7.2–7.6 Comunicación entre rovers | ✅ | ESP-NOW `peer_link` a 10 Hz: cubo reclamado, etapa y si puede moverse. Si se pierde el enlace, se sigue sin él. |
| §7.9–7.10 Estado más reciente | ✅ | Se descartan `seq` y `ts_ms` viejos, hay límites de edad de pose y cubo, y la lectura por turno es acotada (`telemetry_client.cpp:80-114`). |
| §9 Inicio automático sin botón | ✅ / ⚠️ nombre | Arranca en `RUNNING` sin intervención y con botón en 0. Falta confirmar H2. |
| §10 10 min y fin | ✅ | `FINISHED` detiene los motores (`safety_supervisor.cpp:14`). |
| §11.2.5 Sin reinicios | ⚠️ | Si falla la IMU, pasa a solo visión en lugar de FAULT. Otras fallas enclavadas (P0-7 del 23/09) siguen dejando al rover quieto. |
| §12.2.12 Cubo oculto | ✅ | `MAX_PUSH_CUBE_AGE_MS 3000` durante el empuje. |
| §13.3–13.5 No sacar cubos entregados ni tirarlos | ✅ en simulación | Guarda de superficie en el empuje; 0 salidas en la campaña del simulador. No está probado en físico. |
| §17.4–17.5 Sin filas ni columnas fijas | ✅ | Grilla, salida, acopios y tamaños vienen de la telemetría; el A* admite hasta 64×64. |
| §26.1.2 Rover fuera de la superficie | ⚠️ | Solo el margen por visión (`NAV_ARENA_MARGIN_MM 60`); la guarda IR está apagada hasta medir las máscaras en la arena. |

## 5. Orden sugerido

1. **Hoy:** enviar un solo correo con H2 (nombre de la fase de arranque), opcionalmente H3
   (cableado propio de sensores) y la fecha de competencia.
2. Antes de cada ronda: `VRC_COMPETITION 1` y banco en 0 en el `config.h` de
   cada rover, y revisar la línea `[config]` al arrancar.
3. H7: agregar escenarios del generador oficial al simulador antes de ajustar
   más la estrategia.
4. Seguir con las fases 1 a 4 de la auditoría del 23/09 (rover 2 móvil, un
   cubo de punta a punta, dos rovers, ronda completa).
