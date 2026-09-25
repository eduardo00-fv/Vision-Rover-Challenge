# Plan operativo — 21/09/2026

Este estado actualiza la planificación, no reemplaza el reglamento ni la
auditoría histórica de `MAPA_MENTAL_Y_AUDITORIA.md`. Proyecto: CenfoBot,
ESP32 Arduino C++ 3.3.11. El diagnóstico manual es herramienta de laboratorio;
la decisión de competencia debe ejecutarse en los rovers.

## Cierres acordados

- **Rover 1: calibración inicial de movimiento cerrada por decisión del usuario.**
  Bases 18/20, Kp 0,8, Kd 0,08, límite 4, `GSIGN 1`; sesgo recalculado quieto.
  Pruebas de avance, paro y giros ±45°, ±90°, ±180° registradas en
  `rover_control/diagnostico/SESION_2026-09-19.md`. No volver a abrir esta
  etapa salvo un fallo concreto de integración. Ensayos con carga y sensores
  quedan en los siguientes bloques, no bloquean preparar el rover 2.
- **Rover 2: procedimiento guiado preparado para ejecutar después.**
  `rover_control/calibracion_rover2/calibracion_rover2.ino`, menú y comandos
  cortos. Falta cargarlo al segundo robot y registrar sus propios resultados.
- Firmware base ya recibe contrato v2, filtra estados inválidos/viejos y
  comparte navegación/asignación/cesión de paso con Webots. Hay pruebas de
  software; no equivalen a una ronda física o simulada completa.

## Próximo bloque de desarrollo: integrar el movimiento comprobado

El firmware de competencia ya integra el lector y los controladores comunes
mediante `motion/arena_motion.h`. La IMU tiene su propia actualización cooperativa;
`mission_controller.cpp` sigue traduciendo fases, no representa la misión completa.

1. **Completado (21/09):** lectura LSM6DS3TR-C, PD de rumbo y perfil de giro
   viven en `rover_control/motion/`. El diagnóstico usa esos mismos módulos;
   el perfil 1 conserva 18/20, Kp 0,8, Kd 0,08, límite 4 y GSIGN 1. El perfil
   2 es deliberadamente candidato y no habilita movimiento hasta confirmar su
   propio mapa/signo.
2. **Implementado (21/09), pendiente de prueba física:** integración en
   `rover_control.ino`, cero IMU cooperativo, avance 18/20 por segmentos de
   hasta 350 ms, giros relativos según error visual y reposo de 500 ms antes
   de volver a decidir con otra captura. Supervisor conserva prioridad;
   fallos IMU y `!` quedan enclavados. No es fusión de orientación absoluta.
3. **Pruebas host aprobadas (21/09):** controlador y `setup()`/`loop()` reales
   con periféricos simulados: FINISHED, desconexión, pose vencida, `!`, fallo
   IMU; además controlador con giros ±45°, atasco y rollover de micros.
   Falta identificar el marcador físico y verificar la cadena con cámara real.

Compilación ESP32 Arduino 3.3.11 aprobada el 21/09 (`esp32:esp32:esp32`,
configuración de ejemplo): 955003 bytes de flash (72%) y 51932 bytes de RAM
global (15%). No se cargó al robot. El firmware integrado decide sin comandos
serial; queda validar esta integración físicamente y con visión real.

### Prueba de red física — 21/09

- Rover 1 identificado por MAC `70:4b:ca:5e:bc:5c`, cargado por `/dev/ttyUSB0`
  con `VRC_NETWORK_ONLY=1`: motores bloqueados e IMU sin inicializar.
- Laptop conserva Wi-Fi para internet; publicador oficial sintético por
  Ethernet en `192.168.0.100:2026` (IP DHCP, verificar en cada sesión).
- Serial confirmó `wifi=ok tcp=ok`; contador observado de 652 a más de 890
  mensajes válidos, aproximadamente 20/s, cero inválidos y cero saltos de
  secuencia durante la observación. Fase `IDLE`.
- Prueba cerrada: publicador de prueba detenido y serial liberado. Credenciales
  solo en `rover_control/config.h`, ignorado por Git. El rover conserva la
  versión de prueba de red, no la versión habilitada para movimiento.
- Esto valida transporte y recepción, no cámara real, identificación visual
  del marcador ni navegación. Se mantiene inicio en `RUNNING` por decisión
  del usuario mientras la organización resuelve la contradicción documental.

## Después: simulación visible y primer comportamiento completo

4. Ejecutar Webots y observar una ronda reproducible con trazas: cámara →
   visión → contrato v2 → control → ruedas. Ajustar parámetros físicos usando
   los datos disponibles; no marcar física calibrada sin medidas suficientes.
5. Un rover: aproximar, empujar y confirmar la entrega de **un cubo**. Probar
   parada por pose perdida y FINISHED. Corregir recuperación/reintentos.
6. Mientras se ejecuta el diagnóstico del rover 2, validar IR (posición y
   polaridad) e integrar protección de borde antes de maniobras autónomas
   próximas al límite. Ultrasonido/color siguen pospuestos por prioridad.
7. Dos rovers: tareas diferentes, cruces, cesión, bloqueo y recuperación;
   luego tres cubos y ronda completa. La prioridad por ID existente no
   garantiza por sí sola evitar colisiones o bloqueos.

Prueba final: ambos rovers reciben visión real, toman decisiones localmente,
entregan cubos y se detienen correctamente; conservar trazas y resultado.
Casos y criterios de simulación: `webots/VALIDACION.md`.
