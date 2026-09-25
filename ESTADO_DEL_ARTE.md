# Estado del arte: algoritmos para el Vision Rover Challenge — 23/09/2026

Criterio de lectura: **gana el más rápido**. El reglamento (§14) valora
primero la cantidad de cubos entregados y después el tiempo total. Cada
técnica se evalúa por cuánto tiempo ahorra, qué riesgo agrega y si cabe en un
ESP32 con visión cenital a 20 Hz y sin encoders. Complementa
[AUDITORIA_2026-09-23.md](AUDITORIA_2026-09-23.md).

## 1. El problema, en términos de la literatura

El reto combina cuatro problemas clásicos:

| Problema del reto | Nombre en la literatura | Referencias típicas |
| --- | --- | --- |
| Llevar cubos a zonas con dos robots | *Multi-Agent Pickup and Delivery* (MAPD) | Ma et al. 2017; almacenes tipo Kiva |
| Mover el cubo sin agarrarlo | Manipulación no prensil por empuje (*pusher–slider*) | Mason 1986; Lynch y Mason 1996; Hogan y Rodriguez 2016 |
| Repartir cubos entre rovers | Asignación de tareas multirrobot (MRTA), un robot por tarea | Gerkey y Matarić 2004; Kuhn 1955; CBBA |
| Moverse sin chocar | *Multi-Agent Path Finding* (MAPF) y evasión reactiva | Silver 2005; Sharon et al. 2015 (CBS); ORCA |

El entorno más parecido que existe es la **RoboCup Small Size League (SSL)**:
cámara cenital global, robots pequeños y un objeto que se conduce empujándolo.
Los equipos rápidos de SSL tienen tres cosas en común: estimación con
predicción de latencia, trayectorias continuas con perfiles de velocidad, y
una capa de estrategia separada del control. Es el referente a imitar.

**Función objetivo.** El tiempo de la ronda lo marca el **último** cubo
entregado. El objetivo correcto es minimizar el *makespan* (el máximo entre
los dos rovers), no la distancia total ni la suma de tiempos.

## 2. Dónde se va el tiempo hoy (medido en el simulador)

Con el firmware actual, cada maniobra es: avanzar ≤350 ms, detenerse, esperar
500 ms y exigir una captura nueva. Variando solo esos dos números, sin tocar la
lógica (8 semillas, física perturbada, `rover_control/sim`):

| Pausa / tramo máximo | 1 cubo | 1 cubo diagonal | 3 cubos, 2 rovers | Contactos |
| --- | --- | --- | --- | --- |
| 500 ms / 350 ms (actual) | 41 s | 53 s | 154 s | 0 |
| 200 ms / 350 ms | 19 s | 35 s | 132 s | 0 |
| 200 ms / 700 ms | 15 s | 30 s | 132 s | 0 |
| 100 ms / 1000 ms | 13 s | 21 s | 70 s | 0 |

**Conclusión: el mayor salto de velocidad no está en un algoritmo nuevo, sino
en dejar de detenerse.** Hoy el rover pasa más de la mitad del tiempo quieto.

Advertencia: el simulador no modela todavía la borrosidad de la cámara con el
rover en movimiento, ni si el cubo resbala más a mayor velocidad. Esos
números indican la dirección; la arena confirma la magnitud.

## 3. Familias de técnicas por subproblema

Formato: opciones → qué exigen → impacto en el tiempo → recomendación.

### 3.1 Estimación de estado y latencia

La pose que llega por TCP describe dónde **estaba** el rover hace
~50–100 ms (captura, procesamiento, Wi-Fi). Detenerse y esperar una captura
nueva es la forma más simple de no actuar sobre datos viejos, y también la
más lenta.

| Técnica | Idea | Costo en ESP32 |
| --- | --- | --- |
| Parar y observar (actual) | Actuar solo con el robot quieto | Nulo |
| Predicción por latencia | Avanzar la pose de la cámara con el mando aplicado desde `ts_ms` | Muy bajo |
| Filtro complementario cámara + giroscopio | El giroscopio da el rumbo entre capturas y la cámara corrige la deriva | Muy bajo |
| Filtro de Kalman (EKF) | Fusión óptima de cámara, giroscopio y modelo de ruedas | Bajo |

**Recomendación:** filtro complementario de rumbo (la IMU ya se lee a 104 Hz)
más predicción de posición con el modelo de mando. Es la pieza que habilita
el control continuo de §3.2. El EKF completo es opcional.

### 3.2 Control de movimiento

| Técnica | Idea | Velocidad |
| --- | --- | --- |
| Girar en sitio y avanzar recto (actual) | Robusto y simple | Baja: cada giro es una parada |
| *Pure pursuit* (Coulter 1992) | Apunta a un punto del camino a distancia fija; curva continua | Alta |
| Stanley (Thrun et al. 2006), Kanayama (1990) | Control por error lateral y de rumbo | Alta |
| Perfiles trapezoidales / *bang-bang* | Acelerar al máximo, frenar justo a tiempo | Alta en tramos largos |

Diferencial sin encoders: la velocidad real por rueda se infiere de la cámara
y del giroscopio. *Pure pursuit* con velocidad reducida en curvas cerradas es
el estándar práctico, fácil de ajustar, y tolera los 20 Hz de la visión si se
combina con §3.1.

**Recomendación:** mantener el giro en sitio solo para errores >60° y usar
*pure pursuit* para el resto. Frenar según la distancia al objetivo, no por
tramos de tiempo fijo.

### 3.3 Empuje del cubo (no prensil)

- **Mecánica cuasiestática** (Mason 1986; Goyal et al. 1991): con fricción y
  baja velocidad, el cubo se mueve según la "superficie límite" de su
  contacto con el piso.
- **Empuje estable** (Lynch y Mason 1996): un frente plano con dos puntos de
  contacto (las paletas del CenfoBot) mantiene el cubo pegado mientras la
  curvatura del recorrido sea moderada. Existe un cono de direcciones de
  empuje estable: fuera de él, el cubo resbala o gira.
- **Control realimentado** (Hogan y Rodriguez 2016): MPC sobre el modelo
  *pusher–slider*. Es el estado del arte en precisión, pero caro para un
  ESP32, y exige medir fricciones.

Lo que ya hacemos coincide con la práctica recomendada: frente plano, entrada
final perpendicular a la zona, aborto si el cubo resbala. **Para ganar
velocidad:** curvar *mientras* se empuja (dentro del cono estable) en lugar
de retroceder y realinear. Eso exige medir cuánto resbala el cubo, y el
parámetro del simulador (`cube_mu`) es hoy un supuesto.

### 3.4 Planificación de caminos

| Técnica | Característica | Adecuación |
| --- | --- | --- |
| A* en grilla (Hart et al. 1968) — **actual** | Óptimo en la grilla; caminos en zigzag | Buena; 43×43 celdas en ms |
| A* con costo de giro / estado (x, y, rumbo) | Penaliza cambios de rumbo | Muy buena: los giros son caros para este robot |
| Theta* / *any-angle* (Daniel et al. 2010) | Caminos rectos entre vértices, menos giros | Muy buena |
| Hybrid A* | Respeta la cinemática del vehículo | Excesivo para un diferencial |
| RRT / RRT* (Karaman y Frazzoli 2011) | Muestreo en espacios grandes | Innecesario en 1 m² |

**Recomendación:** A* con costo de giro, o Theta*. Menos giros significa
menos paradas: impacto directo en el tiempo con muy poco riesgo.

### 3.5 Asignación y secuencia de tareas

| Técnica | Qué optimiza | Nota |
| --- | --- | --- |
| Voraz (el más cercano) | Nada global | Simple; puede dejar a un rover con todo |
| Húngaro (Kuhn 1955) | Suma de costos, un cubo por rover | Óptimo solo para una ronda de asignación |
| **Secuencias completas min-max** | *Makespan* de toda la ronda | Con 3 cubos y 2 rovers hay muy pocas combinaciones: se enumeran todas |
| Subastas / CBBA (Choi et al. 2009) | Consenso descentralizado | Útil con comunicación; es lo que haría ESP-NOW |
| Token passing (MAPD, Ma et al. 2017) | Tareas en línea sin conflictos | Pensado para flotas grandes |

Hoy asignamos un cubo por rover y reasignamos al entregar, minimizando la
suma. El óptimo para el cronómetro es planificar la **ronda completa**:
quién hace qué cubos y en qué orden, minimizando el tiempo del último. Con 3
cubos se evalúan todas las repartos y órdenes en microsegundos. El costo debe
incluir giros, alineación y empuje, no solo distancias.

**Recomendación:** planificador exhaustivo min-max con costos en segundos
(calibrados con el simulador), histéresis para no cambiar de plan por ruido,
y confirmación del plan por ESP-NOW.

### 3.6 Coordinación y evasión entre rovers

| Técnica | Idea | Adecuación |
| --- | --- | --- |
| Prioridades (actual: el que empuja tiene paso) | Uno cede | Simple; puede esperar de más |
| Tabla de reservas espacio-tiempo / Cooperative A* (Silver 2005) | El segundo planifica evitando las celdas que el primero ocupará | Ideal para 2 robots, con ESP-NOW |
| CBS / PBS (Sharon et al. 2015; Ma et al. 2019) | MAPF óptimo o por prioridades | Excesivo para 2 robots |
| ORCA / obstáculos de velocidad (van den Berg et al.) | Evasión reactiva continua | Requiere control continuo (§3.2) |
| DWA (Fox et al. 1997) | Elegir velocidades seguras en una ventana | Alternativa a ORCA |

Con dos rovers, el mayor ahorro es **no cruzarse**: planes que separan las
zonas de trabajo desde la asignación (§3.5). La reserva espacio-tiempo por
ESP-NOW cubre lo que quede, y la guarda de contacto actual se mantiene como
última barrera de seguridad.

### 3.7 Arquitectura de decisión

Máquinas de estados (actual), árboles de comportamiento, o el esquema
jugadas–tácticas–habilidades de SSL (STP, Browning et al. 2005). Para este
tamaño de problema, la FSM actual con un planificador de ronda encima es
suficiente. No hace falta cambiar de arquitectura.

## 4. Hoja de ruta para ser el más rápido

Ordenada por ganancia esperada respecto de riesgo y esfuerzo:

| # | Cambio | Ganancia esperada | Riesgo | Base |
| --- | --- | --- | --- | --- |
| 1 | Pausas cortas y tramos largos, según distancia | **2× medido en simulación** | Bajo | **Aplicado 23/09** (150/1000 ms, AUDITORIA §8); confirmar en arena |
| 2 | Filtro complementario + predicción de latencia | Habilita el 3 | Bajo | IMU ya integrada |
| 3 | Control continuo (*pure pursuit*) con frenado por distancia | Otro salto grande (sin paradas) | Medio | §3.2 |
| 4 | Subir el PWM de traslado (hoy 18–20 %) | Proporcional a la velocidad | Medio: precisión y empuje | **Medir el viernes** |
| 5 | A* con costo de giro / Theta* | Menos giros y paradas | Bajo | §3.4 |
| 6 | Plan de ronda min-max | Reparto más equilibrado | Bajo | §3.5 |
| 7 | ESP-NOW: plan confirmado y reservas | Menos esperas y duplicados | Medio | §3.5–3.6 |
| 8 | Empuje con curvatura dentro del cono estable | Menos retrocesos y realineos | Medio | §3.3, medir `cube_mu` |

Regla de trabajo: **cada cambio se acepta solo si el simulador muestra menor
tiempo sin contactos ni pérdida de entregas**, y después se confirma en la
arena. La velocidad no vale nada si el cubo queda fuera de la zona: el
reglamento cuenta primero los cubos.

## 5. Qué medir en la arena para decidir

1. Velocidad real contra PWM (20, 30, 40, 60 %), con y sin cubo: define el 4.
2. Máxima velocidad y curvatura a la que el cubo no resbala de las paletas:
   define el 8 y ajusta `cube_mu` y `push_factor`.
3. Latencia real captura → rover (`ts_ms` contra la recepción en la misma
   máquina) y ruido de pose con el rover en movimiento: define el 2.
4. Distancia centro del rover → centro del cubo en contacto (hoy 107 mm).

## Referencias

- Browning, B. et al. (2005). STP: Skills, tactics, and plays for multi-robot control in adversarial environments. *Proc. IMechE, Part I*.
- Choi, H.-L., Brunet, L., How, J. (2009). Consensus-based decentralized auctions for robust task allocation. *IEEE Transactions on Robotics*.
- Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. CMU-RI-TR-92-01.
- Daniel, K., Nash, A., Koenig, S., Felner, A. (2010). Theta*: Any-angle path planning on grids. *JAIR*.
- Fox, D., Burgard, W., Thrun, S. (1997). The dynamic window approach to collision avoidance. *IEEE Robotics & Automation Magazine*.
- Gerkey, B., Matarić, M. (2004). A formal analysis and taxonomy of task allocation in multi-robot systems. *IJRR*.
- Goyal, S., Ruina, A., Papadopoulos, J. (1991). Planar sliding with dry friction. *Wear*.
- Hart, P., Nilsson, N., Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths. *IEEE Trans. SSC*.
- Hogan, F. R., Rodriguez, A. (2016). Feedback control of the pusher-slider system: A story of hybrid and underactuated contact dynamics. *WAFR*.
- Kanayama, Y. et al. (1990). A stable tracking control method for an autonomous mobile robot. *ICRA*.
- Karaman, S., Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *IJRR*.
- Kuhn, H. W. (1955). The Hungarian method for the assignment problem. *Naval Research Logistics Quarterly*.
- Lynch, K. M., Mason, M. T. (1996). Stable pushing: Mechanics, controllability, and planning. *IJRR*.
- Ma, H., Li, J., Kumar, T. K. S., Koenig, S. (2017). Lifelong multi-agent path finding for online pickup and delivery tasks. *AAMAS*.
- Ma, H., Harabor, D., Stuckey, P., Li, J., Koenig, S. (2019). Searching with consistent prioritization for multi-agent path finding. *AAAI*.
- Mason, M. T. (1986). Mechanics and planning of manipulator pushing operations. *IJRR*.
- Sharon, G., Stern, R., Felner, A., Sturtevant, N. (2015). Conflict-based search for optimal multi-agent pathfinding. *Artificial Intelligence*.
- Silver, D. (2005). Cooperative pathfinding. *AIIDE*.
- Thrun, S. et al. (2006). Stanley: The robot that won the DARPA Grand Challenge. *Journal of Field Robotics*.
- van den Berg, J., Guy, S., Lin, M., Manocha, D. (2011). Reciprocal n-body collision avoidance. *Robotics Research (ISRR)*.
- Zickler, S. et al. (2009). SSL-Vision: The shared vision system for the RoboCup Small Size League. *RoboCup Symposium*.
