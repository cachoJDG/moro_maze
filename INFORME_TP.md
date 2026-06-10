# Informe TP — moro_maze (SOAR + MORO)

Documento de trabajo para discutir con el profe. Objetivo: explicar **qué se hizo,
cómo funciona cada parte, qué decisiones se tomaron y cuáles son las limitaciones
conocidas** — sin esconder nada, para que pueda corregir lo que esté mal.

---

## 0. Objetivo del TP

El robot (TurtleBot3 burger) está en un laberinto en Gazebo. Tiene que:
1. **Localizarse** sin saber dónde arrancó (SOAR, kNN sobre el LaserScan).
2. **Planificar globalmente** un camino hasta una salida (SOAR, grafo + búsqueda BFS).
3. **Navegar localmente** ese camino con un controlador propio por **forward
   simulation** (MORO), publicando comandos de velocidad a `/cmd_vel`.

Se corre con: `ros2 launch moro_maze simulation_launch.py x_pose:=X y_pose:=Y`.

---

## 1. Arquitectura general y flujo de datos

Tres nodos propios (Python) + lo que tomamos de Nav2:

```
                 /map_server/map (servicio)        /scan
                          │                           │
                          ▼                           ▼
   ┌──────────────────────────────────────────────────────┐
   │ localisation_node  (SOAR pasos 1-2: kNN, un disparo)   │
   │  - pide el mapa por servicio, arma kNN free/wall       │
   │  - con el 1er scan estima la pose (x,y) entera         │
   └──────────────────────────────────────────────────────┘
        │ /estimated_pose            │ /initialpose (semilla AMCL)
        ▼                            ▼
   ┌─────────────────────────┐   ┌──────────────────────────┐
   │ global_planner_node     │   │ AMCL (Nav2)               │
   │ (SOAR paso 3)           │   │  - localización continua  │
   │  - grafo + BFS          │   │  - publica tf map→odom    │
   │  - /global_path         │   └──────────────────────────┘
   └─────────────────────────┘            │ tf map→base_link
        │ /global_path                     ▼
        ▼                         ┌──────────────────────────┐
   ┌────────────────────────────│ local_planner_node (MORO) │
   │  - localiza por tf map→base_link (de AMCL)              │
   │  - DWA por forward simulation                           │
   │  - publica /cmd_vel, /local_trajectory, /current_goal   │
   └──────────────────────────────────────────────────────┘
                          │ /cmd_vel (TwistStamped)
                          ▼
                 ros_gz_bridge → Gazebo (mueve el robot)
```

### Qué usamos de Nav2 (importante para ser transparentes)
- **`map_server`**: nos sirve el mapa por el servicio `/map_server/map`.
- **`AMCL`**: hace la **localización continua mientras el robot se mueve** y publica
  la transformada `map→odom`. Nuestro `localise_robot()` de MORO consulta la tf
  `map→base_link` (exactamente como el `localiseRobot` del cookbook de MORO).
- **NO usamos** el planner ni el controller de Nav2 (`planner_server`,
  `controller_server`, `bt_navigator`). El launch igual los levanta (vienen en el
  bringup), pero quedan **ociosos** porque nunca mandamos un goal a
  `navigate_to_pose`. La planificación y el control son 100% nuestros.

> **Punto a comentar con el profe:** que el launch levante todo Nav2 aunque solo
> usemos `map_server` + `AMCL` es algo que se podría recortar a "solo
> localización". No lo hicimos para no tocar el launch del bringup. Como efecto
> colateral, el `collision_monitor` (que queda activo y ocioso) tira warnings
> `invalid source` por el scan (ver sección 6). No afecta la navegación.

### Topics
| Topic | Tipo | Quién publica → quién consume |
|---|---|---|
| `/estimated_pose` | `PoseStamped` | localisation → global_planner |
| `/initialpose` | `PoseWithCovarianceStamped` | localisation → AMCL (semilla) |
| `/global_path` | `nav_msgs/Path` | global_planner → local_planner |
| `/cmd_vel` | `TwistStamped` | local_planner → bridge → Gazebo |
| `/local_trajectory` | `nav_msgs/Path` | local_planner (visualización) |
| `/current_goal` | `PoseStamped` (base_link) | local_planner (visualización) |

---

## 2. SOAR — Localización (localisation_node)

**Cookbook pasos 1 y 2.**

### Paso 1 — Mapa y scan
- El mapa se pide **una vez** por el servicio `/map_server/map` (`GetMap`), no por
  topic. Es un `OccupancyGrid` (array 1D + metadata: ancho, alto, resolución, origen).
- Se deserializa a 2D y, con `origin` y `resolution`, se calculan las posiciones en
  **coordenadas del mundo** de cada celda libre y de cada pared.
- El `/scan` (LaserScan) se transforma de polar a cartesiano. Para que las paredes
  detectadas por el láser coincidan con las del mapa (las paredes tienen espesor y el
  láser solo ve el borde), **se corren los puntos del scan medio celda hacia afuera**
  (`+0.5 * resolution`), tal como sugiere el cookbook.
- Mapa real: 27×27, `resolution = 0.166667 m/celda`, `origin = (-0.75, -0.75)`.
  Celdas libres = 589, paredes = 140.

### Paso 2 — Localización por kNN
- Se entrena un **k-Nearest Neighbors** (`sklearn`, con fallback propio en numpy si no
  está) con `k=1`: `X` = posiciones de todas las celdas, `y` = etiqueta (0 libre / 1
  pared). Con `k=1` el modelo convierte el mapa discreto en una representación continua.
- **Supuesto del cookbook:** el robot arranca en coordenadas **enteras** (0..3 en x e
  y) y **mirando al eje x (yaw = 0)**. Se generan 16 poses candidatas (las enteras
  libres).
- Para cada pose candidata: se traslada el scan a esa posición y se le pregunta al kNN
  cuántos puntos caen sobre "pared". El **score = (predicciones 'pared') / (total)**.
  La pose con mayor score gana.
- Se publica `/estimated_pose`, `/estimated_pose_cov` y se siembra `/initialpose`
  (para AMCL). Después se **borra la suscripción al scan** (localización de un solo
  disparo).
- En las pruebas el score dio **0.95–1.00** en los 6 spawns, siempre con la pose
  correcta.

> **Limitación honesta:** la localización es de **un solo disparo** y **asume yaw = 0**
> (es el supuesto del cookbook). Si el robot spawneara rotado, fallaría. Los 6 spawns
> de la consigna usan yaw = 0, así que está cubierto. El seguimiento fino *durante el
> movimiento* lo hace AMCL.

---

## 3. SOAR — Planificación global (global_planner_node)

**Cookbook paso 3 (grafo + búsqueda + publicación).**

### Creación del grafo
- Se muestrean celdas libres sobre una **grilla regular** (cada `graph_step=6`
  celdas). Esos puntos caen en las columnas/filas 4, 10, 16, 22 = world 0, 1, 2, 3 →
  por eso los nodos quedan en coordenadas enteras, igual que el cookbook
  (`'4.4'`, `'10.4'`, etc.). El nombre del nodo codifica la posición: `'gx.gy'`.
- Se conectan nodos vecinos en la misma fila/columna **si hay línea de vista libre**
  (chequeo de Bresenham celda por celda: `line_is_free`). Cada arista guarda
  `parent`, `child`, `cost` (distancia).

### Detección de salidas — `detect_border_exits` (¡acá hubo un fix importante!)
- El laberinto tiene **un anillo exterior de padding que es todo libre**. La versión
  original escaneaba ese anillo y devolvía los **puntos medios de cada lado**
  (`(13,0),(13,26),(0,13),(26,13)`) — que **NO son las aberturas reales**. El robot
  terminaba yendo a una celda interior pegada a una pared y decía "listo" sin salir.
- **Fix:** ahora se localiza el **rectángulo de paredes** (bounding box de las celdas
  ocupadas) y se escanea cada uno de sus 4 lados buscando **huecos** (tramos libres en
  la pared). Esos huecos son las salidas reales. Devuelve la celda del borde exterior
  alineada con cada hueco.
- Resultado para este mapa: **dos salidas reales**
  - `(0, 4)` → world **(0, 0)** — abertura abajo-izquierda (la pared izquierda falta en
    las filas gy 2–6). La llamamos **GAP A**.
  - `(26, 22)` → world **(3, 3)** — abertura arriba-derecha (la pared derecha falta en
    gy 20–24). La llamamos **GAP B**.
- El cookbook, para spawn (2,1), sale por **(3,3)** = GAP B → **coincide** con nuestro
  resultado.

### Búsqueda (BFS) y reconstrucción del path
- Se ancla el start del robot y la salida a sus nodos visibles más cercanos
  (`nearest_visible_node`) y se corre **BFS** desde el start hasta la salida más
  cercana. Se reconstruye el path siguiendo `child → parent` desde el goal.
- Hay un **fallback de resolución**: si con `graph_step=6` no hay path, reintenta con 3
  y con 1.

### Ajustes del path antes de publicar (agregados nuestros)
- **Drop de overshoot:** si el último nodo del grafo se pasa de la salida, se reemplaza
  por la salida real (cuando hay línea de vista).
- **Extensión al doorway (`exit_inset_cells=4`):** se extiende el path hasta una celda
  4 celdas adentro del borde. 4 celdas cae justo en el nodo entero (world 3.0), que es
  donde termina el path del cookbook. (Goals muy pegados al borde rompían el
  NavfnPlanner de Nav2 — pero como ahora no usamos Nav2 para planear, esto quedó más
  como margen de seguridad.)
- **Densificación (`execution_step_cells=2`):** el path del grafo es "sparse" (saltos
  de hasta 1 m entre nodos). Se agregan waypoints intermedios cada ~2 celdas para que
  el controlador local lo siga sin saltos grandes. **Esto es un agregado nuestro**, no
  está en el cookbook (que asume el path sparse directo).
- **Caso borde (fix):** si el robot **arranca sobre la salida** (path de un solo nodo,
  spawns (0,0) y (3,3)), NO se agrega la extensión interior (antes generaba un
  ida-y-vuelta `(3,3)→(2.67,3)→(3,3)`). Queda `[start]` y el controlador detecta que ya
  está en el goal y frena.
- Se publica `/global_path` (`nav_msgs/Path`, frame `map`).

> **Para comentar con el profe:** la densificación y la extensión al doorway son
> agregados nuestros para que el control ande prolijo. Si prefiere el path sparse "puro"
> del cookbook, se desactivan con parámetros.

---

## 4. MORO — Navegación local (local_planner_node)

**Cookbook paso 3 de MORO: las 4 secciones + el loop.** Es un controlador tipo **DWA
(Dynamic Window Approach) por forward simulation**: en cada ciclo genera muchos
comandos posibles, simula a dónde llevarían al robot, los puntúa y publica el mejor.

La matemática está en `control_utils.py`; el nodo ROS en `local_planner_node.py`.

### 4.1 Determinar el goal relativo al robot
- Se localiza el robot con la tf `map→base_link` (de AMCL); el yaw se saca del
  cuaternión.
- Se elige el goal actual del `/global_path`. El `θ` de cada waypoint lo calculamos
  como el **rumbo hacia el siguiente** waypoint.
- Para llevar el goal al **marco del robot** se usan **matrices de transformación
  homogéneas** (`pose2tf_mat` / `tf_mat2pose`):
  `goal_rel = inv(T_robot) · T_goal`. Es la inversa de la tf map→robot multiplicada por
  la tf map→goal, tal cual el cookbook.

### 4.2 Generar señales de control válidas — `generateControls`
- Genera combinaciones de `(v, w)` **cerca del último comando** (por inercia no puede
  saltar bruscamente). Límites usados: `v ∈ [0, 0.22] m/s`, `w ∈ [-1.4, 1.4] rad/s`,
  con saltos máximos por ciclo `v_acc=0.1`, `w_acc=1.0` y granularidad
  `v_step=0.02`, `w_step=0.1`. Da ~126 candidatos por ciclo.

### 4.3 Simular el resultado — forward simulation
- `forwardKinematics` (modelo de cinemática diferencial, tomado del cookbook /
  Thrun) y `PT2Block` (modelo de la **inercia** de la velocidad lineal, aproximación de
  Tustin, tomado del cookbook). **Estos dos bloques están citados** en el código como
  fuente del cookbook.
- Para cada candidato se simula partiendo de `[0,0,0]` (marco del robot) durante
  `horizon=12` pasos de `ts=0.2 s` (≈2.4 s de lookahead), pasando la `v` por el PT2.

### 4.4 Función de coste — `costFn`
- `costo = eᵀ·Q·e + uᵀ·R·u`, con `e = |pose − goal|` (el error angular se envuelve a
  `[-π, π]` antes) y `u = |control|`. Menor costo = mejor.
- **Pesos usados (acá hay una decisión a comentar):**
  - `Q = diag(1, 1, 0)` → `q_theta = 0`. **El cookbook sugiere `q_theta = 0.5`**, pero
    para **seguir waypoints en el laberinto nos importa la posición, no la
    orientación** de cada waypoint intermedio. Con `q_theta > 0` el robot, en cada
    esquina, **giraba en el lugar** para alinear el ángulo (el error angular de 90° al
    cuadrado dominaba el coste) en vez de avanzar. Con `q_theta = 0` sigue posiciones y
    dobla en arco. El cookbook dice explícitamente "tunee su controlador" — esto es
    ese tuning.
  - `R = diag(0.05, 0.02)` → castigo suave al control (más bajo que el 0.1 del
    cookbook) para que pueda girar libre en las esquinas.

### 4.5 Publicar a ROS y loop
- Se elige el control de **menor costo** (`argmin`) y se publica a `/cmd_vel` como
  `geometry_msgs/TwistStamped` (el bridge y el burger.yaml usan TwistStamped).
- Para la visualización (requisito de Moodle) se publican también:
  - `/local_trajectory` (`nav_msgs/Path`, frame `base_link`): la trayectoria simulada
    de menor costo.
  - `/current_goal` (`PoseStamped`, frame `base_link`): el goal actual relativo al robot.
- **Avance de waypoint (fix de overshoot):** se avanza al siguiente waypoint cuando el
  robot está dentro de `goal_tolerance=0.25 m` **o cuando ya lo pasó de largo** (el
  siguiente está más cerca que el actual). Sin esto, si el robot se pasaba un waypoint a
  velocidad, se quedaba persiguiéndolo hacia atrás y se trababa.
- **Parada:** al llegar al último goal (dentro de `final_goal_tolerance=0.2 m`) publica
  `(0, 0)` para frenar el robot y loguea `local planner: final goal reached, robot
  stopped`. **Esa línea es la señal de que todo el pipeline funcionó.**
- El loop corre a `control_rate=5 Hz`.

---

## 5. Decisiones de diseño / cambios hechos (resumen para transparencia)

| # | Qué | Por qué | ¿Desvío del cookbook? |
|---|---|---|---|
| 1 | `detect_border_exits` busca huecos en la pared, no en el anillo de padding | Antes mandaba el robot a celdas interiores, no a la salida real | No (corrige un bug nuestro) |
| 2 | `q_theta = 0` en el coste | Evitar que gire en el lugar en cada esquina; seguir posición | Sí (cookbook: 0.5). Es "tuning", permitido |
| 3 | `R = diag(0.05, 0.02)` | Que doble libre en esquinas | Leve (cookbook: 0.1) |
| 4 | Densificación del path | Que el control siga sin saltos grandes | Agregado nuestro |
| 5 | Avance de waypoint por overshoot | Que no se trabe si se pasa de largo | Agregado nuestro |
| 6 | Path `[start]` si arranca en la salida | Evitar ida-y-vuelta degenerado | Caso borde |
| 7 | `/cmd_vel` como TwistStamped | Lo que espera el bridge / burger.yaml | Detalle de implementación |

---

## 6. Limitaciones conocidas / cosas a discutir con el profe

Para que nos corrija si algo está mal:

1. **El controlador local NO tiene término de obstáculos en el coste.** Solo penaliza
   distancia al goal + control. Confía 100% en que el `/global_path` es libre de
   colisiones (lo es, porque sale del grafo de visibilidad). **Esto coincide con la
   `costFn` del cookbook** (que tampoco tiene término de obstáculos), pero conviene
   saberlo: si el path pasara cerca de una pared, el control no la esquiva por sí solo.
2. **`q_theta = 0`**: el robot no respeta la orientación de los goals, solo la posición.
   Para escapar el laberinto alcanza, pero es un desvío del cookbook. (El `θ` igual se
   calcula y se publica en `/current_goal`, solo que no pesa en el coste.)
3. **Localización de un disparo y con yaw = 0**: es el supuesto del cookbook. No maneja
   spawns rotados. El seguimiento durante el movimiento lo hace AMCL.
4. **Dependencia de AMCL**: la localización *mientras navega* (la tf `map→base_link`)
   viene de AMCL, no de nuestro kNN. Nuestro kNN solo da la pose inicial y siembra AMCL.
   Esto es exactamente lo que hace el `localiseRobot` del cookbook de MORO.
5. **Nav2 ocioso + `collision_monitor`**: el launch levanta todo el stack de Nav2 pero
   solo usamos `map_server` + `AMCL`. El `collision_monitor` queda activo y, por la
   **simulación lenta** (el `/scan` baja a ~3.7 Hz y su `source_timeout` por defecto es
   0.2 s), tira muchos warnings `Robot to stop due to invalid source`. **No es código
   nuestro y no afecta** (publica a `/cmd_vel`, pero nuestro nodo es el único publisher
   real; lo verificamos con `ros2 topic info /cmd_vel --verbose` → 1 publisher =
   `local_planner_node`). Se podría limpiar recortando el launch a "solo localización".
6. **Velocidad de la sim**: con RTF < 1 las corridas tardan 1–3 min. No es un problema
   del código sino de CPU.

---

## 7. Validación (6 spawns de la consigna)

Todos localizan bien, encuentran la salida real y el robot llega y frena
(`final goal reached`):

| Spawn | Localización | Path BFS | Sale por |
|---|---|---|---|
| (0,0) | ✅ score 1.00 | `['4.4']` (trivial, ya está en la salida) | GAP A |
| (0,3) | ✅ 0.95 | `['4.22'…'4.4']` | GAP A |
| (3,0) | ✅ 0.98 | `['22.4','22.10','22.16','22.22']` | GAP B |
| (3,3) | ✅ 1.00 | `['22.22']` (trivial) | GAP B |
| (1,1) | ✅ 0.96 | `['10.10','10.4','4.4']` | GAP A |
| (2,2) | ✅ 0.98 | `['16.16','22.16','22.22']` | GAP B |

Para spawn (2,1) (el de referencia del cookbook) el path es
`['16.10','22.10','22.16','22.22']` = (2,1)→(3,1)→(3,2)→(3,3), **idéntico al cookbook**.

---

## 8. Cómo correr y verificar

```bash
# Build
cd /opt/ros2_ws
colcon build --packages-select moro_maze --symlink-install && source install/setup.bash

# Correr un spawn
ros2 launch moro_maze simulation_launch.py x_pose:=2.0 y_pose:=1.0

# Verificar la cadena completa en el log:
grep -E "initial localisation result|detected exits|bfs found path|local planner received|final goal reached" <log>
# La señal de éxito total es:  "local planner: final goal reached, robot stopped"

# Ver la visualización de MORO (requisito Moodle):
ros2 topic echo /local_trajectory --once   # trayectoria simulada (Path, base_link)
ros2 topic echo /current_goal --once        # goal relativo (PoseStamped, base_link)
ros2 topic echo /cmd_vel                     # comando publicado por nuestro controlador
```

---

## 9. Archivos del paquete

| Archivo | Qué hace |
|---|---|
| `moro_maze/localisation_node.py` | SOAR pasos 1–2: kNN, localización de un disparo |
| `moro_maze/global_planner_node.py` | SOAR paso 3: grafo + BFS + detección de salidas + path |
| `moro_maze/local_planner_node.py` | MORO: controlador local por forward simulation |
| `moro_maze/map_utils.py` | `GridMap`: mapa, conversiones, `detect_border_exits` |
| `moro_maze/search_utils.py` | grafo, BFS, reconstrucción, Bresenham, densificación |
| `moro_maze/control_utils.py` | MORO: transformadas, generateControls, forwardKinematics, PT2, costFn, evaluateControls |
| `launch/simulation_launch.py` | Gazebo + Nav2 (map_server+AMCL) + RViz + nuestros 3 nodos |

> **Citas:** `forwardKinematics` y `PT2Block` en `control_utils.py` están tomados del
> MORO Project Cookbook (que a su vez referencia el ejercicio de la clase 4 y Thrun,
> *Probabilistic Robotics*), y están marcados como tal en los comentarios del código.
