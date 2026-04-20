# cirtesu_da3_mapping

Integración de ROS2 para mapeado 3D usando DA3-Streaming.

1. [Fase 0 - Visualización simple](#fase-0---visualización-simple)
  - cargar una salida ya existente de DA3-Streaming
  - abrir RViz2
  - ver el pointcloud y la geometría de cámaras
2. [Fase 1 - Grabación y procesado](#fase-1---grabación-y-procesado)
  - suscribirse a una cámara en ROS2
  - empezar a guardar imágenes una vez se llame un servicio
  - parar la grabación de imágenes con otro servicio
  - lanzar DA3-Streaming al terminar
  - publicar automáticamente el resultado en RViz2
3. [Fase 2 - Mapeado incremental](#fase-2---mapeado-incremental)
  - suscribirse a una cámara en ROS2
  - empezar una sesión incremental una vez se llame un servicio
  - guardar frames mientras llegan imágenes
  - procesar chunks solapados con DA3-Streaming en paralelo
  - publicar el mapa y la trayectoria acumulados en RViz2 mientras se van guardando frames

## Pre-requisitos

Los módulos, pesos y utilidades de DA3 se encuentran en el repositorio `Depth-Anything-3`:

```bash
git clone https://github.com/ByteDance-Seed/Depth-Anything-3.git
cd Depth-Anything-3
git submodule update --init --recursive
```

- !!!!!!!!!!!! PENDIENTE: Linkear commit exacto y indicar dependencias exactas. En el repo de ellos no está perfecto y yo tengo cambios en local.

## Fase 0 - Visualización simple

Visualizar en RViz2 una salida ya generada por DA3-Streaming.

```bash
# Solo PLY
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  ply_path:=/ruta/a/combined_pcd.ply

# PLY + poses de cámara + intrínsecos
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  ply_path:=/ruta/a/pcd/combined_pcd.ply \
  camera_poses_path:=/ruta/a/camera_poses.txt \
  intrinsics_path:=/ruta/a/intrinsic.txt

# Con voxel downsample (acelera RViz para nubes grandes)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  ply_path:=/ruta/a/combined_pcd.ply \
  voxel_downsample:=0.02
```

Topics:

- `/cirtesu/map_pointcloud` → `sensor_msgs/PointCloud2`
- `/cirtesu/camera_path` → `nav_msgs/Path`
- `/cirtesu/camera_axes` → `visualization_msgs/MarkerArray`
- `/cirtesu/camera_frustums` → `visualization_msgs/MarkerArray`

Parámetros principales:


| Parámetro           | Descripción                      |
| ------------------- | -------------------------------- |
| `ply_path`          | Ruta al fichero `.ply`           |
| `camera_poses_path` | Ruta a `camera_poses.txt`        |
| `intrinsics_path`   | Ruta a `intrinsic.txt`           |
| `voxel_downsample`  | `0` = desactivado; p.ej. `0.02`  |
| `frame_id`          | TF frame del header              |
| `publish_rate_hz`   | `0` = one-shot; `>0` = periódico |


## Fase 1 - Grabación y procesado

Grabar frames desde ROS2 y lanzar DA3-Streaming al terminar, publicando el resultado en RViz2.

Estado: `IDLE → RECORDING → PROCESSING → DONE / ERROR`

Primero, dejar configuradas las variables de entorno:

```bash
export DEPTH_ANYTHING_3_DIR=/ruta/a/Depth-Anything-3
export DA3_CONFIG=/ruta/a/da3_config.yaml

# Opcional:
export DA3_PYTHON=/path/to/custom/venv # En caso de usar conda u otros
export DA3_SESSION_BASE=/ruta/a/da3_sessions # Carpeta donde se guardan las sesiones (imágenes, PLYs, etc.)
```

Y luego, lanzar el launch:

```bash
# 1. Lanzar
ros2 launch cirtesu_da3_mapping record_and_map.launch.py

# 2. Empezar sesión
ros2 service call /frame_recorder/start_recording std_srvs/srv/Trigger {}

# 3. Mover el robot / cámara para capturar frames
ros2 topic echo /frame_recorder/status

# 4. Parar y procesar (DA3-Streaming corre en background)
ros2 service call /frame_recorder/stop_and_process std_srvs/srv/Trigger {}
```

Topics:

- `/camera/image_raw` → `sensor_msgs/Image` (entrada)
- `/cirtesu/map_pointcloud` → `sensor_msgs/PointCloud2` (salida)
- `/cirtesu/camera_path` → `nav_msgs/Path` (salida)
- `/cirtesu/camera_axes` → `visualization_msgs/MarkerArray` (salida)
- `/cirtesu/camera_frustums` → `visualization_msgs/MarkerArray` (salida)
- `/frame_recorder/status` → `std_msgs/String` (estado)

Ejemplo con cámara USB (v4l2):

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640,480]" \
  -r image_raw:=/camera/image_raw
```

## Fase 2 - Mapeado incremental

Mientras siguen llegando frames, publicando el mapa acumulado en vivo en RViz2.

> Igual que en la fase 1, dejar configuradas las variables de entorno primero.

```bash
# 1. Lanzar
ros2 launch cirtesu_da3_mapping incremental_map.launch.py

# 2. Empezar sesión
ros2 service call /incremental_mapper/start std_srvs/srv/Trigger {}

# 3. Mover el robot / cámara mientras se van publicando chunks
ros2 topic echo /incremental_mapper/status

# 4. Parar la sesión
ros2 service call /incremental_mapper/stop  std_srvs/srv/Trigger {}
```

Topics:

- `/image_raw/compressed` -> `sensor_msgs/msg/CompressedImage` (entrada)
- `/cirtesu/map_pointcloud` -> `sensor_msgs/PointCloud2` (salida)
- `/cirtesu/camera_path` -> `nav_msgs/Path` (salida)
- `/incremental_mapper/status` -> `std_msgs/String` (salida)

Otro ejemplo con parámetros explícitos:

```bash
ros2 launch cirtesu_da3_mapping incremental_map.launch.py \
  image_topic:=/image_raw/compressed \
  target_save_fps:=1.0 \
  voxel_downsample:=0.01
```

Si quieres que el nodo publique solo el chunk nuevo y dejar que RViz acumule visualmente los mensajes recibidos:

```bash
ros2 launch cirtesu_da3_mapping incremental_map.launch.py \
  image_topic:=/image_raw/compressed \
  publish_accumulated:=false
```

Notas:

- `publish_accumulated:=true` es el comportamiento por defecto: cada publicación contiene todo el mapa acumulado.
- `publish_accumulated:=false` publica solo el chunk recién procesado; el `Path` sigue publicándose acumulado.
- El RViz incluido en el paquete ya deja `Decay Time: 0` para `/cirtesu/map_pointcloud`, que es la configuración adecuada para conservar indefinidamente los chunks ya recibidos mientras llegan nuevos.

Ejemplo con un rosbag (usando tiempo simulado):

```bash
# 1. Lanzar
ros2 launch cirtesu_da3_mapping incremental_map.launch.py use_sim_time:=true

# 2. Empezar sesión
ros2 service call /incremental_mapper/start std_srvs/srv/Trigger {}

# 3. Reproducir el bag publicando /clock
ros2 bag play /ruta/al/bag --clock 50

# 4. Ver estado mientras se guardan frames y se procesan chunks
ros2 topic echo /incremental_mapper/status

# 5. Parar la sesión al terminar
ros2 service call /incremental_mapper/stop std_srvs/srv/Trigger {}
```

