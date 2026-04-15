# cirtesu_da3_mapping

Paquete ROS2 para mapeado 3D usando DA3-Streaming.

1. **Modo visualización**
   - [Fase 0](#fase-0)
   - cargar una salida ya existente de DA3-Streaming
   - abrir RViz2
   - ver el pointcloud y la geometría de cámaras

2. **Modo record -> DA3-Streaming -> map**
   - [Fase 1](#fase-1)
   - suscribirse a una cámara en ROS2
   - empezar a guardar imágenes mediante un servicio
   - parar la grabación con otro servicio
   - lanzar DA3-Streaming al terminar
   - publicar automáticamente el resultado en RViz2

3. **Modo mapping while recording**
   - [Fase 2](#fase-2)
   - ...

Se cubre:

- publicación de `combined_pcd.ply` como `sensor_msgs/PointCloud2`
- visualización de `camera_poses.txt`: `nav_msgs/Path`, ejes de cámara y frustums
- launch con RViz2 y static TF para visualizar el mundo DA3 dentro de la convención de ROS
- grabación básica de sesiones desde un topic de imagen y procesado posterior con DA3-Streaming

## Fase 0

Validar la visualización en ROS2 de una salida ya generada por DA3-Streaming.

Publicación de:

- `combined_pcd.ply` como `sensor_msgs/PointCloud2`
- `camera_poses.txt` como `nav_msgs/Path`
- ejes XYZ de cámara como `visualization_msgs/MarkerArray`
- frustums de cámara usando `camera_poses.txt + intrinsic.txt` en `visualization_msgs/MarkerArray`

Topics:

- `/cirtesu/map_pointcloud` -> `sensor_msgs/PointCloud2`
- `/cirtesu/camera_path` -> `nav_msgs/Path`
- `/cirtesu/camera_axes` -> `visualization_msgs/MarkerArray`
- `/cirtesu/camera_frustums` -> `visualization_msgs/MarkerArray`

Launch:
```bash
# Con el PLY por defecto (diego_room_few)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py

# Con otro PLY
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  ply_path:=/ruta/a/otro.ply

# Con voxel downsample (acelera RViz para nubes grandes)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  voxel_downsample:=0.02

# Con una salida DA3 completa (PLY + poses + intrínsecos)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  ply_path:=/ruta/a/pcd/combined_pcd.ply \
  camera_poses_path:=/ruta/a/camera_poses.txt \
  intrinsics_path:=/ruta/a/intrinsic.txt
```

Nodos sueltos:

```bash
ros2 run cirtesu_da3_mapping ply_publisher_node.py
ros2 run cirtesu_da3_mapping camera_poses_publisher_node.py
```

Parámetros del launch:

- `ply_path` -> ruta al fichero `.ply`
- `frame_id` -> TF frame del header (hijo del static TF `map → da3_world`)
- `topic` -> topic de publicación del pointcloud
- `publish_rate_hz` -> `0` = one-shot con `transient_local`; `>0` = periódico
- `voxel_downsample` -> `0` = desactivado; ej. `0.02` para reducir puntos
- `camera_poses_path` -> ruta al fichero `camera_poses.txt`
- `intrinsics_path` -> ruta al fichero `intrinsic.txt`
- `camera_publish_rate_hz` -> `0` = one-shot; `>0` = periódico
- `camera_axis_length` -> longitud de los ejes XYZ de la cámara
- `camera_axis_line_width` -> ancho de los ejes XYZ de la cámara
- `camera_frustum_depth` -> profundidad de los frustums
- `camera_frustum_line_width` -> ancho de los frustums

## Fase 1

Grabar una sesión desde ROS2 y lanzar DA3-Streaming al terminar, publicando el resultado automáticamente en RViz2:

**`scripts/frame_recorder_node.py`** — nodo principal. Máquina de estados:

```text
IDLE → RECORDING → PROCESSING → DONE
                              → ERROR
```

**`launch/record_and_map.launch.py`** — lanza:

- `frame_recorder`
- static TF
- RViz2

**`config/frame_recorder.yaml`** — parámetros por defecto del recorder.

Servicios:

- `/frame_recorder/start_recording` -> `std_srvs/Trigger` -> Crea carpeta de sesión y empieza a guardar frames
- `/frame_recorder/stop_and_process` -> `std_srvs/Trigger` -> Para el recording y lanza DA3-Streaming en background

Muestreo de imágenes:

- `target_save_fps` -> FPS objetivo de guardado

Ejemplo simple con la cámara del portátil:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640,480]" \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

Flujo de uso:

```bash
# 1. Lanzar el sistema
ros2 launch cirtesu_da3_mapping record_and_map.launch.py

# 2. Empezar sesión
ros2 service call /frame_recorder/start_recording std_srvs/srv/Trigger {}

# 3. Mover el robot / cámara para capturar frames
ros2 topic echo /frame_recorder/status

# 4. Parar y procesar
ros2 service call /frame_recorder/stop_and_process std_srvs/srv/Trigger {}
```

Después:

- DA3-Streaming corre en background y el log del nodo muestra su salida
- cuando termina, se publica automáticamente en RViz:
  - `/cirtesu/map_pointcloud`
  - `/cirtesu/camera_path`
  - `/cirtesu/camera_axes`
  - `/cirtesu/camera_frustums`

Parámetros del launch:

- `image_topic` -> topic de cámara a grabar
- `target_save_fps` -> FPS objetivo de guardado
- `session_base_dir` -> carpeta base para sesiones
- `da3_python` -> intérprete del entorno DA3
- `da3_script` -> ruta a `da3_streaming.py`
- `da3_config` -> config YAML de DA3-Streaming
- `voxel_downsample` -> downsample opcional al cargar el PLY final
- `frame_id` -> frame de salida DA3

Nota:

- La corrección de ejes **OpenCV → ROS** ya se aplica vía TF; la orientación global de DA3-Streaming dependerá de la secuencia si no hay referencias externas (IMU, ArUco, TF).
