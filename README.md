# cirtesu_da3_mapping

Integración de ROS2 para mapeado 3D usando DA3-Streaming.

- [cirtesu\_da3\_mapping](#cirtesu_da3_mapping)
    - [Pre-requisitos](#pre-requisitos)
    - [Configuración inicial](#configuración-inicial)
    - [Mapeado en tiempo real](#mapeado-en-tiempo-real)
    - [Visualización simple](#visualización-simple)

## Pre-requisitos

Los módulos, pesos y utilidades de DA3 se encuentran en el repositorio `Depth-Anything-3`:

```bash
git clone https://github.com/ByteDance-Seed/Depth-Anything-3.git
cd Depth-Anything-3
git submodule update --init --recursive
```

## Configuración inicial

Importante: edita el fichero `config/mapping_defaults.yaml` con las rutas de instalación de DA3 del paso anterior.

En caso de haber instalado las dependencias de DA3 en un entorno Python propio (venv, conda), configura también la variable de entorno:

```bash
# En tu ~/.bashrc o ~/.zshrc:
export DA3_PYTHON=/opt/venvs/da3/bin/python3
```

En caso de crear otro fichero de configuración, se puede pasar como argumento al launch `params_file`.

## Mapeado en tiempo real

```bash
ros2 launch cirtesu_da3_mapping mapping.launch.py
```

El mapeado incremental procesa chunks de frames en paralelo mientras siguen llegando imágenes, y va publicando el mapa acumulado en tiempo real.

```bash
# 1. Lanzar en modo incremental (por defecto)
ros2 launch cirtesu_da3_mapping mapping.launch.py processing_mode:=incremental

# 2. Empezar sesión
ros2 service call /da3_mapper/start std_srvs/srv/Trigger {}

# 3. Mover la cámara — los chunks se procesan y publican solos
ros2 topic echo /da3_mapper/status

# 4. Parar la sesión (el worker drena los frames restantes antes de salir)
ros2 service call /da3_mapper/stop std_srvs/srv/Trigger {}
```

También se pueden recolectar los frames primero y al final procesarlos.

```bash
# 1. Lanzar en modo on-stop
ros2 launch cirtesu_da3_mapping mapping.launch.py processing_mode:=on_stop

# 2, 3, 4. - Mismos comandos
```

El resto de parámetros configurables se pueden ver en `config/mapping_defaults.yaml`.

> **Nota RViz2 — `publish_accumulated: false`:** Para ahorrar ancho de banda, se puede configurar `publish_accumulated: false`, habrá que configurar `Decay Time` a un valor alto en rviz.

Otro ejemplo de uso es con rosbag:

```bash
# 1. Lanzar con tiempo simulado (puedes cambiar el modo con processing_mode:=incremental o processing_mode:=on_stop)
ros2 launch cirtesu_da3_mapping mapping.launch.py use_sim_time:=true

# Mismos comandos para empezar y parar la sesión.

# Reproducir el bag publicando /clock
ros2 bag play /ruta/al/bag --clock

# Parar al terminar
```

## Visualización simple

Visualizar en RViz2 una salida ya generada por DA3-Streaming, en formato PLY.

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
