# cirtesu_da3_mapping

Integración de ROS2 para mapeado 3D usando DA3-Streaming.

- [cirtesu\_da3\_mapping](#cirtesu_da3_mapping)
    - [Pre-requisitos y Configuración Inicial](#pre-requisitos-y-configuración-inicial)
        - [Sistema - Depth Anything 3 + ROS2](#sistema---depth-anything-3--ros2)
        - [DA3-Streaming - Descarga de Modelos](#da3-streaming---descarga-de-modelos)
        - [DA3 Streaming - Fichero de Configuración](#da3-streaming---fichero-de-configuración)
        - [Paquete - Fichero de configuración](#paquete---fichero-de-configuración)
    - [Mapeado en tiempo real](#mapeado-en-tiempo-real)
    - [Mapeado simultáneo](#mapeado-simultáneo)
    - [Procesar carpeta de imágenes](#procesar-carpeta-de-imágenes)
    - [Extraer imágenes desde vídeo](#extraer-imágenes-desde-vídeo)
    - [Visualización simple](#visualización-simple)

## Pre-requisitos y Configuración Inicial

### Sistema - Depth Anything 3 + ROS2

Es necesario tener un entorno con ROS2, DA3-Streaming y sus dependencias Python.

La opción más directa es usar los dockers siguientes:

- [diegomarzaa/dockers_cirtesu](https://github.com/diegomarzaa/dockers_cirtesu)

También es importante tener el repositorio de DA3 clonado, para configurar el funcionamiento de DA3-Streaming y para descargar los pesos.

- Fork con ajustes (recomendado): [diegomarzaa/Depth-Anything-3](https://github.com/diegomarzaa/Depth-Anything-3)
- Repositorio oficial: [ByteDance-Seed/Depth-Anything-3](https://github.com/ByteDance-Seed/Depth-Anything-3)

### DA3-Streaming - Descarga de Modelos

Desde el repositorio de DA3 (suponemos fork), los pesos se pueden descargar con:

```bash
cd /ruta/a/Depth-Anything-3/da3_streaming

# Modelo pequeño
bash scripts/download_weights.sh small

# Modelo base
bash scripts/download_weights.sh base

# Otros
bash scripts/download_weights.sh -h
```

El script deja la estructura así:

```text
da3_streaming/weights/
  DA3-SMALL/
    config.json
    model.safetensors
  DA3-BASE/
    config.json
    model.safetensors
  dino_salad.ckpt
```

### DA3 Streaming - Fichero de Configuración

En el repositorio de DA3, en el fichero `Depth-Anything-3/da3_streaming/configs/base_config.yaml`, poner la ruta de los pesos que hemos descargado. Por ejemplo, para el modelo DA3-SMALL:

```yaml
Weights:
  DA3: './weights/DA3-SMALL/model.safetensors'
  DA3_CONFIG: './weights/DA3-SMALL/config.json'
  SALAD: './weights/dino_salad.ckpt'
```

Aquí también se podrán modificar otros parámetros, como el tamaño de los chunks, el número de imágenes por chunk, etc.

### Paquete - Fichero de configuración

Finalmente, indicamos los parámetros del fichero de configuración de DA3-Streaming anterior, además de otras rutas de las carpetas de DA3.

Fichero: `config/mapping_defaults.yaml` en este paquete. 

Mínimo recomendable:

```yaml
da3_mapper:
  ros__parameters:
    da3_streaming_dir: /ruta/a/Depth-Anything-3/da3_streaming
    da3_src_dir: /ruta/a/Depth-Anything-3/src
    da3_config: /ruta/a/Depth-Anything-3/da3_streaming/configs/tu_config.yaml
    image_topic: /image_raw/compressed
```

En caso de usar fichero de configuración personalizado:

```bash
ros2 launch cirtesu_da3_mapping mapping.launch.py \
  params_file:=/ruta/a/mi_mapping_defaults.yaml
```

## Mapeado en tiempo real

Una vez configurado todo, se puede lanzar:

```bash
ros2 launch cirtesu_da3_mapping mapping.launch.py
```

o bien, si se quiere usar un fichero de configuración personalizado:

```bash
ros2 launch cirtesu_da3_mapping mapping.launch.py \
  params_file:=/ruta/a/mi_mapping_defaults.yaml
```

El mapeado incremental procesa chunks de frames en paralelo mientras siguen llegando imágenes, y va publicando el mapa acumulado en tiempo real.

```bash
# 1. Lanzar en modo incremental (por defecto)
ros2 launch cirtesu_da3_mapping mapping.launch.py processing_mode:=incremental
```

y desde otro terminal:

```bash
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

## Mapeado simultáneo

Modo alternativo pensado para buscar la máxima calidad, en lugar de dividir por chunks, se capturan imágenes continuamente y se reconstruye el mapa completo cada cierto tiempo. Pensado para ordenador con GPU potente y casos con pocas imágenes (por defecto hasta 80), una vez se pase de esto, se hará DA3-Streaming.

```bash
ros2 launch cirtesu_da3_mapping mapping_simultaneous.launch.py \
  image_topic:=/image_raw/compressed \
  max_normal_images:=80 \
  min_images_initial:=8 \
  min_new_images:=8 \
  min_seconds_between_runs:=30.0
```

Servicios útiles:

```bash
ros2 service call /mapping_simultaneous/start std_srvs/srv/Trigger {}
ros2 service call /mapping_simultaneous/run_now std_srvs/srv/Trigger {}
ros2 service call /mapping_simultaneous/stop std_srvs/srv/Trigger {}
ros2 topic echo /mapping_simultaneous/status
```

Parámetros habituales:

```bash
ros2 launch cirtesu_da3_mapping mapping_simultaneous.launch.py \
  process_res:=504 \
  num_max_points:=4000000 \
  conf_thresh_percentile:=10.0 \
  voxel_downsample:=0.0 \
  save_depth_outputs:=true \
  fallback_to_streaming:=true
```

## Procesar carpeta de imágenes

Este modo procesa una carpeta ya existente, publica el resultado como `PointCloud2` y guarda las salidas de DA3 en una carpeta de ejecución.

```bash
ros2 launch cirtesu_da3_mapping mapping_folder.launch.py \
  image_dir:=/home/usuario/DockerWorkspace/src/media/puerto/dataset_1_red_mar_imgs_contour/
```

Ejemplo con salida y parámetros explícitos:

```bash
ros2 launch cirtesu_da3_mapping mapping_folder.launch.py \
  image_dir:=/ruta/a/imagenes \
  output_dir:=/home/usuario/DockerWorkspace/tmp/da3_folder_mapping \
  max_normal_images:=80 \
  process_res:=504 \
  num_max_points:=4000000 \
  conf_thresh_percentile:=10.0 \
  save_depth_outputs:=true
```

Si `process_on_start:=false`, se puede lanzar manualmente:

```bash
ros2 service call /mapping_folder/run std_srvs/srv/Trigger {}
ros2 topic echo /mapping_folder/status
```

Para forzar DA3-Streaming:

```bash
ros2 launch cirtesu_da3_mapping mapping_folder.launch.py \
  image_dir:=/ruta/a/imagenes \
  force_streaming:=true
```

Cuando `save_depth_outputs:=true`, las salidas se guardan dentro del run en:

```text
depth/npz/
depth/colored/
```

## Extraer imágenes desde vídeo

El script `video_to_images.py` convierte un vídeo en una carpeta de imágenes para usar con `mapping_folder.launch.py`.

```bash
ros2 run cirtesu_da3_mapping video_to_images.py \
  /ruta/al/video.mp4 \
  /ruta/salida/imagenes \
  --fps 5 \
  --max-width 1280
```

Opciones útiles:

```bash
ros2 run cirtesu_da3_mapping video_to_images.py \
  /ruta/al/video.mp4 \
  /ruta/salida/imagenes \
  --start-sec 10 \
  --end-sec 40 \
  --max-frames 80 \
  --overwrite
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

Visualizar una salida GLB generada por DA3 normal:

```bash
ros2 launch cirtesu_da3_mapping visualize_glb.launch.py \
  glb_path:=/ruta/a/scene.glb

ros2 launch cirtesu_da3_mapping visualize_glb.launch.py \
  glb_path:=/ruta/a/scene.glb \
  voxel_downsample:=0.02
```
