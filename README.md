# cirtesu_da3_mapping

ROS2 Humble package for the DA3-Streaming mapping pipeline.

Publica un `.ply` generado por `da3_streaming` como topic `sensor_msgs/PointCloud2` con QoS `transient_local` y lo visualiza en RViz2.

Además, publica la geometría de cámaras asociada a una salida de DA3:

- `camera_poses.txt` como `nav_msgs/Path`
- ejes XYZ de cámara como `visualization_msgs/MarkerArray`
- frustums de cámara usando `camera_poses.txt + intrinsic.txt`

```bash
# Con el PLY por defecto (diego_room_few)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py

# Con otro PLY
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py ply_path:=/ruta/a/otro.ply

# Con voxel downsample (acelera RViz para nubes grandes)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py voxel_downsample:=0.02

# Con otra salida DA3 completa (PLY + poses + intrínsecos)
ros2 launch cirtesu_da3_mapping visualize_ply.launch.py \
  ply_path:=/ruta/a/pcd/combined_pcd.ply \
  camera_poses_path:=/ruta/a/camera_poses.txt \
  intrinsics_path:=/ruta/a/intrinsic.txt
```

O solo los nodos, sin RViz:
```bash
ros2 run cirtesu_da3_mapping ply_publisher_node.py
ros2 run cirtesu_da3_mapping camera_poses_publisher_node.py
```

Topics:

- `/cirtesu/map_pointcloud` -> `sensor_msgs/PointCloud2`
- `/cirtesu/camera_path` -> `nav_msgs/Path`
- `/cirtesu/camera_axes` -> `visualization_msgs/MarkerArray`
- `/cirtesu/camera_frustums` -> `visualization_msgs/MarkerArray`

## Parámetros

- `ply_path` -> ruta al fichero `.ply`
- `frame_id` -> TF frame del header (hijo del static TF `map → da3_world`)
- `topic` -> topic de publicación
- `publish_rate_hz` -> 0 = one-shot transient_local; >0 = periódico
- `voxel_downsample` -> 0 = desactivado; ej. `0.02` para reducir puntos
- `camera_poses_path` -> ruta al fichero `camera_poses.txt`
- `intrinsics_path` -> ruta al fichero `intrinsic.txt`
- `camera_publish_rate_hz` -> 0 = one-shot transient_local; >0 = periódico
- `camera_axis_length` -> longitud de los ejes XYZ de la cámara
- `camera_axis_line_width` -> ancho de los ejes XYZ de la cámara
- `camera_frustum_depth` -> profundidad de los frustums de la cámara
- `camera_frustum_line_width` -> ancho de los frustums de la cámara
