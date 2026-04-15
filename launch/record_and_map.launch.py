"""
record_and_map.launch.py — Launch the frame_recorder node + static TF + RViz2.

TF hierarchy (same as visualize_ply):
  map (ROS REP-103: X-forward, Y-left, Z-up)   ← RViz Fixed Frame
   └── da3_world (OpenCV: X-right, Y-down, Z-forward)  ← DA3 outputs live here

Workflow
--------
  # 1. Launch everything
  ros2 launch cirtesu_da3_mapping record_and_map.launch.py

  # 2. Start a recording session (in another terminal, inside the container)
  ros2 service call /frame_recorder/start_recording std_srvs/srv/Trigger {}

  # 3. Move the robot / camera while frames are saved.
  #    Watch frame count:
  ros2 topic echo /frame_recorder/status

  # 4. Stop recording and trigger DA3-Streaming
  ros2 service call /frame_recorder/stop_and_process std_srvs/srv/Trigger {}

  # 5. The node streams DA3 output to the log and publishes the result
  #    automatically in RViz when processing finishes.

Parameters
----------
  image_topic         Camera topic to subscribe to (default: /camera/image_raw)
  target_save_fps     Target frame save rate in FPS (default: 1.0)
  da3_config          Path to DA3-Streaming config YAML
  session_base_dir    Base directory where session folders are created
  voxel_downsample    Voxel size in meters when loading the final PLY (0 = off)
  frame_id            DA3 output frame (child of world TF, default: da3_world)
  world_frame         ROS world frame (parent of da3_world, default: map)
  rviz_config         Path to RViz2 config file
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DA3_PYTHON = '/opt/venvs/da3/bin/python3'
_DA3_SCRIPT = (
    '/home/diego/Cirtesu/Repositories/Depth-Anything-3/'
    'da3_streaming/da3_streaming.py'
)
_DA3_CONFIG = (
    '/home/diego/Cirtesu/Repositories/Depth-Anything-3/'
    'da3_streaming/configs/diego_config.yaml'
)
_SESSION_BASE = '/home/diego/Cirtesu/media/da3_sessions'

# Quaternion (x, y, z, w) that rotates da3_world (OpenCV) into map (ROS REP-103):
#   R = [[ 0, 0, 1],    X_ros =  Z_cv
#        [-1, 0, 0],    Y_ros = -X_cv
#        [ 0,-1, 0]]    Z_ros = -Y_cv  (Y-down → Z-up)
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description():
    pkg_share   = get_package_share_directory('cirtesu_da3_mapping')
    default_rviz = os.path.join(pkg_share, 'rviz', 'ply_view.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image_raw',
            description='Camera image topic to subscribe to for recording',
        ),
        DeclareLaunchArgument(
            'target_save_fps', default_value='1.0',
            description='Target frame save rate in FPS (<= 0 means save every frame)',
        ),
        DeclareLaunchArgument(
            'da3_config', default_value=_DA3_CONFIG,
            description='Path to DA3-Streaming config YAML',
        ),
        DeclareLaunchArgument(
            'session_base_dir', default_value=_SESSION_BASE,
            description='Base directory where session folders are created',
        ),
        DeclareLaunchArgument(
            'voxel_downsample', default_value='0.0',
            description='Voxel downsample size in meters when loading the PLY (0=off)',
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='da3_world',
            description='Frame ID for DA3 outputs (child frame of the static TF)',
        ),
        DeclareLaunchArgument(
            'world_frame', default_value='map',
            description='ROS world frame (REP-103 Z-up); Fixed Frame in RViz',
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='Path to RViz2 config file',
        ),

        # Static TF: rotates DA3/OpenCV world into ROS REP-103 world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_da3_world',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', str(_QX), '--qy', str(_QY),
                '--qz', str(_QZ), '--qw', str(_QW),
                '--frame-id',       LaunchConfiguration('world_frame'),
                '--child-frame-id', LaunchConfiguration('frame_id'),
            ],
        ),

        Node(
            package='cirtesu_da3_mapping',
            executable='frame_recorder_node.py',
            name='frame_recorder',
            output='screen',
            parameters=[{
                'image_topic':         LaunchConfiguration('image_topic'),
                'session_base_dir':    LaunchConfiguration('session_base_dir'),
                'da3_python':          _DA3_PYTHON,
                'da3_script':          _DA3_SCRIPT,
                'da3_config':          LaunchConfiguration('da3_config'),
                'target_save_fps':     LaunchConfiguration('target_save_fps'),
                'voxel_downsample':    LaunchConfiguration('voxel_downsample'),
                'frame_id':            LaunchConfiguration('frame_id'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
