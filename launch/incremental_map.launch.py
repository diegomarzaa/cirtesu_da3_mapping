"""
incremental_map.launch.py — incremental mapper node + static TF + RViz2.

Launches:
- static TF: map -> da3_world
- incremental mapper node
- RViz2

External dependencies
---------------------
This package expects the Depth-Anything-3 repository and Python environment
to be provided explicitly, either through launch arguments or environment vars.

Example:
    ros2 launch cirtesu_da3_mapping incremental_map.launch.py

Or override explicitly:
    ros2 launch cirtesu_da3_mapping incremental_map.launch.py \
        da3_python:=/opt/venvs/da3/bin/python3 \
        depth_anything_3_dir:=/workspace/Depth-Anything-3 \
        da3_config:=/workspace/Depth-Anything-3/da3_streaming/configs/default.yaml
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

_IMAGE_TOPIC = "/image_raw/compressed"
_POINTCLOUD_TOPIC = "/cirtesu/map_pointcloud"
_PATH_TOPIC = "/cirtesu/camera_path"

_WORLD_FRAME = "map"
_DA3_FRAME = "da3_world"

# Static rotation that maps da3_world (OpenCV) → map (REP-103):
#   R = [[ 0, 0, 1], [-1, 0, 0], [ 0,-1, 0]]
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description():
    pkg_share = get_package_share_directory("cirtesu_da3_mapping")

    default_rviz_cfg = os.path.join(pkg_share, "rviz", "da3_mapping.rviz")
    default_session_base = os.path.join(str(Path.home()), "da3_sessions")

    da3_python = LaunchConfiguration("da3_python")
    depth_anything_3_dir = LaunchConfiguration("depth_anything_3_dir")
    da3_streaming_dir = LaunchConfiguration("da3_streaming_dir")
    da3_src_dir = LaunchConfiguration("da3_src_dir")
    da3_config = LaunchConfiguration("da3_config")

    image_topic = LaunchConfiguration("image_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    path_topic = LaunchConfiguration("path_topic")
    frame_id = LaunchConfiguration("frame_id")
    session_base_dir = LaunchConfiguration("session_base_dir")
    target_save_fps = LaunchConfiguration("target_save_fps")
    debug_save = LaunchConfiguration("debug_save")
    voxel_downsample = LaunchConfiguration("voxel_downsample")
    publish_accumulated = LaunchConfiguration("publish_accumulated")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "da3_python",
            default_value=EnvironmentVariable("DA3_PYTHON", default_value="/usr/bin/python3"),
            description="Python interpreter with both rclpy and DA3 dependencies installed.",
        ),
        DeclareLaunchArgument(
            "depth_anything_3_dir",
            default_value=EnvironmentVariable("DEPTH_ANYTHING_3_DIR", default_value=""),
            description="Absolute path to the external Depth-Anything-3 repository.",
        ),
        DeclareLaunchArgument(
            "da3_streaming_dir",
            default_value=PathJoinSubstitution([depth_anything_3_dir, "da3_streaming"]),
            description="Path to the DA3 streaming directory.",
        ),
        DeclareLaunchArgument(
            "da3_src_dir",
            default_value=PathJoinSubstitution([depth_anything_3_dir, "src"]),
            description="Path to the Depth-Anything-3 src directory.",
        ),
        DeclareLaunchArgument(
            "da3_config",
            default_value=EnvironmentVariable(
                "DA3_CONFIG",
                default_value="",
            ),
            description="DA3 streaming YAML config.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_cfg,
            description="RViz configuration file.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value=_IMAGE_TOPIC,
            description="Compressed image topic consumed by the incremental mapper.",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value=_POINTCLOUD_TOPIC,
            description="Output topic for accumulated pointcloud.",
        ),
        DeclareLaunchArgument(
            "path_topic",
            default_value=_PATH_TOPIC,
            description="Output topic for accumulated camera path.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value=_DA3_FRAME,
            description="Frame id used in pointcloud and path headers.",
        ),
        DeclareLaunchArgument(
            "session_base_dir",
            default_value=EnvironmentVariable("DA3_SESSION_BASE", default_value=default_session_base),
            description="Base directory where session outputs are stored.",
        ),
        DeclareLaunchArgument(
            "target_save_fps",
            default_value="1.0",
            description="Frame save rate. Use 0 to save every incoming frame.",
        ),
        DeclareLaunchArgument(
            "debug_save",
            default_value="false",
            description="Save per-chunk debug outputs on disk.",
        ),
        DeclareLaunchArgument(
            "voxel_downsample",
            default_value="0.01",
            description="Voxel size for downsampling before publishing.",
        ),
        DeclareLaunchArgument(
            "publish_accumulated",
            default_value="true",
            description="Publish the full accumulated map each update. Set false to publish only the new chunk.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock instead of wall time.",
        ),

        LogInfo(msg=["=============== LAUNCH ARGUMENTS ==============="]),
        LogInfo(msg=["[incremental_map] DA3 python: ", da3_python]),
        LogInfo(msg=["[incremental_map] DA3 repo: ", depth_anything_3_dir]),
        LogInfo(msg=["[incremental_map] DA3 streaming dir: ", da3_streaming_dir]),
        LogInfo(msg=["[incremental_map] DA3 src dir: ", da3_src_dir]),
        LogInfo(msg=["[incremental_map] DA3 config: ", da3_config]),
        LogInfo(msg=["[incremental_map] Session base: ", session_base_dir]),
        LogInfo(msg=["[incremental_map] Image topic: ", image_topic]),
        LogInfo(msg=["[incremental_map] Pointcloud topic: ", pointcloud_topic]),
        LogInfo(msg=["[incremental_map] Path topic: ", path_topic]),
        LogInfo(msg=["[incremental_map] Frame id: ", frame_id]),
        LogInfo(msg=["[incremental_map] Target save FPS: ", target_save_fps]),
        LogInfo(msg=["[incremental_map] Debug save: ", debug_save]),
        LogInfo(msg=["[incremental_map] Voxel downsample: ", voxel_downsample]),
        LogInfo(msg=["[incremental_map] Publish accumulated: ", publish_accumulated]),
        LogInfo(msg=["[incremental_map] Use sim time: ", use_sim_time]),
        LogInfo(msg=["================================================"]),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_da3_world",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", str(_QX), "--qy", str(_QY),
                "--qz", str(_QZ), "--qw", str(_QW),
                "--frame-id", _WORLD_FRAME,
                "--child-frame-id", _DA3_FRAME,
            ],
        ),

        Node(
            package="cirtesu_da3_mapping",
            executable="incremental_mapper_node.py",
            name="incremental_mapper",
            output="screen",
            prefix=[da3_python, " -u"],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "image_topic": image_topic,
                    "session_base_dir": session_base_dir,
                    "da3_streaming_dir": da3_streaming_dir,
                    "da3_src_dir": da3_src_dir,
                    "da3_config": da3_config,
                    "pointcloud_topic": pointcloud_topic,
                    "path_topic": path_topic,
                    "frame_id": frame_id,
                    "target_save_fps": target_save_fps,
                    "debug_save": debug_save,
                    "voxel_downsample": voxel_downsample,
                    "publish_accumulated": publish_accumulated,
                },
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
