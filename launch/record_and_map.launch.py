"""
record_and_map.launch.py — frame_recorder node + static TF + RViz2.

Launches:
- static TF: map -> da3_world
- frame_recorder node
- RViz2

External dependencies
---------------------
This package expects the Depth-Anything-3 repository and Python environment
to be provided explicitly, either through launch arguments or environment vars.

Example:
    ros2 launch cirtesu_da3_mapping record_and_map.launch.py

Or override explicitly:
    ros2 launch cirtesu_da3_mapping record_and_map.launch.py \
        da3_python:=/opt/venvs/da3/bin/python3 \
        depth_anything_3_dir:=/workspace/Depth-Anything-3 \
        da3_config:=/workspace/Depth-Anything-3/da3_streaming/configs/default.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

_IMAGE_TOPIC = "/camera/image_raw"
_POINTCLOUD_TOPIC = "/cirtesu/map_pointcloud"
_WORLD_FRAME = "map"
_DA3_FRAME = "da3_world"

# Quaternion (x, y, z, w) that rotates da3_world (OpenCV) into map (ROS REP-103):
#   R = [[ 0, 0, 1],    X_ros =  Z_cv
#        [-1, 0, 0],    Y_ros = -X_cv
#        [ 0,-1, 0]]    Z_ros = -Y_cv  (Y-down → Z-up)
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description():
    pkg_share = get_package_share_directory("cirtesu_da3_mapping")
    default_rviz_cfg = os.path.join(pkg_share, "rviz", "da3_mapping.rviz")
    default_session_base = os.path.join(str(os.path.expanduser("~")), "da3_sessions")

    da3_python = LaunchConfiguration("da3_python")
    depth_anything_3_dir = LaunchConfiguration("depth_anything_3_dir")
    da3_streaming_dir = LaunchConfiguration("da3_streaming_dir")
    da3_script = LaunchConfiguration("da3_script")
    da3_config = LaunchConfiguration("da3_config")

    image_topic = LaunchConfiguration("image_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    session_base_dir = LaunchConfiguration("session_base_dir")
    target_save_fps = LaunchConfiguration("target_save_fps")
    voxel_downsample = LaunchConfiguration("voxel_downsample")
    frame_id = LaunchConfiguration("frame_id")
    world_frame = LaunchConfiguration("world_frame")
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
            "da3_script",
            default_value=PathJoinSubstitution([da3_streaming_dir, "da3_streaming.py"]),
            description="Path to the DA3 streaming entrypoint script.",
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
            description="Camera image topic to subscribe to for recording.",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value=_POINTCLOUD_TOPIC,
            description="Output topic for accumulated pointcloud.",
        ),
        DeclareLaunchArgument(
            "session_base_dir",
            default_value=EnvironmentVariable("DA3_SESSION_BASE", default_value=default_session_base),
            description="Base directory where session folders are created.",
        ),
        DeclareLaunchArgument(
            "target_save_fps",
            default_value="1.0",
            description="Target frame save rate in FPS (<= 0 means save every frame).",
        ),
        DeclareLaunchArgument(
            "voxel_downsample",
            default_value="0.0",
            description="Voxel downsample size in meters when loading the PLY (0=off).",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value=_DA3_FRAME,
            description="Frame ID for DA3 outputs (child frame of the static TF).",
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value=_WORLD_FRAME,
            description="ROS world frame (REP-103 Z-up); Fixed Frame in RViz.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock instead of wall time.",
        ),

        LogInfo(msg=["=============== LAUNCH ARGUMENTS ==============="]),
        LogInfo(msg=["[record_and_map] DA3 python:      ", da3_python]),
        LogInfo(msg=["[record_and_map] DA3 repo:        ", depth_anything_3_dir]),
        LogInfo(msg=["[record_and_map] DA3 script:      ", da3_script]),
        LogInfo(msg=["[record_and_map] DA3 config:      ", da3_config]),
        LogInfo(msg=["[record_and_map] RViz config:     ", rviz_config]),
        LogInfo(msg=["[record_and_map] Session base:    ", session_base_dir]),
        LogInfo(msg=["[record_and_map] Image topic:     ", image_topic]),
        LogInfo(msg=["[record_and_map] Pointcloud topic:", pointcloud_topic]),
        LogInfo(msg=["[record_and_map] Frame id:        ", frame_id]),
        LogInfo(msg=["[record_and_map] World frame:     ", world_frame]),
        LogInfo(msg=["[record_and_map] Target save FPS: ", target_save_fps]),
        LogInfo(msg=["[record_and_map] Voxel downsample:", voxel_downsample]),
        LogInfo(msg=["[record_and_map] Use sim time:    ", use_sim_time]),
        LogInfo(msg=["================================================"]),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_da3_world",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", str(_QX), "--qy", str(_QY),
                "--qz", str(_QZ), "--qw", str(_QW),
                "--frame-id", world_frame,
                "--child-frame-id", frame_id,
            ],
        ),

        Node(
            package="cirtesu_da3_mapping",
            executable="frame_recorder_node.py",
            name="frame_recorder",
            output="screen",
            prefix=[da3_python, " -u"],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "image_topic": image_topic,
                    "session_base_dir": session_base_dir,
                    "da3_python": da3_python,
                    "da3_script": da3_script,
                    "da3_config": da3_config,
                    "target_save_fps": target_save_fps,
                    "voxel_downsample": voxel_downsample,
                    "pointcloud_topic": pointcloud_topic,
                    "frame_id": frame_id,
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
