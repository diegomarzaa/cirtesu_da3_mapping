"""Unified DA3 mapping launch — incremental and on-stop modes.

Parameter priority (highest → lowest):
  1. CLI launch arguments  (ros2 launch ... key:=value)
  2. DA3_PYTHON / DA3_SESSION_BASE environment variables
  3. params_file  (config/mapping_defaults.yaml by default)
  4. Node declare_parameter defaults

Example — use defaults from mapping_defaults.yaml:
    ros2 launch cirtesu_da3_mapping mapping.launch.py

Example — quick overrides without editing any file:
    ros2 launch cirtesu_da3_mapping mapping.launch.py \\
        image_topic:=/cam/image_raw/compressed \\
        use_sim_time:=true \\
        processing_mode:=on_stop

Example — point to a different params file:
    ros2 launch cirtesu_da3_mapping mapping.launch.py \\
        params_file:=/path/to/my_params.yaml
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

_WORLD_FRAME = "map"
_DA3_FRAME = "da3_world"
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("cirtesu_da3_mapping")
    default_params_file = os.path.join(pkg_share, "config", "mapping_defaults.yaml")
    default_rviz_config = os.path.join(pkg_share, "rviz", "da3_mapping.rviz")
    default_image_topic = "/image_raw/compressed"
    default_processing_mode = "incremental"

    da3_python      = LaunchConfiguration("da3_python")
    params_file     = LaunchConfiguration("params_file")
    use_sim_time    = LaunchConfiguration("use_sim_time")
    use_rviz        = LaunchConfiguration("rviz")
    rviz_config     = LaunchConfiguration("rviz_config")
    image_topic     = LaunchConfiguration("image_topic")
    processing_mode = LaunchConfiguration("processing_mode")
# voxel_downsample = LaunchConfiguration("voxel_downsample")
    # max_publish_points = LaunchConfiguration("max_publish_points")
    publish_accumulated = LaunchConfiguration("publish_accumulated")

    return LaunchDescription([
        # ------------------------------------------------------------------
        # da3_python: which interpreter runs the mapper.
        # Set DA3_PYTHON in your shell profile once and never pass it again.
        # Falls back to plain python3 for users without a dedicated venv.
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "da3_python",
            default_value=EnvironmentVariable("DA3_PYTHON", default_value="python3"),
            description=(
                "Python interpreter that has rclpy and DA3 installed. "
                "Override via the DA3_PYTHON environment variable or this argument."
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="ROS 2 params YAML file for the da3_mapper node.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock (bag playback) instead of wall time.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value=default_image_topic,
            description="Image topic for the mapper node.",
        ),
        DeclareLaunchArgument(
            "processing_mode",
            default_value=default_processing_mode,
            description="Processing mode for the mapper node: 'incremental' or 'on_stop'.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 alongside the mapper.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz configuration file.",
        ),
        DeclareLaunchArgument(
            "publish_accumulated",
            default_value="false",
            description="Publish accumulated cloud each chunk instead of only the new chunk.",
        ),

        LogInfo(msg=["=============== DA3 MAPPING LAUNCH ==============="]),
        LogInfo(msg=["[mapping] Python interpreter : ", da3_python]),
        LogInfo(msg=["[mapping] Use sim time       : ", use_sim_time]),
        LogInfo(msg=["[mapping] Params file        : ", params_file]),
        LogInfo(msg=["[mapping] Image topic        : ", image_topic]),
        LogInfo(msg=["[mapping] Processing mode    : ", processing_mode]),
        LogInfo(msg=["==================================================="]),

        # TF: map (REP-103) → da3_world (OpenCV convention)
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

        # Mapper node.
        # - params_file sets all node parameters (see mapping_defaults.yaml).
        # - The dict below overrides only what was explicitly passed on the CLI
        #   (launch args always have a value, so we always pass them; that is
        #   fine because their defaults match the YAML values).
        Node(
            package="cirtesu_da3_mapping",
            executable="mapping_node.py",
            name="da3_mapper",
            output="screen",
            prefix=[da3_python, " -u"],
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "image_topic": image_topic,
                    "processing_mode": processing_mode,
                    "publish_accumulated": publish_accumulated,
                },
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
