"""Batch-incremental DA3 mapping launch.

This launch captures incoming images continuously and periodically rebuilds one
global DA3 point cloud from all frames seen so far. If the global DA3 command
runs out of GPU memory, the node falls back to DA3-Streaming for that update.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_WORLD_FRAME = "map"
_DA3_FRAME = "da3_world"
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description():
    pkg_share = get_package_share_directory("cirtesu_da3_mapping")
    default_rviz_config = os.path.join(pkg_share, "rviz", "da3_mapping.rviz")

    auto_start = LaunchConfiguration("auto_start")
    image_topic = LaunchConfiguration("image_topic")
    session_base_dir = LaunchConfiguration("session_base_dir")
    da3_root_dir = LaunchConfiguration("da3_root_dir")
    da3_cli = LaunchConfiguration("da3_cli")
    da3_model_dir = LaunchConfiguration("da3_model_dir")
    da3_streaming_dir = LaunchConfiguration("da3_streaming_dir")
    streaming_config = LaunchConfiguration("streaming_config")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    target_save_fps = LaunchConfiguration("target_save_fps")
    min_images_initial = LaunchConfiguration("min_images_initial")
    min_new_images = LaunchConfiguration("min_new_images")
    min_seconds_between_runs = LaunchConfiguration("min_seconds_between_runs")
    max_normal_images = LaunchConfiguration("max_normal_images")
    process_res = LaunchConfiguration("process_res")
    num_max_points = LaunchConfiguration("num_max_points")
    conf_thresh_percentile = LaunchConfiguration("conf_thresh_percentile")
    voxel_downsample = LaunchConfiguration("voxel_downsample")
    fallback_to_streaming = LaunchConfiguration("fallback_to_streaming")
    save_depth_outputs = LaunchConfiguration("save_depth_outputs")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("image_topic", default_value="/image_raw/compressed"),
        DeclareLaunchArgument(
            "session_base_dir",
            default_value="/home/usuario/DockerWorkspace/tmp/da3_simultaneous",
        ),
        DeclareLaunchArgument(
            "da3_root_dir",
            default_value="/home/usuario/DockerWorkspace/src/Depth-Anything-3",
        ),
        DeclareLaunchArgument("da3_cli", default_value="da3"),
        DeclareLaunchArgument(
            "da3_model_dir",
            default_value=(
                "/home/usuario/DockerWorkspace/src/Depth-Anything-3/"
                "da3_streaming/weights/DA3-LARGE-1.1"
            ),
        ),
        DeclareLaunchArgument(
            "da3_streaming_dir",
            default_value=(
                "/home/usuario/DockerWorkspace/src/Depth-Anything-3/da3_streaming"
            ),
        ),
        DeclareLaunchArgument(
            "streaming_config",
            default_value=(
                "/home/usuario/DockerWorkspace/src/Depth-Anything-3/"
                "da3_streaming/configs/rtx4070_large_balanced.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/cirtesu/map_pointcloud",
        ),
        DeclareLaunchArgument("target_save_fps", default_value="30.0"),
        DeclareLaunchArgument("min_images_initial", default_value="8"),
        DeclareLaunchArgument("min_new_images", default_value="8"),
        DeclareLaunchArgument("min_seconds_between_runs", default_value="30.0"),
        DeclareLaunchArgument("max_normal_images", default_value="80"),
        DeclareLaunchArgument("process_res", default_value="504"),
        DeclareLaunchArgument("num_max_points", default_value="4000000"),
        DeclareLaunchArgument("conf_thresh_percentile", default_value="10.0"),
        DeclareLaunchArgument("voxel_downsample", default_value="0.0"),
        DeclareLaunchArgument("fallback_to_streaming", default_value="true"),
        DeclareLaunchArgument("save_depth_outputs", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),

        LogInfo(msg=["=========== DA3 MAPPING SIMULTANEOUS ==========="]),
        LogInfo(msg=["[simultaneous] Image topic      : ", image_topic]),
        LogInfo(msg=["[simultaneous] DA3 model        : ", da3_model_dir]),
        LogInfo(msg=["[simultaneous] Streaming config : ", streaming_config]),
        LogInfo(msg=["[simultaneous] Normal max imgs  : ", max_normal_images]),
        LogInfo(msg=["[simultaneous] Batch trigger    : ", min_new_images, " new images"]),
        LogInfo(msg=["================================================="]),

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
            executable="mapping_simultaneous_node.py",
            name="mapping_simultaneous",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "auto_start": auto_start,
                "image_topic": image_topic,
                "session_base_dir": session_base_dir,
                "da3_root_dir": da3_root_dir,
                "da3_cli": da3_cli,
                "da3_model_dir": da3_model_dir,
                "da3_streaming_dir": da3_streaming_dir,
                "streaming_config": streaming_config,
                "pointcloud_topic": pointcloud_topic,
                "frame_id": _DA3_FRAME,
                "target_save_fps": target_save_fps,
                "min_images_initial": min_images_initial,
                "min_new_images": min_new_images,
                "min_seconds_between_runs": min_seconds_between_runs,
                "max_normal_images": max_normal_images,
                "process_res": process_res,
                "num_max_points": num_max_points,
                "conf_thresh_percentile": conf_thresh_percentile,
                "voxel_downsample": voxel_downsample,
                "fallback_to_streaming": fallback_to_streaming,
                "save_depth_outputs": save_depth_outputs,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
