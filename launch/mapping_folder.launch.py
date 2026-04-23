"""Launch DA3 folder reconstruction and publish the resulting point cloud."""

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

    image_dir = LaunchConfiguration("image_dir")
    output_dir = LaunchConfiguration("output_dir")
    process_on_start = LaunchConfiguration("process_on_start")
    force_streaming = LaunchConfiguration("force_streaming")
    da3_root_dir = LaunchConfiguration("da3_root_dir")
    da3_cli = LaunchConfiguration("da3_cli")
    da3_model_dir = LaunchConfiguration("da3_model_dir")
    da3_streaming_dir = LaunchConfiguration("da3_streaming_dir")
    streaming_config = LaunchConfiguration("streaming_config")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
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
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "image_dir",
            default_value="/home/usuario/DockerWorkspace/src/media/net_parche_imgs",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/home/usuario/DockerWorkspace/tmp/da3_folder_mapping",
        ),
        DeclareLaunchArgument("process_on_start", default_value="true"),
        DeclareLaunchArgument("force_streaming", default_value="false"),
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
            default_value="/home/usuario/DockerWorkspace/src/Depth-Anything-3/da3_streaming",
        ),
        DeclareLaunchArgument(
            "streaming_config",
            default_value=(
                "/home/usuario/DockerWorkspace/src/Depth-Anything-3/"
                "da3_streaming/configs/rtx4070_large_balanced.yaml"
            ),
        ),
        DeclareLaunchArgument("pointcloud_topic", default_value="/cirtesu/map_pointcloud"),
        DeclareLaunchArgument("max_normal_images", default_value="80"),
        DeclareLaunchArgument("process_res", default_value="504"),
        DeclareLaunchArgument("num_max_points", default_value="4000000"),
        DeclareLaunchArgument("conf_thresh_percentile", default_value="10.0"),
        DeclareLaunchArgument("voxel_downsample", default_value="0.0"),
        DeclareLaunchArgument("fallback_to_streaming", default_value="true"),
        DeclareLaunchArgument("save_depth_outputs", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),

        LogInfo(msg=["=============== DA3 MAPPING FOLDER ==============="]),
        LogInfo(msg=["[folder] Image dir        : ", image_dir]),
        LogInfo(msg=["[folder] Output dir       : ", output_dir]),
        LogInfo(msg=["[folder] DA3 model        : ", da3_model_dir]),
        LogInfo(msg=["[folder] Streaming config : ", streaming_config]),
        LogInfo(msg=["[folder] Normal max imgs  : ", max_normal_images]),
        LogInfo(msg=["==================================================="]),

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
            executable="mapping_folder_node.py",
            name="mapping_folder",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "image_dir": image_dir,
                "output_dir": output_dir,
                "process_on_start": process_on_start,
                "force_streaming": force_streaming,
                "da3_root_dir": da3_root_dir,
                "da3_cli": da3_cli,
                "da3_model_dir": da3_model_dir,
                "da3_streaming_dir": da3_streaming_dir,
                "streaming_config": streaming_config,
                "pointcloud_topic": pointcloud_topic,
                "frame_id": _DA3_FRAME,
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
