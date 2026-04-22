"""
visualize_glb.launch.py - Launch GLB publisher + static TF + RViz2.

TF hierarchy:
  map (ROS REP-103: X-forward, Y-left, Z-up)  <- RViz Fixed Frame
   └── da3_world (OpenCV: X-right, Y-down, Z-forward)  <- DA3 GLB points live here

Usage:
  ros2 launch cirtesu_da3_mapping visualize_glb.launch.py glb_path:=/path/to/scene.glb
  ros2 launch cirtesu_da3_mapping visualize_glb.launch.py glb_path:=/path/to/scene.glb voxel_downsample:=0.02
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Quaternion (x, y, z, w) for R_cv2ros:
#   R = [[ 0, 0, 1],
#        [-1, 0, 0],
#        [ 0,-1, 0]]
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description():
    pkg_share = get_package_share_directory("cirtesu_da3_mapping")
    default_rviz = os.path.join(pkg_share, "rviz", "da3_mapping.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "glb_path", default_value="",
            description="Absolute path to DA3 .glb file",
        ),
        DeclareLaunchArgument(
            "frame_id", default_value="da3_world",
            description="Child frame of the static TF; PointCloud2 header frame",
        ),
        DeclareLaunchArgument(
            "world_frame", default_value="map",
            description="ROS world frame (REP-103, Z-up); RViz Fixed Frame",
        ),
        DeclareLaunchArgument(
            "topic", default_value="/cirtesu/map_pointcloud",
            description="ROS2 topic to publish PointCloud2",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="0.0",
            description="0.0 = one-shot transient_local; >0 = periodic",
        ),
        DeclareLaunchArgument(
            "voxel_downsample", default_value="0.0",
            description="Voxel size in meters (0.0 = disabled)",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="Path to RViz2 config file",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_da3_world",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", str(_QX), "--qy", str(_QY),
                "--qz", str(_QZ), "--qw", str(_QW),
                "--frame-id", LaunchConfiguration("world_frame"),
                "--child-frame-id", LaunchConfiguration("frame_id"),
            ],
        ),

        Node(
            package="cirtesu_da3_mapping",
            executable="glb_publisher_node.py",
            name="glb_publisher",
            output="screen",
            parameters=[{
                "glb_path": LaunchConfiguration("glb_path"),
                "frame_id": LaunchConfiguration("frame_id"),
                "topic": LaunchConfiguration("topic"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "voxel_downsample": LaunchConfiguration("voxel_downsample"),
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            output="screen",
        ),
    ])
