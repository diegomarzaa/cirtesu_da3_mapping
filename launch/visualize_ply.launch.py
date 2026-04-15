"""
visualize_ply.launch.py — Launch PLY publisher + camera pose visualization + static TF + RViz2.

TF hierarchy:
  map (ROS REP-103: X-forward, Y-left, Z-up)  ← RViz Fixed Frame
   └── da3_world (OpenCV: X-right, Y-down, Z-forward)  ← PLY lives here

A static_transform_publisher rotates da3_world into map so the scene shows
upright without modifying the points. The rotation is a 120° turn around
(-1, 1, -1)/√3, expressed as the quaternion below.

Usage:
  ros2 launch cirtesu_da3_mapping visualize_ply.launch.py
  ros2 launch cirtesu_da3_mapping visualize_ply.launch.py voxel_downsample:=0.02
  ros2 launch cirtesu_da3_mapping visualize_ply.launch.py ply_path:=/path/to/other.ply
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_PLY = (
    '/home/diego/Cirtesu/media/da3_outputs/diego_room_few/pcd/combined_pcd.ply'
)
_DEFAULT_POSES = (
    '/home/diego/Cirtesu/media/da3_outputs/diego_room_few/camera_poses.txt'
)
_DEFAULT_INTRINSICS = (
    '/home/diego/Cirtesu/media/da3_outputs/diego_room_few/intrinsic.txt'
)

# Quaternion (x, y, z, w) for R_cv2ros:
#   R = [[ 0, 0, 1],    X_ros =  Z_cv  (forward stays forward)
#        [-1, 0, 0],    Y_ros = -X_cv  (right becomes -left)
#        [ 0,-1, 0]]    Z_ros = -Y_cv  (down becomes -up, i.e. up)
_QX, _QY, _QZ, _QW = -0.5, 0.5, -0.5, 0.5


def generate_launch_description():
    pkg_share = get_package_share_directory('cirtesu_da3_mapping')
    default_rviz = os.path.join(pkg_share, 'rviz', 'ply_view.rviz')

    return LaunchDescription([
        # PLY publisher parameters
        DeclareLaunchArgument(
            'ply_path', default_value=_DEFAULT_PLY,
            description='Absolute path to .ply pointcloud file',
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='da3_world',
            description='Child frame of the static TF; PointCloud2 header frame',
        ),
        DeclareLaunchArgument(
            'world_frame', default_value='map',
            description='ROS world frame (REP-103, Z-up); RViz Fixed Frame',
        ),
        DeclareLaunchArgument(
            'topic', default_value='/cirtesu/map_pointcloud',
            description='ROS2 topic to publish PointCloud2',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='0.0',
            description='0.0 = one-shot transient_local; >0 = periodic',
        ),
        DeclareLaunchArgument(
            'voxel_downsample', default_value='0.0',
            description='Voxel size in meters (0.0 = disabled)',
        ),
        
        # Camera poses publisher parameters
        DeclareLaunchArgument(
            'camera_poses_path', default_value=_DEFAULT_POSES,
            description='Absolute path to camera_poses.txt',
        ),
        DeclareLaunchArgument(
            'intrinsics_path', default_value=_DEFAULT_INTRINSICS,
            description='Absolute path to intrinsic.txt',
        ),
        DeclareLaunchArgument(
            'camera_publish_rate_hz', default_value='0.0',
            description='0.0 = one-shot transient_local; >0 = periodic',
        ),
        DeclareLaunchArgument(
            'camera_axis_length', default_value='0.06',
            description='Length of camera axes markers in meters',
        ),
        DeclareLaunchArgument(
            'camera_axis_line_width', default_value='0.005',
            description='Line width for camera axes markers',
        ),
        DeclareLaunchArgument(
            'camera_frustum_depth', default_value='0.18',
            description='Depth of the frustum pyramid in camera coordinates',
        ),
        DeclareLaunchArgument(
            'camera_frustum_line_width', default_value='0.003',
            description='Line width for frustum markers',
        ),
        
        # RViz2 parameters
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='Path to RViz2 config file',
        ),

        # Static TF: rotates DA3/OpenCV world into ROS world so RViz shows
        # the scene with gravity pointing down.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_da3_world',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', str(_QX), '--qy', str(_QY),
                '--qz', str(_QZ), '--qw', str(_QW),
                '--frame-id', LaunchConfiguration('world_frame'),
                '--child-frame-id', LaunchConfiguration('frame_id'),
            ],
        ),

        Node(
            package='cirtesu_da3_mapping',
            executable='ply_publisher_node.py',
            name='ply_publisher',
            output='screen',
            parameters=[{
                'ply_path': LaunchConfiguration('ply_path'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic': LaunchConfiguration('topic'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'voxel_downsample': LaunchConfiguration('voxel_downsample'),
            }],
        ),

        Node(
            package='cirtesu_da3_mapping',
            executable='camera_poses_publisher_node.py',
            name='camera_poses_publisher',
            output='screen',
            parameters=[{
                'poses_path': LaunchConfiguration('camera_poses_path'),
                'intrinsics_path': LaunchConfiguration('intrinsics_path'),
                'frame_id': LaunchConfiguration('frame_id'),
                'path_topic': '/cirtesu/camera_path',
                'axes_topic': '/cirtesu/camera_axes',
                'frustums_topic': '/cirtesu/camera_frustums',
                'publish_rate_hz': LaunchConfiguration('camera_publish_rate_hz'),
                'axis_length': LaunchConfiguration('camera_axis_length'),
                'axis_line_width': LaunchConfiguration('camera_axis_line_width'),
                'frustum_depth': LaunchConfiguration('camera_frustum_depth'),
                'frustum_line_width': LaunchConfiguration('camera_frustum_line_width'),
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
