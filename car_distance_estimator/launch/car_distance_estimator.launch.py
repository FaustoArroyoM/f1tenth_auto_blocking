# """Launch file for car distance estimator node."""

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition

# import os
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():
#     """Generate launch description."""
    
#     # Get package directory
#     package_dir = get_package_share_directory('car_distance_estimator')
#     config_dir = os.path.join(package_dir, 'config')

#     # Launch arguments - CORRECT SYNTAX: only name, defaults via parameters
#     # yolo_topic_arg = DeclareLaunchArgument('yolo_topic')
#     # depth_topic_arg = DeclareLaunchArgument('depth_topic')
#     # debug_arg = DeclareLaunchArgument('debug_mode')
#     # run_test_node_arg = DeclareLaunchArgument('run_test_node')

#     yolo_topic_arg = DeclareLaunchArgument('yolo_topic', default_value='/yolo/detections')
#     depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/zed2/depth/depth_registered')
#     debug_arg = DeclareLaunchArgument('debug_mode', default_value='false')
#     run_test_node_arg = DeclareLaunchArgument('run_test_node', default_value='false')

#     # Main node
#     car_distance_node = Node(
#         package='car_distance_estimator',
#         executable='car_distance_node',
#         name='car_distance_estimator',
#         output='screen',
#         parameters=[
#             os.path.join(config_dir, 'camera_params.yaml'),
#             {
#                 'yolo_topic': LaunchConfiguration('yolo_topic'),
#                 'depth_topic': LaunchConfiguration('depth_topic'),
#                 'debug_mode': LaunchConfiguration('debug_mode'),
#             }
#         ]
#     )

#     # Optional test node
#     test_node = Node(
#         package='car_distance_estimator',
#         executable='test_distance_pub',
#         name='distance_test_node',
#         output='screen',
#         condition=IfCondition(LaunchConfiguration('run_test_node'))
#     )

#     return LaunchDescription([
#         yolo_topic_arg,
#         depth_topic_arg,
#         debug_arg,
#         run_test_node_arg,
#         car_distance_node,
#         test_node,
#     ])


"""Launch file for race tracker."""

"""Launch file for race tracker node."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('car_distance_estimator')
    config_file = os.path.join(package_dir, 'config', 'race_tracker_params.yaml')

    race_tracker_node = Node(
        package='car_distance_estimator',
        executable='race_tracker_node',
        name='race_tracker',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/race_tracker/frame_annotated', '/camera/image_raw'),
        ]
    )

    return LaunchDescription([race_tracker_node])