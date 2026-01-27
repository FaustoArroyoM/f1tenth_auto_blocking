"""Launch file for race tracker."""

"""Launch file for race tracker node."""

# from car_distance_estimator.car_distance_estimator import raceline_manager_node
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('car_distance_estimator')
    config_file = os.path.join(package_dir, 'config', 'race_tracker_params.yaml')
    
    raceline_manager_node = Node(
        package='car_distance_estimator',
        executable='raceline_manager_node',
        name='raceline_manager',
        output='screen',
        parameters=[config_file]

    )

    return LaunchDescription([raceline_manager_node])