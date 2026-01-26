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

    race_tracker_node = Node(
        package='car_distance_estimator',
        executable='race_tracker_node',
        name='race_tracker',
        output='screen',
        parameters=[config_file],
        # remappings=[
        #     ('/race_tracker/frame_annotated', '/camera/image_raw'),
        # ]
    )

    return LaunchDescription([race_tracker_node])