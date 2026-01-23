import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to existing launch files
    pf_pkg = get_package_share_directory('particle_filter')
    pp_pkg = get_package_share_directory('pure_pursuit')
    cde_pkg = get_package_share_directory('car_distance_estimator')

    
    # 2. Define the 'Includes' 
    # Particle Filter Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pf_pkg, 'launch', 'localize_launch.py'))
    )

    # Pure Pursuit Controller 
    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pp_pkg, 'launch', 'pure_pursuit_multi_launch.py'))
    )

    # Car Distance Estimator
    car_distance_estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cde_pkg, 'launch', 'car_distance_estimator.launch.py'))
    )

    # Raceline manager 
    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cde_pkg, 'launch', 'raceline_manager.launch.py'))
    )


    # 4. Assemble the Launch Description
    ld = LaunchDescription()
    
    # Add actions in order
    ld.add_action(localization_launch)
    ld.add_action(pure_pursuit_launch)
    ld.add_action(car_distance_estimator_launch)
    ld.add_action(manager_launch)

    return ld