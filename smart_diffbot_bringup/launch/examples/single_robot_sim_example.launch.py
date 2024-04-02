from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

robot = 'smart_diffbot'

# Example launch file for a Gazebo world with docking station and one robot (not namespaced)

def generate_launch_description():
        
    # Launch docking world
    launch_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'gazebo_world.launch.py')]),
            launch_arguments=[('world', 'test_world')]
    )

    # Launch robot
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'main.launch.py')]),
            launch_arguments=[('costmap_path', os.path.join(get_package_share_directory(robot+'_bringup'), 
                                                            'worlds', 'costmaps', 'docking_world.yaml'))]
    )

    ## Launch description
    return LaunchDescription([

        # Launch
        launch_empty_world,
        launch_robot,

    ])
    
