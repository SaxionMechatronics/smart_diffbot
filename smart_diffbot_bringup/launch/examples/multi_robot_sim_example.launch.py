from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

robot = 'smart_diffbot'

# Example launch file for an empty Gazebo world with two robots

def generate_launch_description():
        
    # Launch empty
    launch_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'gazebo_world.launch.py')]),
            launch_arguments=[('world', 'empty_world')]
    )


    # Launch first robot
    launch_robot_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'main.launch.py')]),
        launch_arguments=[  ('robot_name', 'diffbot_1'),
                            ('use_namespace', 'true'),
                            ('color', 'red'),
                            ('camera', 'false'),
                            ('x', '5'),
                            ('y', '2'),
                        ])

    # Launch second robot
    launch_robot_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'main.launch.py')]),
        launch_arguments=[  ('robot_name', 'diffbot_2'),
                            ('use_namespace', 'true'),
                            ('color', 'yellow'),
                            ('camera', 'false'),
                            ('x', '10'),
                            ('y', '-2'),
                        ])


    ## Launch description
    return LaunchDescription([

        # Launch
        launch_empty_world,
        launch_robot_1,
        launch_robot_2,

    ])
    