from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory

import os

robot = 'smart_diffbot'

def generate_launch_description():
    
    ## Arguments
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Set to true to switch from hardware to simulation in the loop')
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value=robot,
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    costmap_arg = DeclareLaunchArgument(name='costmap_path', 
                                        default_value=os.path.join(get_package_share_directory(robot+'_bringup'),
                                                                   'worlds', 'costmaps', 'empty_world.yaml'),
                                        description='Path to global cosmtap .yaml file')
    
    ## Simulation specific arguments
    camera_arg = DeclareLaunchArgument(name='camera', default_value='true', choices=['true', 'false'],
                                    description='[Simulation only] Include camera in robot model')
    color_arg = DeclareLaunchArgument(name='color', default_value='SaxionGreen', 
                                    choices=['SaxionGreen', 'green', 'red', 'yellow', 'blue'],
                                    description='[Simulation only] Color of the main body of the robot')
    x_arg = DeclareLaunchArgument(name='x', default_value='0', description='[Simulation only] x-position')
    y_arg = DeclareLaunchArgument(name='y', default_value='0', description='[Simulation only] y-position')
    

    ## Launch group
    launch_group = GroupAction([

        # Launch hardware 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'hardware.launch.py')]),
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),
        # Or simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'simulation.launch.py')]),
            condition=IfCondition(LaunchConfiguration('sim')),
        ),

        # Launch control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_control'), 'launch', 'control.launch.py')]),
        ),

        # Launch localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_localization'), 'launch', 'localization.launch.py')]),
        ),

        # Launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_navigation'), 'launch', 'navigation.launch.py')]),
        )
    ])



    ## Launch description
    return LaunchDescription([

        # Arguments
        sim_arg,
        robot_name_arg,
        namespace_arg,
        costmap_arg,
        camera_arg,
        color_arg,
        x_arg,
        y_arg,

        # Launch
        launch_group
    ])
    
