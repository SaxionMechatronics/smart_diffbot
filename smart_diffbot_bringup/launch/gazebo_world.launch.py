from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

robot = 'smart_diffbot'

def generate_launch_description():
    
    ## Arguments
    world_arg = DeclareLaunchArgument(name='world', default_value='empty_world', description='Gazebo world name')

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', [' -r -v 0 ', get_package_share_directory(robot+'_bringup'), '/worlds/', LaunchConfiguration('world'), '.sdf'])],
    )

    # Bridge clock 
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )


    ## Launch description
    return LaunchDescription([

        # Arguments
        world_arg,

        # Launch
        gazebo,

        # Nodes
        gz_bridge_node,

    ])
    
