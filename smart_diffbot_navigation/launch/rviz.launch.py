import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = 'smart_diffbot'


def generate_launch_description():

    ## Nodes
    # Rviz
    rviz_config = os.path.join(get_package_share_directory(robot+'_navigation'), 'launch', 'rviz', 'monitor_navigation.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    ## Launch description
    return LaunchDescription([

        rviz
    ])
    
