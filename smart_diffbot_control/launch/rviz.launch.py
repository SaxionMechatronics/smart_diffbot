import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot = 'smart_diffbot'

    ## Nodes
    # Rviz
    rviz_config = os.path.join(get_package_share_directory(robot+'_control'), 'launch', 'rviz', 'monitor_control.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ## Launch description
    return LaunchDescription([
        rviz
    ])
    
