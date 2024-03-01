from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

import os

robot = 'smart_diffbot'
world = 'empty'

def generate_launch_description():
    
    ## Arguments
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Set to true to switch from hardware to simulation in the loop')
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value=robot,
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    
    ## Parameters
    controller_params = os.path.join(get_package_share_directory(robot+'_control'), 'config', 'controller_params.yaml')

    ## Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'urdf', 'robot.urdf.xacro')
    
    ## Nodes
    nodes = GroupAction(
        actions=[
            PushRosNamespace(condition=IfCondition(LaunchConfiguration('use_namespace')), 
                            namespace=LaunchConfiguration('robot_name')),

            ## Controller manager (only on real robot, Gazebo start one by default)
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[controller_params, 
                            {'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim:=False']), value_type=str)}
                            ],
                condition=UnlessCondition(LaunchConfiguration('sim')),
                emulate_tty=True,
            ),

            # Controllers spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "joint_state_broadcaster"],
            ),
        ]
    )
 
    ## Launch description
    return LaunchDescription([
        
        # Arguments
        sim_arg,
        robot_name_arg,
        namespace_arg,

        # Nodes
        nodes,

    ])
    
