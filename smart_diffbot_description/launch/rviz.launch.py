import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = 'smart_diffbot'

def generate_launch_description():
    
    ## Arguments
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value=robot,
                                    description='Unique robot name')
    camera_arg = DeclareLaunchArgument(name='camera', default_value='true', choices=['true', 'false'],
                                    description='Include camera in robot model')

    ## Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'urdf', 'robot.urdf.xacro')


    ## Nodes

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        name="robot_state_publisher",
        parameters=[{
            'frame_prefix': [LaunchConfiguration('robot_name'), "/"],
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, 
                                                         ' sim:=True robot_name:=', LaunchConfiguration('robot_name'), 
                                                         ' camera:=', LaunchConfiguration('camera')]), value_type=str) }],
        emulate_tty=True
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=LaunchConfiguration('robot_name'),
    )

    # Rviz
    rviz_config = os.path.join(get_package_share_directory(robot+'_description'), 'launch', 'rviz', 'view_model.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ## Launch description
    return LaunchDescription([

        # Arguments
        robot_name_arg,
        camera_arg,

        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
    
