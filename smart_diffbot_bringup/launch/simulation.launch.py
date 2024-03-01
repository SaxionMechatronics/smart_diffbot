from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

import os

robot = 'smart_diffbot'


def generate_launch_description():
    
    ## Arguments
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value=robot,
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    camera_arg = DeclareLaunchArgument(name='camera', default_value='true', choices=['true', 'false'],
                                    description='Include camera in robot model')
    color_arg = DeclareLaunchArgument(name='color', default_value='SaxionGreen', 
                                    choices=['SaxionGreen', 'green', 'red', 'yellow', 'blue'],
                                    description='Color of the main body of the robot')
    x_arg = DeclareLaunchArgument(name='x', default_value='0', description='x-position')
    y_arg = DeclareLaunchArgument(name='y', default_value='0', description='y-position')
    
    ## Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'urdf', 'robot.urdf.xacro')

    ## Nodes without namespacing 
    nodes_without_namespacing = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_namespace')),
        actions=[
            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name="robot_state_publisher",
                parameters=[{
                    'robot_description': ParameterValue(Command(['xacro ', xacro_file, 
                                                                ' sim:=True',
                                                                ' robot_name:=', LaunchConfiguration('robot_name'), 
                                                                ' camera:=', LaunchConfiguration('camera'),
                                                                ' color:=' , LaunchConfiguration('color')]), value_type=str) 
                            }],
                emulate_tty=True,
            ),

            # Spawn Gazebo model
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', LaunchConfiguration('robot_name'), 
                        '-topic', '/robot_description', 
                        '-x', LaunchConfiguration('x'), 
                        '-y', LaunchConfiguration('y'), 
                        '-z', '1.0'],
                output='screen',
            ),

            # Bridge topics from Gazebo to ROS 2
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[['/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'],
                            ['/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
                            ['/navsat/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],
                            ['/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
                            ],
                output='screen',
            ),
        ]
    )

    ## Nodes with namespacing 
    nodes_with_namespacing = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_namespace')),
        actions=[
            PushRosNamespace(condition=IfCondition(LaunchConfiguration('use_namespace')), 
                            namespace=LaunchConfiguration('robot_name')),

            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name="robot_state_publisher",
                parameters=[{
                    'frame_prefix': [LaunchConfiguration('robot_name'), "/"],
                    'robot_description': ParameterValue(Command(['xacro ', xacro_file, 
                                                                ' sim:=True',
                                                                ' robot_name:=', LaunchConfiguration('robot_name'), 
                                                                ' namespace:=', LaunchConfiguration('robot_name'),
                                                                ' frame_prefix:=', LaunchConfiguration('robot_name'), "/",
                                                                ' camera:=', LaunchConfiguration('camera'),
                                                                ' color:=' , LaunchConfiguration('color')]), value_type=str) 
                            }],
                emulate_tty=True,
            ),

            # Spawn Gazebo model
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', LaunchConfiguration('robot_name'), 
                        '-topic', ['/',LaunchConfiguration('robot_name'),'/robot_description'], 
                        '-x', LaunchConfiguration('x'), 
                        '-y', LaunchConfiguration('y'), 
                        '-z', '1.0'],
                output='screen',
            ),

            # Bridge topics from Gazebo to ROS 2
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[['/',LaunchConfiguration('robot_name'),'/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'],
                            ['/',LaunchConfiguration('robot_name'),'/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
                            ['/',LaunchConfiguration('robot_name'),'/navsat/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],
                            ['/',LaunchConfiguration('robot_name'),'/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
                            ],
                output='screen',
            ),
        ]
    )


    ## Launch description
    return LaunchDescription([

        # Arguments
        robot_name_arg,
        namespace_arg,
        camera_arg,
        color_arg,
        x_arg,
        y_arg,

        # Nodes
        nodes_without_namespacing, # or
        nodes_with_namespacing

    ])
    
