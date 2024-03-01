from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString

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
        
    ## Parameters (replace frame names in case of namespacing)
    localization_params = ReplaceString(condition=IfCondition(LaunchConfiguration('use_namespace')),
                                        source_file=os.path.join(get_package_share_directory(robot+'_localization'), 'config', 'localization_params.yaml'), 
                                        replacements={"base_link_frame: base_link": ["base_link_frame: ", LaunchConfiguration('robot_name'), "/base_link"],
                                                      "odom_frame: odom": ["odom_frame: ", LaunchConfiguration('robot_name'), "/odom"],})
    
    ## Nodes
    nodes = GroupAction(
        actions=[
            PushRosNamespace(condition=IfCondition(LaunchConfiguration('use_namespace')), 
                            namespace=LaunchConfiguration('robot_name')),

            # Navsat tranform
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            localization_params],        
                remappings=[('gps/fix', 'navsat/fix'),
                            ('/imu', 'imu/data'),
                            ('odometry/filtered', 'global_ekf/odometry_filtered'),
                            ('odometry/gps', 'navsat_transform/navsat_odometry'),
                            ('gps/filtered', 'navsat_transform/filtered_fix')
                            ]
            ),

            # Global EKF for odom => map tranform 
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='global_ekf_node',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            localization_params],   
                remappings=[('odometry/filtered', 'global_ekf/odometry_filtered')]
            ),

            # Marker detection
            Node(
                package='ros2_aruco',
                executable='aruco_node',
                output='screen',
                name='marker_detection_node',
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            localization_params],
                remappings=[('/camera/camera_info', 'camera/camera_info'),
                            ('/camera/image_raw', 'camera/image_raw'),
                            ]
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
    
