from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
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
    costmap_arg = DeclareLaunchArgument(name='costmap_path', 
                                        default_value=os.path.join(get_package_share_directory(robot+'_bringup'),
                                                                   'worlds', 'costmaps', 'empty_world.yaml'),
                                        description='Path to global cosmtap .yaml file')
    
    ## Parameters
    navigation_params = ReplaceString(condition=IfCondition(LaunchConfiguration('use_namespace')),
                                        source_file=os.path.join(get_package_share_directory(robot+'_navigation'), 'config', 'navigation_params.yaml'), 
                                        replacements={"robot_base_frame: base_link": ["robot_base_frame: ", LaunchConfiguration('robot_name'), "/base_link"]})
    

    ## Default behavior trees
    navigate_to_pose_bt = os.path.join(get_package_share_directory(robot+'_navigation'), 'behavior_trees', 'navigate_to_pose_w_simple_recovery.xml')
    navigate_through_poses_bt = os.path.join(get_package_share_directory(robot+'_navigation'), 'behavior_trees', 'navigate_through_poses_w_simple_recovery.xml')

    ## Lifecycle nodes
    lifecycle_nodes = ['map_server',
                       'coverage_server',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'velocity_smoother']
    
    ## Nodes 
    nodes = GroupAction(
        actions=[
            PushRosNamespace(condition=IfCondition(LaunchConfiguration('use_namespace')), 
                            namespace=LaunchConfiguration('robot_name')),
            Node(
                package='nav2_map_server',
                executable='map_server',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            {'yaml_filename': LaunchConfiguration('costmap_path')}],
            ),
            Node(
                package='opennav_coverage',
                executable='coverage_server',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            navigation_params],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            navigation_params],   
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            navigation_params],   
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            navigation_params],   
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            navigation_params],   
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            {'default_nav_to_pose_bt_xml': navigate_to_pose_bt}, 
                            {'default_nav_through_poses_bt_xml': navigate_through_poses_bt},
                            navigation_params],   
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                emulate_tty=True, 
                parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                            navigation_params],   
                remappings=[('cmd_vel_smoothed', 'diff_drive_controller/cmd_vel_unstamped')],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                emulate_tty=True, 
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}],
            )
        ]
    )
  

    ## Launch description
    return LaunchDescription([
        
        # Arguments
        sim_arg,
        robot_name_arg,
        namespace_arg,
        costmap_arg,

        # Or as seperate nodes
        nodes,

    ])