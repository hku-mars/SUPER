import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('perfect_drone_sim')

    default_rviz_config_path = os.path.join(
        package_path, 'rviz2', 'fpv.rviz')

    default_config_path = 'waypoint.yaml'
    default_data_path =  'benchmark.txt'

    perfect_drone_sim_config_name = 'high_speed.yaml'
    super_config_name = 'static_high_speed.yaml'


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_name', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_data_path_cmd = DeclareLaunchArgument(
        'data_name', default_value=default_data_path,
        description='Yaml config file path'
    )

    perfect_drone_sim_config_name_cmd = DeclareLaunchArgument(
        'config_name', default_value=perfect_drone_sim_config_name,
        description='Yaml config file path'
    )

    super_config_name_cmd = DeclareLaunchArgument(
        'config_name', default_value=super_config_name,
        description='Yaml config file path'
    )



    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_data_path_cmd)
    ld.add_action(perfect_drone_sim_config_name_cmd)
    ld.add_action(super_config_name_cmd)


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen'
    )
    ld.add_action(rviz_node)

    mission_planner = Node(
        package='mission_planner',
        executable='waypoint_mission',
        output='log',
        parameters=[{
            'config_name': default_config_path,
            'data_name': default_data_path
        }]
    )
    ld.add_action(mission_planner)

    perfect_drone_sim = Node(
        package='perfect_drone_sim',
        executable='perfect_drone_node',
        output='log',
        parameters=[{
            'config_name': perfect_drone_sim_config_name,
        }]
    )

    ld.add_action(perfect_drone_sim)

    SUPER = Node(
        package='super_planner',
        executable='fsm_node',
        output='screen',
        parameters=[{
            'config_name': super_config_name,
        }]
    )

    ld.add_action(SUPER)

    return ld
