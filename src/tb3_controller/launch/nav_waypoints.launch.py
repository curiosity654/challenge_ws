#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # 声明初始位姿参数
    set_initial_pose_arg = DeclareLaunchArgument(
        'set_initial_pose', default_value='true',
        description='是否设置AMCL的初始位姿'
    )
    
    initial_x_arg = DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='初始位置的X坐标'
    )
    
    initial_y_arg = DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='初始位置的Y坐标'
    )
    
    initial_z_arg = DeclareLaunchArgument(
        'initial_z', default_value='0.0',
        description='初始位置的Z坐标'
    )
    
    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw', default_value='-1.57',
        description='初始朝向（弧度）'
    )
    
    # 创建路径点导航节点
    waypoint_navigator_node = Node(
        package='tb3_controller',
        executable='nav_waypoints.py',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_z': LaunchConfiguration('initial_z'),
            'initial_yaw': LaunchConfiguration('initial_yaw')
        }]
    )
    
    # 返回启动描述
    return LaunchDescription([
        set_initial_pose_arg,
        initial_x_arg,
        initial_y_arg,
        initial_z_arg,
        initial_yaw_arg,
        waypoint_navigator_node
    ])
