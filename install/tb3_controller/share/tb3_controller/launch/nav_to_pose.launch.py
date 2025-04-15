#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # 声明导航目标参数
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='1.0',
        description='目标位置的X坐标'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='1.0',
        description='目标位置的Y坐标'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='1.57',  # 约90度，弧度制
        description='目标朝向（弧度）'
    )
    
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
    
    # 创建导航节点
    navigation_node = Node(
        package='tb3_controller',
        executable='nav_to_pose.py',
        name='navigation_client',
        output='screen',
        parameters=[{
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw')
        }]
    )
    
    # 返回启动描述
    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        yaw_arg,
        set_initial_pose_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        navigation_node
    ])
