#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 查找包路径
    tb3_controller_dir = FindPackageShare('tb3_controller').find('tb3_controller')
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    # 使用相对路径查找文件
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(tb3_controller_dir))))
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(workspace_dir, 'world', 'challenge.world'),
        description='Full path to world model file to load')
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute headless (no GUI)')
    # 使用相对路径查找机器人模型文件
    robot_sdf_path = os.path.join(workspace_dir, 'src', 'robots', 'turtlebot3_burger', 'model.sdf')
    robot_sdf_arg = DeclareLaunchArgument(
        'robot_sdf',
        default_value=robot_sdf_path,
        description='Robot SDF model file')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(workspace_dir, 'maps', 'challenge_map.yaml'),
        description='Full path to map yaml file to load')
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='-1.57',
        description='Initial yaw angle')
    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        default_value='0.0',
        description='Initial x position for AMCL')
    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        default_value='0.0',
        description='Initial y position for AMCL')
    initial_z_arg = DeclareLaunchArgument(
        'initial_z',
        default_value='0.0',
        description='Initial z position for AMCL')

    # 设置tb3_simulation_launch.py路径
    tb3_sim_launch = os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
    
    # 设置Nav2参数文件路径 - 使用绝对路径确保正确找到文件
    nav2_params_path = os.path.join(workspace_dir, 'src', 'nav2', 'nav2_params.yaml')
    print(f'Nav2参数文件路径: {nav2_params_path}')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Include the original tb3_simulation_launch.py with forwarded arguments
    tb3_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'map': LaunchConfiguration('map'),
            'yaw': LaunchConfiguration('yaw'),
            'params_file': LaunchConfiguration('params_file'),
            'robot_sdf': LaunchConfiguration('robot_sdf'),
        }.items()
    )

    # 创建设置初始位姿的节点
    set_initial_pose_node = Node(
        package='tb3_controller',
        executable='set_initial_pose.py',
        name='initial_pose_node',
        output='screen',
        parameters=[
            {'initial_x': LaunchConfiguration('initial_x')},
            {'initial_y': LaunchConfiguration('initial_y')},
            {'initial_z': LaunchConfiguration('initial_z')},
            {'initial_yaw': LaunchConfiguration('yaw')}
        ],
    )
    
    # 使用TimerAction延迟启动初始位姿节点，等待Nav2完全启动
    delayed_initial_pose = TimerAction(
        period=10.0,  # 延迟10秒，等待Nav2完全启动
        actions=[set_initial_pose_node]
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        map_arg,
        yaw_arg,
        initial_x_arg,
        initial_y_arg,
        initial_z_arg,
        params_arg,
        robot_sdf_arg,
        tb3_simulation,
        delayed_initial_pose  # 使用延迟启动的初始位姿节点
    ])
