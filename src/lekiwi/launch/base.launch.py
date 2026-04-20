#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 路径配置
    pkg_path = FindPackageShare('lekiwi')
    urdf_path = PathJoinSubstitution([pkg_path, 'urdf', 'lekiwi_base.urdf'])
    controller_config = PathJoinSubstitution([pkg_path, 'config', 'controllers.yaml'])

    # 读取机器人描述（标准写法，不会报错）
    robot_description = {
        'robot_description': Command(['xacro ', urdf_path])
    }

    # 机器人状态发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    # 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, robot_description],
        output='screen',
    )
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_gui': False}],  # True 打开滑块调关节角度
    # )

    # # 启动关节状态广播
    # joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    # )

    # 启动底盘控制器
    wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lekiwi_wheel_controller'],
    )
    # # 按顺序启动
    # delay_wheel_controller = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=joint_state_broadcaster,
    #         on_exit=[wheel_controller],
    #     )
    # )
    # Holonomic controller for omniwheels
    base_controller_node = Node(
        package='lekiwi',
        executable='base_controller.py',
        name='base_controller',
        parameters=[{
            'wheel_radius': 0.05,      # 轮径
            'base_radius': 0.125,      # 底盘半径
            'max_wheel_velocity': 3.0, # 最大加速度
            'cmd_timeout': 0.5,        # 松手延迟， 舵机需要缓慢停下来，否则会很快过热
            'ramp_rate': 5.0,             # 加速限制，单位 m/s^2 或 rad/s^2
            'control_freq': 50.0,       # 底盘控制频率
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
        }],
        output='screen',
    )


    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        wheel_controller,
        base_controller_node,
    ])
