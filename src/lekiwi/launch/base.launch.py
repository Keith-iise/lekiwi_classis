#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ------------------------------------------
    # 仅底盘：无机械臂、无夹爪、无rviz
    # ------------------------------------------

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution(
                [FindPackageShare('lekiwi'), 'urdf', 'lekiwi_base.urdf']
            ),
            ' ',
            'use_fake_hardware:=false'
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # 发布机器人状态
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ros2_control 核心
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([FindPackageShare('lekiwi'), 'config', 'controllers.yaml']),
        ],
        output="screen",
    )

    # 状态广播（必须）
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 底盘速度控制器（唯一需要的控制器）
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lekiwi_wheel_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # 顺序启动：先状态广播 → 再轮子控制器
    delay_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )

    

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_wheel_controller,
        # holonomic_controller_node,
        # odometry_publisher_node
    ])
