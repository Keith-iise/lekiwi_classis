from launch import LaunchDescription
from launch_ros.actions import Node
from controller_manager.launch_utils import generate_load_controller_launch_description
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lekiwi'),
        'config',
        'lekiwi_controllers.yaml'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config],
        output='screen',
    )

    velocity_controller = generate_load_controller_launch_description(
        controller_name='joint_velocity_controller',
        controller_type='velocity_controllers/JointGroupVelocityController'
    )

    return LaunchDescription([
        control_node,
        velocity_controller,
    ])
