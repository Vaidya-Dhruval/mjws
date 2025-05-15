import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_accessories')

    urdf_path = os.path.join(pkg_share, 'configs', 'ridgeback_dummy.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = {'robot_description': infp.read()}

    mujoco_model_path = os.path.join(pkg_share, 'robot', 'ridgeback_default.xml')
    controller_config = os.path.join(pkg_share, 'configs', 'ridgeback_controllers.yaml')
    jsb_config = os.path.join(pkg_share, 'configs', 'joint_state_broadcaster.yaml')
    velocity_config = os.path.join(pkg_share, 'configs', 'ridgeback_velocity_controller.yaml')

    return LaunchDescription([
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            output='screen',
            parameters=[
                robot_description,
                {'mujoco_model_path': mujoco_model_path},
                controller_config
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[jsb_config],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ridgeback_velocity_controller'],
            parameters=[velocity_config],
            output='screen'
        )
    ])
