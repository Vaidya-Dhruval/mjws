from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('ridgeback_mujoco_sim')

    # Resolve file paths
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'ridgeback.urdf.xacro')
    mujoco_model_path = os.path.join(pkg_path, 'mujoco_models', 'ridgeback.xml')
    controller_config = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')

    # Process Xacro into robot_description
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        # Robot State Publisher with processed URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        # MuJoCo simulation node
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            parameters=[{
                'mujoco_model_path': mujoco_model_path,
                'use_sim_time': True,
                'control_mode': 'velocity'
            }],
            output='screen'
        ),

        # ROS 2 control: controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
            output='screen'
        ),

        # Spawn joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Spawn diff_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
        ),
    ])
