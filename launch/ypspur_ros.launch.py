import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros


def generate_launch_description():
    descriptions = launch.LaunchDescription()

    config_file_path = launch.substitutions.LaunchConfiguration(
            'config_file_path')
    parameter_file_path = launch.substitutions.LaunchConfiguration(
            'parameter_file_path')
    device_file_path = launch.substitutions.LaunchConfiguration(
            'device_file_path')

    declare_config_file_path = launch.actions.DeclareLaunchArgument(
            'config_file_path',
            default_value=os.path.join(
                get_package_share_directory('ypspur_ros'),
                'config',
                'ypspur_ros.config.yaml'
                )
            )

    declare_parameter_file_path = launch.actions.DeclareLaunchArgument(
            'parameter_file_path',
            default_value=os.path.join(
                get_package_share_directory('ypspur_ros'),
                'config',
                'icart-mini.param'
                )
            )

    declare_device_file_path = launch.actions.DeclareLaunchArgument(
            'device_file_path',
            default_value='/dev/ttyACM0'
            )

    ypspur_ros_node = launch_ros.actions.Node(
            package='ypspur_ros',
            executable='ypspur_ros',
            name='ypspur_ros',
            parameters=[config_file_path],
            output='screen'
            )

    ypspur_coordinator = launch.actions.ExecuteProcess(
            cmd=[
                'ypspur-coordinator',
                '-p',
                parameter_file_path,
                '-d',
                device_file_path,
                '--without-device',
                '--without-control'
                ],
            output='screen'
            )

    descriptions.add_action(declare_config_file_path)
    descriptions.add_action(declare_parameter_file_path)
    descriptions.add_action(declare_device_file_path)
    descriptions.add_action(ypspur_coordinator)
    descriptions.add_action(ypspur_ros_node)

    return descriptions
