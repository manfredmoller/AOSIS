import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    realsense_config = os.path.join(
        '/workspaces/isaac_ros-dev/src/aosis/config',
        'realsense_aosis.yaml'
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[realsense_config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        realsense_node,
    ])
