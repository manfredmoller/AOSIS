import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        parameters=[{
            "enable_depth": True,
            "enable_infra1": True,
            "enable_infra2": True,
            "enable_color": True,
            "depth_width": 848,
            "depth_height": 480,
            "depth_fps": 30,
            "infra_width": 848,
            "infra_height": 480,
            "infra_fps": 30,
            "emitter_enabled": 1,
            "pointcloud__neon_.enable": True,
        }],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/workspaces/isaac_ros-dev/src/aosis/config/aosis_demo.rviz"],
        output="screen",
    )

    return LaunchDescription([
        realsense_node,
        TimerAction(period=6.0, actions=[rviz_node]),
    ])
