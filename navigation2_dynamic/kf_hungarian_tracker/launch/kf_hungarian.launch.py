import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tracker_config = os.path.join(
        get_package_share_directory('kf_hungarian_tracker'),
        'config',
        'kf_hungarian.yaml'
    )

    kf_hungarian_node = Node(
        package='kf_hungarian_tracker',
        name='kf_hungarian_node',
        executable='kf_hungarian_node',
        parameters=[tracker_config],
        remappings=[
            # ("tracking", "kf_tracking"),
            # ("detection", "/yolo/detections_3d")
        ]
    )

    return launch.LaunchDescription([kf_hungarian_node])
