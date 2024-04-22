# Requirements:
# Example:

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'semantic_rules'

    parameters = [{
        'direction_type': 'linear',
        'cost_type': 'reverse_falloff',
        'falloff_type': 'abs_percentage',
        'base_cost': 20.0,
        'falloff': -0.20,
        'min_range': 0,
        'max_range': 10,
    }]

    tracking_topic = "tracking"
    # detections_topic = "/yolo/detections_3d"
    remappings = [
        # ('costmap_publisher', '/costmap/rule1'),
        ('tracking', tracking_topic),
        # ('tracking_marker', '/yolo/dgb_kp_markers'),
    ]
    #
    # ARGS
    #
    model = LaunchConfiguration("model")  # TODO: why is this needed?
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8m.pt",
        description="Model name or path")
    #
    # NODES
    #

    rule1 = Node(
            package=package_name, executable='detection_costmap_rule', output='screen',
            parameters=parameters,
            remappings=remappings)

    ld = LaunchDescription()

    # ld.add_action(model_cmd)
    ld.add_action(rule1)
    return ld
