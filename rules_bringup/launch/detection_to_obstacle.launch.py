# Requirements:
# Example:

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'semantic_rules'

    obstacle_topic = "detection"
    detections_topic = "/yolo/detections_3d"

    parameters = [{
        "semantic_classification": "all",
        "publisher_count": 1,
        "min_range": 0.0,
        "max_range": 20.0,
        # "dynamic_objects": [],
    }]

    remappings = [
        ('/obstacles', obstacle_topic),
        ('/detections_3d', detections_topic),
    ]
    #
    # ARGS
    #
    # obstacle_topic = LaunchConfiguration("obstacle_topic")
    # detection_cmd = DeclareLaunchArgument(
    #     "detection",
    #     default_value="detection",
    #     description="Name for obstacle topic")
    #
    # NODES
    #

    detection_converter = Node(
            package=package_name, executable='detection_converter_node', output='screen',
            parameters=parameters,
            remappings=remappings)

    # TODO: segmentation_converter (segmentation_3d -> Mask)

    ld = LaunchDescription()

    # ld.add_action(model_cmd)
    ld.add_action(detection_converter)
    return ld
