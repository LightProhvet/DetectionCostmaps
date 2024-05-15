import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'semantic_rules'
    #
    # Command-line args?
    #
    model = LaunchConfiguration("model")  # TODO: why is this needed?
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8m.pt",
        description="Model name or path")

    #
    # NODE ARGS
    #
    # detection to obstacle
    obstacle_topic = "detection"
    detections_topic = "/yolo/detections_3d"

    parameters_dto = [{
        "semantic_classification": "binary",
        "publisher_count": 1,
        "min_range": 0.0,
        "max_range": 20.0,
        # "dynamic_objects": [],
    }]
    remappings_dto = [
        ('/obstacles', obstacle_topic),
        ('/detections_3d', detections_topic),
    ]
    # kf hungarian tracker
    tracker_config = os.path.join(
        get_package_share_directory('kf_hungarian_tracker'),
        'config',
        'kf_hungarian.yaml'
    )
    tracker_remappings = [
            # ("tracking", "kf_tracking"),
            # ("detection", "/yolo/detections_3d")
    ]

    assigner_parameters = [{
        'publisher_count': 3,
        'min_range': 0,
        'max_range': 6,
        'semantic_classification': "all",

    }]
    # semantic rules
    # TODO: rule params to config
    rule1_parameters = [{
        'direction_type': 'front',
        'cost_type': 'velocity_falloff',
        'falloff_type': 'abs_percentage',
        'base_cost': 20.0,
        'falloff': -0.20,
        'min_range': 0,
        'velocity_segments': 5,
        'velocity_duration': 1.0,
    }]
    rule2_parameters = [{
        'direction_type': 'front',
        'cost_type': 'velocity_falloff',
        'falloff_type': 'abs_percentage',
        'base_cost': 20.0,
        'falloff': -0.30,
        'min_range': 0,
        'velocity_segments': 5,
        'velocity_duration': 3,
    }]
    rule3_parameters = [{
        'direction_type': 'front',
        'cost_type': 'direction_falloff',
        'falloff_type': 'abs_percentage',
        'base_cost': 20.0,
        'falloff': -0.50,
        'min_range': 0,
        'velocity_segments': 5,
        'velocity_duration': 5,
    }]

    rule1_remappings = [
        ('costmap_publisher', '/costmap/rule1'),
        ('tracking', "tracking1"),
    ]
    rule2_remappings = [
        ('costmap_publisher', '/costmap/rule2'),
        ('tracking', "tracking2"),
    ]
    rule3_remappings = [
        ('costmap_publisher', '/costmap/rule3'),
        ('tracking', "tracking3"),
    ]

    #
    # NODES
    #
    detection_converter = Node(
        package=package_name, executable='detection_converter_node', output='screen',
        parameters=parameters_dto,
        remappings=remappings_dto)

    kf_hungarian_node = Node(
        package='kf_hungarian_tracker',
        name='kf_hungarian_node',
        executable='kf_hungarian_node',
        parameters=[tracker_config],
        remappings=tracker_remappings
    )

    rule_assigner_node = Node(
        package=package_name,
        name='rule_assigner_node',
        executable='rule_assigner_node',
        parameters=[tracker_config],
        remappings=tracker_remappings
    )

    rule1 = Node(
            package=package_name, executable='detection_costmap_rule', output='screen',
            parameters=rule1_parameters,
            remappings=rule1_remappings)
    rule2 = Node(
            package=package_name, executable='detection_costmap_rule', output='screen',
            parameters=rule2_parameters,
            remappings=rule2_remappings)
    rule3 = Node(
            package=package_name, executable='detection_costmap_rule', output='screen',
            parameters=rule3_parameters,
            remappings=rule3_remappings)

    ld = LaunchDescription()

    # ld.add_action(model_cmd)
    ld.add_action(detection_converter)
    ld.add_action(kf_hungarian_node)
    ld.add_action(rule_assigner_node)
    ld.add_action(rule1)
    ld.add_action(rule2)
    ld.add_action(rule3)

    return ld
