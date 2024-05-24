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

    #
    # NODE ARGS
    #
    # detection to obstacle
    obstacle_topic = "detection"
    detections_topic = "/yolo/detections_3d"

    parameters_dto = []
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
            ("tracking", "kf_tracking"),
            # ("detection", "/yolo/detections_3d")
    ]
    # rule assigner

    assigner_parameters = [{
        'publisher_count': 3,
        'min_range': 0.0,
        'max_range': 5.0,
        'semantic_classification': "binary",

    }]
    assigner_remappings = [
        ("tracking", "kf_tracking"),
    ]

    # semantic rules
    # TODO: rule params to config
    rule1_parameters = [{
        'direction_type': 'front',
        'reverse_falloff': False,
        'cost_type': 'velocity_falloff',
        'falloff_type': 'abs_percentage',
        'base_cost': 80.0,
        'falloff': 0.2,
        'min_range': 0,
        'velocity_segments': 5,
        'velocity_duration': 1.0,
        'use_sim_time': False,
        'resolution': 0.1,
        'width': 100,
        'height': 100
    }]
    rule2_parameters = [{
        'direction_type': 'front',
        'cost_type': 'velocity_falloff',
        'falloff_type': 'linear',
        'reverse_falloff': True,
        'base_cost': 80.0,
        'falloff': 10.0,
        'min_range': 0,
        'velocity_segments': 4,
        'velocity_duration': 2.0,
        'use_sim_time': False,
        'resolution': 0.1,
        'width': 100,
        'height': 100
    }]
    rule3_parameters = [{
        'direction_type': 'front',
        'cost_type': 'direction_falloff',
        'reverse_falloff': True,
        'falloff_type': 'rel_percentage',
        'base_cost': 80.0,
        'falloff': 0.10,
        'min_range': 0,
        'velocity_segments': 4,
        'velocity_duration': 0.8,
        'use_sim_time': False,
        'resolution': 0.1,
        'width': 100,
        'height': 100
    }]

    rule1_remappings = [
        ('costmap_publisher', '/costmap/rule1'),
        ('tracking', "kf_tracking"),
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
        parameters=assigner_parameters,
        remappings=assigner_remappings
    )

    rule1 = Node(
        package=package_name,
        name='to2_meter_rule',
        executable='detection_costmap_rule', output='screen',
        parameters=rule1_parameters,
        remappings=rule1_remappings)
    rule2 = Node(
        package=package_name,
        name='to4_meter_rule',
        executable='detection_costmap_rule',
        output='screen',
        parameters=rule2_parameters,
        remappings=rule2_remappings)
    rule3 = Node(
        name='to6_meter_rule',
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
