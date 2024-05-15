import math
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy import Parameter
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Vector3
from yolov8_msgs.msg import DetectionArray, Detection, BoundingBox3D
from nav2_dynamic_msgs.msg import ObstacleArray, Obstacle
# from tf.transformations import euler_from_quaternion


# A simple Node to convert DetectionArray from yolov8_ros to ObstacleArray from and for our kf_hungarian_tracker
class RuleAssignerNode(Node):
    def __init__(self):
        super().__init__('rule_assigner_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                # input params
                ('publisher_count', 3),
                ('sequence', 1),  # TODO: implement sequence
                # ranges are not used with publisher_count == 1
                ('min_range', Parameter.Type.DOUBLE),
                ('max_range', 6.0),
                # semantic classification params
                ('semantic_classification', 'binary'),
                ('dynamic_objects', ['person', 'human', 'cat', 'dog', 'car', 'truck', 'bus'])  # you can also add id-s here
            ])
        """    Subscribers    """
        self.subscription = self.create_subscription(
            DetectionArray,
            '/tracking',
            self.detection_callback,
            10
        )

        self.data = []
        self.publisher_count = self.get_parameter('publisher_count').get_parameter_value().integer_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.dynamic_objects = set(self.get_parameter('dynamic_objects')._value)

        """   Publisher creation    """
        self.all_publishers = []
        for i in range(1, self.publisher_count+1):
                publisher = self.create_publisher(
                    ObstacleArray,
                    '/obstacles'+str(i),
                    10
                )
                self.all_publishers.append(publisher)
        if len(self.all_publishers):
            self.publisher = self.all_publishers[0]
        else:
            raise UserWarning('No publishers created. Did you set a positive publisher count?')

    def position_callback(self, msg):
        self.current_position = msg

    def detection_callback(self, msg, class_is_obstacle=False):
        # Callback function to handle incoming detection messages

        obstacles = msg.obstacles
        dynamic_obstacles = self.check_for_dynamic_objects(obstacles)
        for index in range(len(self.publisher_count)):
            indexed_obstacles = self.get_indexed_obstacles(index, dynamic_obstacles)
            publisher = self.all_publishers[index]

            indexed_message = ObstacleArray()
            indexed_message.obstacles = indexed_obstacles
            indexed_message.header = msg.header
            publisher.publish(indexed_message)

    def check_for_dynamic_objects(self, obstacles):
        dynamic_obstacles = []
        if not len(self.dynamic_objects):
            return obstacles

        for o in obstacles:
            if o.class_id in self.dynamic_objects or o.class_name in self.dynamic_objects:
                dynamic_obstacles.append(o)
        return dynamic_obstacles

    def get_indexed_obstacles(self, index, detections):
        index_range = (self.max_range - self.min_range) / self.publisher_count
        max_range = (self.min_range + index_range) * (index+1)
        min_range = (self.min_range + index_range) * index
        index_detections = []
        for d in detections:
            detection = d.bbox3d
            center = self.convert_pose_to_point(detection.center)
            # TODO: should we consider z distance?
            distance = math.sqrt((center.x - self.current_position.x)**2 + (center.y - self.current_position.y)**2 + (center.z - self.current_position.z)**2)
            if min_range <= distance <= max_range:
                index_detections.append(d)
        return index_detections

def main(args=None):
    rclpy.init(args=args)
    detection_converter_node = RuleAssignerNode()
    rclpy.spin(detection_converter_node)
    detection_converter_node.destroy_node()
    rclpy.shutdown()

