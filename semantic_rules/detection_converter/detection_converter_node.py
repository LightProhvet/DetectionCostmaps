import math
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy import Parameter
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Vector3
from yolov8_msgs.msg import DetectionArray, Detection, BoundingBox3D
from nav2_dynamic_msgs.msg import ObstacleArray, Obstacle
from warnings import warn
# from tf.transformations import euler_from_quaternion


# A simple Node to convert DetectionArray from yolov8_ros to ObstacleArray from and for our kf_hungarian_tracker
class DetectionConverterNode(Node):
    def __init__(self):
        super().__init__('detection_converter_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sequence', 1),  # TODO: implement sequence
            ])
        """    Subscribers    """
        self.subscription = self.create_subscription(
            DetectionArray,
            '/detections_3d',
            self.detection_callback,
            10
        )

        self.data = []
        self.current_position = Point()

        """   Publisher creation    """
        self.publisher = self.create_publisher(
            ObstacleArray,
            '/obstacles',
            10
        )

    def detection_callback(self, msg):
        # Callback function to handle incoming detection messages
        detections = msg.detections
        self.create_obstacles_and_publish(detections, msg)

    def create_obstacles_and_publish(self, detections, input_msg, publisher_index=-1):
        # TODO TRANSFORM - see get_transform from yolo_v8 detect_3d_node
        # detections = input_msg.detections
        obstacle_msg = ObstacleArray()
        obstacle_msg.header = input_msg.header
        detection_frame = False
        track_list = []
        for d in detections:  # detection is Detection msg
            obstacle = Obstacle()
            detection = d.bbox3d
            obstacle.position = self.convert_pose_to_point(detection.center)
            obstacle.size = detection.size  # Assuming size is directly available in detection
            obstacle.velocity = Vector3()  # self.convert_pose_to_vector(detection.center) # get direction only
            # not Hungarian-related fields:
            obstacle.score = d.score
            obstacle.class_id = d.class_id
            obstacle.class_name = d.class_name
            obstacle.orientation = detection.center  # TODO: either implement orientation in rules / tracker or remove
            # We have 3 options for using detection id-s: 1) Do not use - hungarian does everything
            # 2) use with hungarian - hungarian considers these when tracking 3) use only these - trust detection model
            obstacle.detection_id = d.id
            # TODO: #1 What is the correct way to communicate this need to the user - or can we solve this differently?
            # obstacle.frame_id = detection.frame_id
            if detection_frame and detection.frame_id != detection_frame:
                warn(f"Detected mismatch in detection bounding box frames!!!")
            detection_frame = detection.frame_id
            obstacle.frame_id = input_msg.header.frame_id # write the original header here, as we replace the msg header

            track_list.append(obstacle)
        # TODO #1
        if detection_frame:
            obstacle_msg.header.frame_id = detection_frame

        obstacle_msg.obstacles = track_list
        self.publisher.publish(obstacle_msg)

    @staticmethod
    def convert_pose_to_point(pose):
        point = Point()
        point.x = pose.position.x
        point.y = pose.position.y
        point.z = pose.position.z
        return point

    def pose_to_unit_vector(pose):
        # Extract orientation from the pose
        orientation = pose.orientation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        # Convert quaternion to Euler angles
        r = Rotation.from_quat(quaternion)
        roll, pitch, yaw = r.as_euler('xyz')

        # Calculate the unit vector in the direction of orientation
        unit_vector = Vector3()
        unit_vector.x = math.cos(yaw) * math.cos(pitch)
        unit_vector.y = math.sin(yaw) * math.cos(pitch)
        unit_vector.z = math.sin(pitch)

        return unit_vector

    @staticmethod
    def convert_pose_to_vector(pose):
        # TODO
        return Vector3()


def main(args=None):
    rclpy.init(args=args)
    detection_converter_node = DetectionConverterNode()
    rclpy.spin(detection_converter_node)
    detection_converter_node.destroy_node()
    rclpy.shutdown()

