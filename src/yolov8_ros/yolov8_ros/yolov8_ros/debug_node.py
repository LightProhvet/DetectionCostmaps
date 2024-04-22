# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import cv2
import random
import numpy as np
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import KeyPoint3D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray


class DebugNode(Node):

    def __init__(self) -> None:
        super().__init__("debug_node")

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        # params
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # pubs
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        self._bb_markers_pub = self.create_publisher(
            MarkerArray, "dgb_bb_markers", 10)
        self._kp_markers_pub = self.create_publisher(
            MarkerArray, "dgb_kp_markers", 10)

        # subs
        image_sub = message_filters.Subscriber(
            self, Image, "image_raw", qos_profile=image_qos_profile)
        detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (image_sub, detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)

    def draw_box(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:

        # get detection info
        label = detection.class_name
        score = detection.score
        box_msg: BoundingBox2D = detection.bbox
        track_id = detection.id

        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
                  round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
                  round(box_msg.center.position.y + box_msg.size.y / 2.0))

        # draw box
        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

        # write text
        label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
        pos = (min_pt[0] + 5, min_pt[1] + 25)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, label, pos, font,
                    1, color, 1, cv2.LINE_AA)

        return cv_image

    def draw_mask(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:

        mask_msg = detection.mask
        mask_array = np.array([[int(ele.x), int(ele.y)]
                              for ele in mask_msg.data])

        if mask_msg.data:
            layer = cv_image.copy()
            layer = cv2.fillPoly(layer, pts=[mask_array], color=color)
            cv2.addWeighted(cv_image, 0.4, layer, 0.6, 0, cv_image)
            cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True,
                                     color=color, thickness=2, lineType=cv2.LINE_AA)
        return cv_image

    def draw_keypoints(self, cv_image: np.array, detection: Detection) -> np.array:

        keypoints_msg = detection.keypoints

        ann = Annotator(cv_image)

        kp: KeyPoint2D
        for kp in keypoints_msg.data:
            color_k = [int(x) for x in ann.kpt_color[kp.id - 1]
                       ] if len(keypoints_msg.data) == 17 else colors(kp.id - 1)

            cv2.circle(cv_image, (int(kp.point.x), int(kp.point.y)),
                       5, color_k, -1, lineType=cv2.LINE_AA)

        def get_pk_pose(kp_id: int) -> Tuple[int]:
            for kp in keypoints_msg.data:
                if kp.id == kp_id:
                    return (int(kp.point.x), int(kp.point.y))
            return None

        for i, sk in enumerate(ann.skeleton):
            kp1_pos = get_pk_pose(sk[0])
            kp2_pos = get_pk_pose(sk[1])

            if kp1_pos is not None and kp2_pos is not None:
                cv2.line(cv_image, kp1_pos, kp2_pos, [
                    int(x) for x in ann.limb_color[i]], thickness=2, lineType=cv2.LINE_AA)

        return cv_image

    def create_bb_marker(self, detection: Detection) -> Marker:

        bbox3d = detection.bbox3d

        marker = Marker()
        marker.header.frame_id = bbox3d.frame_id

        marker.ns = "yolov8_3d"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = bbox3d.center.position.x
        marker.pose.position.y = bbox3d.center.position.y
        marker.pose.position.z = bbox3d.center.position.z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = bbox3d.size.x
        marker.scale.y = bbox3d.size.y
        marker.scale.z = bbox3d.size.z

        marker.color.b = 0.0
        marker.color.g = detection.score * 255.0
        marker.color.r = (1.0 - detection.score) * 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = detection.class_name

        return marker

    def create_kp_marker(self, keypoint: KeyPoint3D) -> Marker:

        marker = Marker()

        marker.ns = "yolov8_3d"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = keypoint.point.x
        marker.pose.position.y = keypoint.point.y
        marker.pose.position.z = keypoint.point.z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.b = keypoint.score * 255.0
        marker.color.g = 0.0
        marker.color.r = (1.0 - keypoint.score) * 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = str(keypoint.id)

        return marker

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        bb_marker_array = MarkerArray()
        kp_marker_array = MarkerArray()

        detection: Detection
        for detection in detection_msg.detections:

            # random color
            label = detection.class_name

            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)

            color = self._class_to_color[label]

            cv_image = self.draw_box(cv_image, detection, color)
            cv_image = self.draw_mask(cv_image, detection, color)
            cv_image = self.draw_keypoints(cv_image, detection)

            if detection.bbox3d.frame_id:
                marker = self.create_bb_marker(detection)
                marker.header.stamp = img_msg.header.stamp
                marker.id = len(bb_marker_array.markers)
                bb_marker_array.markers.append(marker)

            if detection.keypoints3d.frame_id:
                for kp in detection.keypoints3d.data:
                    marker = self.create_kp_marker(kp)
                    marker.header.frame_id = detection.keypoints3d.frame_id
                    marker.header.stamp = img_msg.header.stamp
                    marker.id = len(kp_marker_array.markers)
                    kp_marker_array.markers.append(marker)

        # publish dbg image
        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,
                                                           encoding=img_msg.encoding))
        self._bb_markers_pub.publish(bb_marker_array)
        self._kp_markers_pub.publish(kp_marker_array)


def main():
    rclpy.init()
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
