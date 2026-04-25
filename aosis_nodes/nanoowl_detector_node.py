#!/usr/bin/env python3
"""
AOSIS NanoOWL Detector Node
Captures frames from UC70 camera, runs NanoOWL inference,
publishes bounding box centroid for visual servoing.

Topic published: /aosis/detection  (Float32MultiArray)
  data[0] = centroid_x (pixels)
  data[1] = centroid_y (pixels)
  data[2] = bbox_area  (pixels^2, used for Z/depth proxy)
  data[3] = confidence score
  data[4] = 1.0 if detected, 0.0 if not
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import cv2
from PIL import Image

from nanoowl.owl_predictor import OwlPredictor

CAMERA_INDEX     = 0
IMAGE_WIDTH      = 1920
IMAGE_HEIGHT     = 1080
ENGINE_PATH      = '/workspaces/isaac_ros-dev/src/nanoowl/data/owl_image_encoder_patch32.engine'
DETECT_LABELS    = ['a soldering iron', 'a soldering wand', 'a hand holding a soldering iron']
THRESHOLD        = 0.1
TARGET_LABEL_IDX = 0
PUBLISH_HZ       = 30


class NanoOwlDetectorNode(Node):

    def __init__(self):
        super().__init__('nanoowl_detector')
        self.pub = self.create_publisher(Float32MultiArray, '/aosis/detection', 10)

        self.get_logger().info('Opening UC70 camera...')
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMAGE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError('Camera not available at index %d' % CAMERA_INDEX)
        self.get_logger().info('Camera opened at %dx%d' % (IMAGE_WIDTH, IMAGE_HEIGHT))

        self.get_logger().info('Loading NanoOWL predictor...')
        self.predictor = OwlPredictor(
            'google/owlvit-base-patch32',
            image_encoder_engine=ENGINE_PATH
        )
        self.text_encodings = self.predictor.encode_text(DETECT_LABELS)
        self.get_logger().info('NanoOWL ready. Tracking: %s' % DETECT_LABELS)

        # Dedicated capture thread — decouples grab latency from inference
        self._frame = None
        self._lock  = threading.Lock()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer       = self.create_timer(1.0 / PUBLISH_HZ, self.detect_and_publish)
        self.frame_count = 0

    def _capture_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def detect_and_publish(self):
        with self._lock:
            if self._frame is None:
                return
            frame = self._frame.copy()

        image  = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        output = self.predictor.predict(
            image=image,
            text=DETECT_LABELS,
            text_encodings=self.text_encodings,
            threshold=THRESHOLD
        )

        best_score = -1.0
        best_box   = None
        if output.labels is not None and len(output.labels) > 0:
            for i in range(len(output.labels)):
                if output.labels[i].item() == TARGET_LABEL_IDX:
                    score = output.scores[i].item()
                    if score > best_score:
                        best_score = score
                        best_box   = output.boxes[i]

        msg = Float32MultiArray()
        if best_box is not None:
            xmin, ymin, xmax, ymax = [v.item() for v in best_box]
            cx   = (xmin + xmax) / 2.0
            cy   = (ymin + ymax) / 2.0
            area = (xmax - xmin) * (ymax - ymin)
            msg.data = [cx, cy, area, best_score, 1.0]
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    'Detected @ (%.0f, %.0f) score=%.2f' % (cx, cy, best_score))
        else:
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.pub.publish(msg)
        self.frame_count += 1

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NanoOwlDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
