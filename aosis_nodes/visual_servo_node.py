#!/usr/bin/env python3
"""
AOSIS Visual Servo Controller Node
Subscribes to /aosis/detection, computes pixel error from image center,
publishes velocity commands to /aosis/cmd_vel for the Teensy TMC5160T.

  data[0] = X velocity  (left/right)
  data[1] = Y velocity  (fore/aft depth)
  data[2] = Z velocity  (vertical)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

IMAGE_WIDTH              = 1920
IMAGE_HEIGHT             = 1080
KP_X                     = 0.05
KP_Z                     = 0.05
DEAD_ZONE_PX             = 15.0
MAX_VEL                  = 5.0
PROXIMITY_AREA_THRESHOLD = 250000.0
DETECTION_TIMEOUT_CYCLES = 10


class VisualServoNode(Node):

    def __init__(self):
        super().__init__('aosis_visual_servo')
        self.sub = self.create_subscription(
            Float32MultiArray, '/aosis/detection',
            self.detection_callback, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/aosis/cmd_vel', 10)

        self.image_cx  = IMAGE_WIDTH  / 2.0
        self.image_cy  = IMAGE_HEIGHT / 2.0
        self.lost_count = 0
        self.tracking   = False
        self.get_logger().info('Visual servo ready — image center (%.0f, %.0f)'
                               % (self.image_cx, self.image_cy))

    def detection_callback(self, msg):
        data       = msg.data
        centroid_x = data[0]
        centroid_y = data[1]
        bbox_area  = data[2]
        detected   = data[4] > 0.5

        cmd = Float32MultiArray()

        if not detected:
            self.lost_count += 1
            if self.lost_count >= DETECTION_TIMEOUT_CYCLES:
                if self.tracking:
                    self.get_logger().info('Target lost — stopping motors')
                    self.tracking = False
                cmd.data = [0.0, 0.0, 0.0]
                self.pub.publish(cmd)
            return

        self.lost_count = 0
        if not self.tracking:
            self.get_logger().info('Target acquired — begin tracking')
            self.tracking = True

        error_x = centroid_x - self.image_cx
        error_z = centroid_y - self.image_cy

        if abs(error_x) < DEAD_ZONE_PX: error_x = 0.0
        if abs(error_z) < DEAD_ZONE_PX: error_z = 0.0

        vel_x = max(-MAX_VEL, min(MAX_VEL, KP_X * error_x))
        vel_z = max(-MAX_VEL, min(MAX_VEL, KP_Z * error_z))

        # Proximity guard — stop ALL axes if target too close to lens
        if bbox_area > PROXIMITY_AREA_THRESHOLD:
            self.get_logger().warn(
                'PROXIMITY STOP — bbox_area=%.0f' % bbox_area)
            cmd.data = [0.0, 0.0, 0.0]
            self.pub.publish(cmd)
            return

        vel_y = 0.0  # Y depth — placeholder until RealSense integrated
        cmd.data = [vel_x, vel_y, vel_z]
        self.pub.publish(cmd)

        self.get_logger().debug(
            'err=(%.0f,%.0f) vel=(%.2f,%.2f,%.2f)'
            % (error_x, error_z, vel_x, vel_y, vel_z))


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Float32MultiArray()
        stop.data = [0.0, 0.0, 0.0]
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
