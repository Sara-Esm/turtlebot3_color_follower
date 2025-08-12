#!/usr/bin/env python3
"""
color_detector.py
Single-node implementation: color detection (OpenCV) + motion controller (P-controller).
Publish /cmd_vel to drive the TurtleBot3 toward a red object and stop when it's close.
Parameters are declared so you can tune behavior without editing code.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.qos import QoSProfile

def clamp(v, mn, mx):
    return max(mn, min(mx, v))

class ColorDetectorAndFollower(Node):
    def __init__(self):
        super().__init__('color_detector')

        # --- parameters (tweakable from launch or ros2 param set) ---
        p = self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/image_raw'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('linear_speed', 0.15),
                ('angular_gain', 0.002),    # P gain for rotation
                ('center_tolerance', 30),   # pixels
                ('min_area', 500.0),        # min contour area to consider
                ('max_area', 8000.0),       # if area > max_area -> stop (too close)
                ('smoothing_alpha', 0.6),   # EMA smoothing for cx/area
                ('max_angular', 0.6),       # limit rotation speed
                ('max_linear', 0.25),       # limit forward speed
                ('lost_timeout', 1.0),      # seconds to consider target lost
                ('show_debug_windows', False),  # Show OpenCV windows
            ]
        )

        # read params into attributes for convenience
        self.image_topic = p[0].value
        self.cmd_vel_topic = p[1].value
        self.linear_speed = float(p[2].value)
        self.angular_gain = float(p[3].value)
        self.center_tolerance = float(p[4].value)
        self.min_area = float(p[5].value)
        self.max_area = float(p[6].value)
        self.smoothing_alpha = float(p[7].value)
        self.max_angular = float(p[8].value)
        self.max_linear = float(p[9].value)
        self.lost_timeout = float(p[10].value)
        self.show_debug = bool(p[11].value)

        # --- ROS interfaces ---
        qos = QoSProfile(depth=5)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # --- internal state ---
        self.smoothed_cx = None
        self.smoothed_area = None
        self.last_seen = self.get_clock().now()

        # small periodic timer to enforce stop when lost
        self.create_timer(0.2, self.control_watchdog)

        self.get_logger().info("✅ Color Detector & Motion Controller Node Started")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")

    def control_watchdog(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_seen).nanoseconds * 1e-9
        if elapsed > self.lost_timeout:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.get_logger().info("🚫 Lost target — stopping robot")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        height, width = frame.shape[:2]
        frame_center_x = width // 2

        # convert to HSV and threshold red (two ranges)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # morphology to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            # largest contour
            c = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(c))

            if area >= self.min_area:
                M = cv2.moments(c)
                if M['m00'] == 0:
                    cx = frame_center_x
                else:
                    cx = float(M['m10'] / M['m00'])

                # smoothing
                if self.smoothed_cx is None:
                    self.smoothed_cx = cx
                    self.smoothed_area = area
                else:
                    a = self.smoothing_alpha
                    self.smoothed_cx = a * cx + (1.0 - a) * self.smoothed_cx
                    self.smoothed_area = a * area + (1.0 - a) * self.smoothed_area

                error_x = float(self.smoothed_cx - frame_center_x)

                self.get_logger().info(f"Detected object at cx={int(self.smoothed_cx)}, area={self.smoothed_area:.1f}")

                # If object is TOO CLOSE: stop immediately
                if self.smoothed_area >= self.max_area:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("🚫 Object too close — stopping")
                else:
                    # If large angular error -> rotate in place until centered
                    if abs(error_x) > self.center_tolerance:
                        ang = - self.angular_gain * error_x
                        ang = clamp(ang, -self.max_angular, self.max_angular)
                        twist.angular.z = ang
                        twist.linear.x = 0.0
                    else:
                        # centered -> move forward
                        twist.angular.z = 0.0
                        twist.linear.x = clamp(self.linear_speed, 0.0, self.max_linear)

                # draw debugging visuals
                cx_int = int(self.smoothed_cx)
                cv2.circle(frame, (cx_int, int(height/2)), 6, (0,255,0), -1)
                cv2.putText(frame, f'area={self.smoothed_area:.0f}', (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                # update last seen timestamp
                self.last_seen = self.get_clock().now()
            else:
                # contour too small
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # no contour -> stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # publish command
        self.cmd_pub.publish(twist)

        # debug windows (WSLg / local)
        if self.show_debug:
            try:
                cv2.imshow('camera', frame)
                cv2.imshow('mask', mask)
                cv2.waitKey(1)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorAndFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == '__main__':
    main()
