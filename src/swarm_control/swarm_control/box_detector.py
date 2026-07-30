#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealSenseBoxDetectorNode(Node):
    def __init__(self):
        super().__init__('box_detector_node')
        self.bridge = CvBridge()
        self.drones = ['mother', 'child_1', 'child_2', 'child_3']
        
        self.depth_maps = {drone: None for drone in self.drones}
        self.detections = {drone: False for drone in self.drones}
        
        for drone in self.drones:
            # Subscribing to exact world-prefixed ROS 2 topics
            self.create_subscription(
                Image,
                f'/world/swarm_world/model/{drone}/link/body/sensor/realsense_depth/depth_image',
                lambda msg, name=drone: self.depth_callback(msg, name),
                qos_profile=qos_profile_sensor_data
            )
            self.create_subscription(
                Image,
                f'/world/swarm_world/model/{drone}/link/body/sensor/realsense_rgb/image',
                lambda msg, name=drone: self.rgb_callback(msg, name),
                qos_profile=qos_profile_sensor_data
            )
        
        self.detection_pub = self.create_publisher(Bool, '/swarm/box_visual_detection', 10)
        self.target_3d_pub = self.create_publisher(PointStamped, '/swarm/target_3d_position', 10)
        self.get_logger().info('👁️ RealSense Visual Perception Node Active. Listening for camera streams...')

    def depth_callback(self, msg, drone_name):
        self.depth_maps[drone_name] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg, drone_name):
        if self.depth_maps[drone_name] is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Expanded HSV range for red detection under Gazebo lighting
            lower_red1 = np.array([0, 100, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 50])
            upper_red2 = np.array([180, 255, 255])

            mask = cv2.inRange(hsv_image, lower_red1, upper_red1) + cv2.inRange(hsv_image, lower_red2, upper_red2)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            box_visible = False

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:
                    box_visible = True
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    u_center = int(x + w / 2)
                    v_center = int(y + h / 2)

                    depth_map = self.depth_maps[drone_name]
                    if 0 <= v_center < depth_map.shape[0] and 0 <= u_center < depth_map.shape[1]:
                        z_distance = float(depth_map[v_center, u_center])

                        if not np.isnan(z_distance) and z_distance > 0.1:
                            fx, fy, cx, cy = 380.0, 380.0, 320.0, 240.0
                            x_3d = (u_center - cx) * z_distance / fx
                            y_3d = (v_center - cy) * z_distance / fy

                            target_point = PointStamped()
                            target_point.header.stamp = self.get_clock().now().to_msg()
                            target_point.header.frame_id = f'{drone_name}_realsense'
                            target_point.point.x = x_3d
                            target_point.point.y = y_3d
                            target_point.point.z = z_distance
                            self.target_3d_pub.publish(target_point)

                            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.putText(cv_image, f'Box: {z_distance:.2f}m', (x, y - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.detections[drone_name] = box_visible
            
            detection_msg = Bool()
            detection_msg.data = any(self.detections.values())
            self.detection_pub.publish(detection_msg)

            if drone_name == 'mother':
                cv2.imshow("Mother RealSense Feed", cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing stream for {drone_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseBoxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()