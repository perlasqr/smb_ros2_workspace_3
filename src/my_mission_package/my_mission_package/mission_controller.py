#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from object_detection_msgs.msg import ObjectDetectionInfoArray

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to detection info
        self.detection_sub = self.create_subscription(
            ObjectDetectionInfoArray,
            '/detection_info',
            self.detection_callback,
            10)
        
        # Mission state
        self.mission_done = False
        self.confidence_threshold = 0.5
        self.target_class = 56
        
        # Timer to publish keep-alive /cmd_vel while exploring
        self.timer = self.create_timer(0.5, self.keep_moving)
        
        self.get_logger().info('Mission Controller started. Looking for "{}"...'.format(self.target_class))

    def detection_callback(self, msg):
        if self.mission_done:
            return
        
        for detection in msg.info:
            if detection.class_id == self.target_class and detection.confidence >= self.confidence_threshold:
                self.get_logger().info('Detected "{}" with confidence {:.2f}!'.format(
                    detection.class_id, detection.confidence))
                
                # Stop robot
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                self.get_logger().info('Mission done: target object detected.')
                self.mission_done = True
                return
    
    def keep_moving(self):
        if self.mission_done:
            return
        
        # Optional: send small forward velocity to keep exploring
        move_msg = Twist()
        move_msg.linear.x = 0.2  # Example forward speed
        move_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(move_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
