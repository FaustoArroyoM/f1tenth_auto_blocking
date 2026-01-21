#!/usr/bin/env python3
"""Example: Subscribe to distance estimates."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class DistanceListener(Node):
    def __init__(self):
        super().__init__('distance_listener')
        
        self.create_subscription(
            Float32MultiArray,
            'car_distance/all_distances',
            self.callback,
            10
        )
    
    def callback(self, msg):
        stereo, focal, calib, hybrid, yolo_conf, confidence = msg.data[:6]
        print(f"Distance: {hybrid:.2f}m (Conf: {confidence:.3f})")

def main():
    rclpy.init()
    listener = DistanceListener()
    rclpy.spin(listener)

if __name__ == '__main__':
    main()
