#!/usr/bin/env python3
"""
Standalone test node for car distance estimation.
Publishes mock YOLO detections and subscribes to distance outputs.
Useful for testing without actual YOLO detector.
"""

import rclpy
from rclpy.node import Node
import json
import time
from std_msgs.msg import String, Float32, Float32MultiArray


class DistancePublisherTestNode(Node):
    """Test node that publishes mock YOLO detections and listens to distances."""
    
    def __init__(self):
        super().__init__('distance_publisher_test_node')
        
        # Publisher for mock YOLO detections
        self.yolo_pub = self.create_publisher(
            String,
            '/yolo/detections', 
            10
        )
        
        # Subscribers for distance outputs
        self.create_subscription(
            Float32,
            'car_distance/stereo_depth',
            lambda msg: self.get_logger().info(f'Stereo Depth: {msg.data:.2f}m'),
            10
        )
        
        self.create_subscription(
            Float32,
            'car_distance/focal_length',
            lambda msg: self.get_logger().info(f'Focal Length: {msg.data:.2f}m'),
            10
        )
        
        self.create_subscription(
            Float32,
            'car_distance/calibration',
            lambda msg: self.get_logger().info(f'Calibration: {msg.data:.2f}m'),
            10
        )
        
        self.create_subscription(
            Float32,
            'car_distance/hybrid',
            lambda msg: self.get_logger().info(f'Hybrid: {msg.data:.2f}m'),
            10
        )
        
        self.create_subscription(
            Float32MultiArray,
            'car_distance/all_distances',
            self._all_distances_callback,
            10
        )
        
        # Timer to publish mock detections
        self.timer = self.create_timer(2.0, self._publish_mock_detection)
        self.detection_count = 0
        
        self.get_logger().info('Test node initialized - publishing mock detections every 2 seconds')
    
    def _publish_mock_detection(self):
        """Publish a mock YOLO detection."""
        self.detection_count += 1
        
        # Simulate varying detection (different distances)
        bbox_height = 50 + (self.detection_count % 5) * 20  # Varying height
        
        detection = {
            'detections': [
                {
                    'class': 'car',
                    'x1': 100,
                    'y1': 100,
                    'x2': 300,
                    'y2': 100 + bbox_height,
                    'conf': 0.95
                }
            ],
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(detection)
        self.yolo_pub.publish(msg)
        
        self.get_logger().info(
            f'Published mock detection #{self.detection_count} '
            f'(bbox_height={bbox_height}px)'
        )
    
    def _all_distances_callback(self, msg: Float32MultiArray):
        """Log all distances together."""
        stereo, focal, calib, hybrid, yolo_conf, confidence = msg.data[:6]
        
        self.get_logger().info(
            f'=== ALL DISTANCES ===\n'
            f'  Stereo Depth:  {stereo:.2f}m\n'
            f'  Focal Length:  {focal:.2f}m\n'
            f'  Calibration:   {calib:.2f}m\n'
            f'  Hybrid:        {hybrid:.2f}m\n'
            f'  Confidence:    {confidence:.3f}\n'
            f'  YOLO Conf:     {yolo_conf:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisherTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
