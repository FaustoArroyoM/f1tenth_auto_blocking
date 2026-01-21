#!/usr/bin/env python3
"""
ROS2 Node for car distance estimation using multiple methods.
Subscribes to YOLO detections and ZED-2 depth, publishes distance estimates.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
from datetime import datetime
import json

# Message types
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Float32MultiArray
from geometry_msgs.msg import Point

# Import the distance calculator
from car_distance_estimator import (
    CarDistanceCalculator,
    DistanceResult
)


class CarDistanceNode(Node):
    """ROS2 Node for multi-method car distance estimation."""
    
    def __init__(self):
        super().__init__('car_distance_estimator_node')
        
        # Initialize parameters
        self._init_parameters()
        
        # Create distance calculator
        self.calculator = CarDistanceCalculator.from_ros_node(self)
        
        # Load calibration data if provided
        if self.calibration_data_path:
            self._load_calibration_data()
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # ========== SUBSCRIPTIONS ==========
        
        # TODO Subscribe to YOLO detections (as JSON string)
        self.yolo_subscription = self.create_subscription(  
            String,
            self.yolo_topic,
            self._yolo_callback,
            qos_profile
        )
        
        # TODO Subscribe to ZED-2 depth map
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos_profile
        )
        
        # Store latest depth map
        self.latest_depth_map = None
        self.latest_depth_timestamp = None
        
        # ========== PUBLISHERS ==========
        
        # Publish individual distances (Float32 for each method)
        self.pub_stereo_depth = self.create_publisher(
            Float32,
            'car_distance/stereo_depth',
            10
        )
        
        self.pub_focal_length = self.create_publisher(
            Float32,
            'car_distance/focal_length',
            10
        )
        
        self.pub_calibration = self.create_publisher(
            Float32,
            'car_distance/calibration',
            10
        )
        
        self.pub_hybrid = self.create_publisher(
            Float32,
            'car_distance/hybrid',
            10
        )
        
        # Publish all distances together (Float32MultiArray)
        self.pub_all_distances = self.create_publisher(
            Float32MultiArray,
            'car_distance/all_distances',
            10
        )
        
        # Publish confidence score
        self.pub_confidence = self.create_publisher(
            Float32,
            'car_distance/confidence',
            10
        )
        
        # Publish diagnostics/debug info
        self.pub_debug = self.create_publisher(
            String,
            'car_distance/debug',
            10
        )
        
        self.get_logger().info('Car Distance Estimator Node initialized')
    
    #TODO Change this here
    def _init_parameters(self):
        """Declare and get ROS2 parameters."""
        
        # Camera parameters
        self.declare_parameter('sensor_width_px', 2688)
        self.sensor_width_px = self.get_parameter('sensor_width_px').value
        
        self.declare_parameter('sensor_width_mm', 6.912)
        self.sensor_width_mm = self.get_parameter('sensor_width_mm').value
        
        self.declare_parameter('focal_length_mm', 2.12)
        self.focal_length_mm = self.get_parameter('focal_length_mm').value
        
        self.declare_parameter('known_car_height', 0.20)
        self.known_car_height = self.get_parameter('known_car_height').value
        
        # Topic names
        self.declare_parameter('yolo_topic', '/yolo/detections')
        self.yolo_topic = self.get_parameter('yolo_topic').value
        
        self.declare_parameter('depth_topic', '/zed2/depth/depth_registered')
        self.depth_topic = self.get_parameter('depth_topic').value
        
        # Calibration
        self.declare_parameter('calibration_data_path', '')
        self.calibration_data_path = self.get_parameter('calibration_data_path').value
        
        # Enable/disable methods
        self.declare_parameter('enable_stereo_depth', True)
        self.enable_stereo_depth = self.get_parameter('enable_stereo_depth').value
        
        self.declare_parameter('enable_focal_length', True)
        self.enable_focal_length = self.get_parameter('enable_focal_length').value
        
        self.declare_parameter('enable_calibration', True)
        self.enable_calibration = self.get_parameter('enable_calibration').value
        
        self.declare_parameter('enable_hybrid', True)
        self.enable_hybrid = self.get_parameter('enable_hybrid').value
        
        # Debug mode
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info(f'Parameters loaded:')
        self.get_logger().info(f'  Camera: focal_length={self.focal_length_mm}mm')
        self.get_logger().info(f'  Known object height: {self.known_car_height}m')
        self.get_logger().info(f'  YOLO topic: {self.yolo_topic}')
        self.get_logger().info(f'  Depth topic: {self.depth_topic}')
    
    def _load_calibration_data(self):
        """Load calibration data from JSON file."""
        try:
            with open(self.calibration_data_path, 'r') as f:
                calib_data = json.load(f)
                self.calculator.set_calibration_data(calib_data)
                self.get_logger().info(
                    f'Calibration data loaded from {self.calibration_data_path} '
                    f'({len(calib_data)} points)'
                )
        except Exception as e:
            self.get_logger().error(
                f'Failed to load calibration data: {e}'
            )
    
    def _yolo_callback(self, msg: String):  # TODO check this with the real hardware
        """
        Callback for YOLO detections.
        Expected format: JSON string with bounding boxes.
        
        Example:
        {
            "detections": [
                {"class": "car", "x1": 100, "y1": 200, "x2": 300, "y2": 400, "conf": 0.95}
            ],
            "timestamp": 1234567890.123
        }
        """
        try:
            detections = json.loads(msg.data)
            timestamp = detections.get('timestamp', 0.0)
            
            for det in detections.get('detections', []):
                if det.get('class') != 'car':
                    continue
                
                # Extract bounding box
                bbox = (
                    det.get('x1', 0),
                    det.get('y1', 0),
                    det.get('x2', 0),
                    det.get('y2', 0)
                )
                
                confidence = det.get('conf', 0.0)
                
                # Calculate distances
                self._process_detection(bbox, timestamp, confidence)
        
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse YOLO detections JSON')
        except Exception as e:
            self.get_logger().error(f'Error in YOLO callback: {e}')
    
    def _depth_callback(self, msg: Image):
        """Callback for depth map from ZED-2."""
        try:
            self.latest_depth_map = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
            self.latest_depth_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def _process_detection(self, bbox: tuple, timestamp: float, yolo_conf: float):
        """
        Process a single car detection and estimate distance.
        
        Args:
            bbox: (x1, y1, x2, y2) bounding box coordinates
            timestamp: Detection timestamp
            yolo_conf: YOLO confidence score
        """
        try:
            # Calculate all distances
            result = self.calculator.calculate_all_distances(
                bbox,
                self.latest_depth_map,
                timestamp
            )
            
            # Publish individual results
            self._publish_results(result, yolo_conf)
            
            # Debug logging
            if self.debug_mode:
                self._log_debug_info(result, bbox, yolo_conf)
        
        except Exception as e:
            self.get_logger().error(f'Error processing detection: {e}')
    
    def _publish_results(self, result: DistanceResult, yolo_conf: float):
        """Publish distance estimation results to ROS2 topics."""
        
        # Publish individual distances
        if result.stereo_depth is not None and self.enable_stereo_depth:
            msg = Float32()
            msg.data = result.stereo_depth
            self.pub_stereo_depth.publish(msg)
        
        if result.focal_length is not None and self.enable_focal_length:
            msg = Float32()
            msg.data = result.focal_length
            self.pub_focal_length.publish(msg)
        
        if result.calibration is not None and self.enable_calibration:
            msg = Float32()
            msg.data = result.calibration
            self.pub_calibration.publish(msg)
        
        if result.hybrid is not None and self.enable_hybrid:
            msg = Float32()
            msg.data = result.hybrid
            self.pub_hybrid.publish(msg)
        
        # Publish all distances as array
        msg_array = Float32MultiArray()
        msg_array.data = [
            result.stereo_depth or -1.0,
            result.focal_length or -1.0,
            result.calibration or -1.0,
            result.hybrid or -1.0,
            yolo_conf,
            result.confidence
        ]
        self.pub_all_distances.publish(msg_array)
        
        # Publish confidence
        msg_conf = Float32()
        msg_conf.data = result.confidence
        self.pub_confidence.publish(msg_conf)
    
    def _log_debug_info(self, result: DistanceResult, bbox: tuple, yolo_conf: float):
        """Publish debug information."""
        debug_info = {
            'timestamp': datetime.now().isoformat(),
            'bbox': bbox,
            'yolo_confidence': yolo_conf,
            'distances': result.to_dict()
        }
        
        msg = String()
        msg.data = json.dumps(debug_info)
        self.pub_debug.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarDistanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
