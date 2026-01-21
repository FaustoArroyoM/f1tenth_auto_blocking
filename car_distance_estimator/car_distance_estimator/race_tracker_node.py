#!/usr/bin/env python3
"""ROS2 node wrapping race_tracker.py."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import json
from cv_bridge import CvBridge

from .race_tracker import RaceTracker, DetectionResult


class RaceTrackerNode(Node):
    """ROS2 node for real-time race tracking."""
    
    def __init__(self):
        super().__init__('race_tracker_node')
        
        # Load YAML parameters
        self._init_parameters()
        
        # Initialize tracker with YAML parameters
        self.tracker = RaceTracker(
            yolo_model_path=self.yolo_model_path,
            vehicle_height=self.vehicle_height,
            focal_length=self.focal_length,
            offset_bb_measure=self.offset_bb_measure,
            offset_stereo_cam=self.offset_stereo_cam,
            yolo_conf_threshold=self.yolo_conf_threshold,
            yolo_imgsz=self.yolo_imgsz
        )
        
        # Publishers
        self.pub_distance_focal = self.create_publisher(Float32, '/race_tracker/distance_bb', 10)
        self.pub_distance_stereo = self.create_publisher(Float32, '/race_tracker/distance_stereo', 10)
        self.pub_distance = self.create_publisher(Float32, '/race_tracker/distance_car', 10)

        self.pub_all_detections = self.create_publisher(String, '/race_tracker/detections', 10)
        self.pub_frame = self.create_publisher(Image, '/race_tracker/frame_annotated', 10)
        
        # Timer for processing loop
        self.timer = self.create_timer(0.033, self._process_loop)  # ~30 Hz
        
        self.bridge = CvBridge()
        
        self.get_logger().info('RaceTrackerNode initialized')
        self.get_logger().info(f'  Vehicle height: {self.vehicle_height}m')
        self.get_logger().info(f'  Focal length: {self.focal_length}px')
        self.get_logger().info(f'  YOLO model: {self.yolo_model_path}')
    
    def _init_parameters(self):
        """Declare and load YAML parameters."""
        # YOLO config
        self.declare_parameter('yolo_model_path', '/path/to/best.engine')
        self.yolo_model_path = self.get_parameter('yolo_model_path').value
        
        self.declare_parameter('yolo_conf_threshold', 0.4)
        self.yolo_conf_threshold = self.get_parameter('yolo_conf_threshold').value
        
        self.declare_parameter('yolo_imgsz', 640)
        self.yolo_imgsz = self.get_parameter('yolo_imgsz').value
        
        # Camera/Vehicle config
        self.declare_parameter('vehicle_height', 0.2)
        self.vehicle_height = self.get_parameter('vehicle_height').value
        
        self.declare_parameter('focal_length', 528.15)
        self.focal_length = self.get_parameter('focal_length').value
        
        # Distance offsets
        self.declare_parameter('offset_bb_measure', 0.17)
        self.offset_bb_measure = self.get_parameter('offset_bb_measure').value
        
        self.declare_parameter('offset_stereo_cam', 0.2)
        self.offset_stereo_cam = self.get_parameter('offset_stereo_cam').value
        
        # Debug
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value
    
    def _process_loop(self):
        """Main processing loop: grab frame, process, publish."""
        try:
            frame, detections, success = self.tracker.process_frame()
            
            if not success:
                return
            
            # Publish individual distances for closest vehicle
            if detections:
                # Sort by stereo distance (most reliable)
                valid_detections = [d for d in detections if d.distance_from_stereo > 0]
                if valid_detections:
                    closest = min(valid_detections, key=lambda d: d.distance_from_bb)
                    
                    # Publish individual distances
                    msg_focal = Float32(data=closest.distance_from_bb)
                    self.pub_distance_focal.publish(msg_focal)
                    
                    msg_stereo = Float32(data=closest.distance_from_stereo)
                    self.pub_distance_stereo.publish(msg_stereo)
                    
                    
                    # Use focal if the car is closer, else stereo
                    if closest.distance_from_bb > 1.0:
                        msg_distance = Float32(data=closest.distance_from_stereo)
                    else:
                        msg_distance = Float32(data=closest.distance_from_bb)
                        
                    self.pub_distance.publish(msg_distance)
                    
                    if self.debug_mode:
                        self.get_logger().info(
                            f'ID={closest.track_id}: Distance={msg_distance.data:.2f}m,  Focal={closest.distance_from_bb:.2f}m, Stereo={closest.distance_from_stereo:.2f}m'
                        )
            
            # Publish all detections as JSON
            detections_json = self._detections_to_json(detections)
            msg_detections = String(data=detections_json)
            self.pub_all_detections.publish(msg_detections)
            
            # Publish annotated frame
            if frame is not None:
                frame_annotated = self._annotate_frame(frame, detections)
                msg_frame = self.bridge.cv2_to_imgmsg(frame_annotated, encoding='bgr8')
                self.pub_frame.publish(msg_frame)
        
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error in process loop: {e}')
            self.get_logger().error(traceback.format_exc())    

    def _detections_to_json(self, detections: list) -> str:
        """Convert detections to JSON string."""
        data = {
            'count': len(detections),
            'detections': [
                {
                    'track_id': d.track_id,
                    'bbox': [int(d.bbox_2d[0]), int(d.bbox_2d[1]), 
                             int(d.bbox_2d[2]), int(d.bbox_2d[3])],
                    'distance_bb': round(d.distance_from_bb, 2),
                    'distance_stereo': round(d.distance_from_stereo, 2),
                    'confidence': round(d.confidence, 2),
                }
                for d in detections
            ]
        }
        return json.dumps(data)
    
    def _annotate_frame(self, frame, detections):
        """Draw detections on frame."""
        annotated = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox_2d
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw text
            label = (f"ID:{detection.track_id} "
                    f"BB:{detection.distance_from_bb:.2f}m "
                    f"Stereo:{detection.distance_from_stereo:.2f}m")
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return annotated
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.tracker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RaceTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
