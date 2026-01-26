"""Core race tracking logic with YOLO + ZED-2."""

import numpy as np
np.bool = bool  # Fix TensorRT/Numpy compatibility

try:
    import pyzed.sl as sl
    HAS_PYZED = True
except ImportError:
    HAS_PYZED = False
    sl = None


# FLAGS
DEBUG_MODE = True

# import pyzed.sl as sl
import cv2
from ultralytics import YOLO
from dataclasses import dataclass
from typing import List, Optional, Tuple
import os


@dataclass
class DetectionResult:
    """Single detection result."""
    track_id: int
    bbox_2d: np.ndarray  # [x1, y1, x2, y2]
    distance_from_bb: float  # Focal length method
    distance_from_stereo: float  # Stereo depth method
    bbox_center: float # center of bounding box 
    confidence: float
    position_3d: np.ndarray  # [x, y, z] from ZED
    velocity_z: float


class RaceTracker:
    """Core tracking pipeline with dual distance methods."""
    
    def __init__(self,
                 yolo_model_path: str,
                 vehicle_height: float,
                 focal_length: float,
                 offset_bb_measure: float,
                 offset_stereo_cam: float,
                 yolo_conf_threshold: float = 0.4,
                 yolo_imgsz: int = 640):
        """
        Initialize tracker with parameters from YAML.
        
        Args:
            yolo_model_path: Path to YOLO model (.engine, .pt, etc.)
            vehicle_height: Real height of target vehicle in meters
            focal_length: Camera focal length in pixels
            offset_bb_measure: Offset for BB method in meters
            offset_stereo_cam: Offset for stereo method in meters
            yolo_conf_threshold: YOLO confidence threshold
            yolo_imgsz: YOLO inference image size
        """
        self.vehicle_height = vehicle_height
        self.focal_length = focal_length
        self.offset_bb_measure = offset_bb_measure
        self.offset_stereo_cam = offset_stereo_cam
        self.yolo_conf_threshold = yolo_conf_threshold
        self.yolo_imgsz = yolo_imgsz
        
        
        # Handle ROS2 package URI       
        from ament_index_python.packages import get_package_share_directory

        if yolo_model_path.startswith("package://"):
            # Example: package://car_distance_estimator/models/best.engine
            parts = yolo_model_path.replace("package://", "").split("/", 1)
            if len(parts) != 2:
                raise ValueError(f"Invalid package URL for yolo_model_path: {yolo_model_path}")
            package_name, relative_path = parts
            package_share = get_package_share_directory(package_name)
            model_path = os.path.join(package_share, relative_path)
        else:
            model_path = yolo_model_path


        self.zed = None

        # Load YOLO model
        self.model = YOLO("/home/f1tenth/f1tenth_auto_blocking/lab_ws25/install/car_distance_estimator/share/car_distance_estimator/models/best.engine", task="detect")
        
        # Initialize ZED camera
        self._init_zed()
    
    def _init_zed(self):
        """Initialize ZED-2 camera."""

        if not HAS_PYZED:
            self.get_logger().warning("PyZED not available - test mode only!")
            self.zed = None
            return
        
        self.zed = sl.Camera()
        
        # Camera configuration
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")
        
        self.zed.enable_positional_tracking(sl.PositionalTrackingParameters())
        
        # Object detection setup
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        
        err = self.zed.enable_object_detection(obj_param)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to enable object detection: {err}")
    
    def process_frame(self) -> Tuple[np.ndarray, List[DetectionResult], bool]:
        """
        Process one frame: grab from ZED, run YOLO, calculate distances.
        
        Returns:
            (frame, detections, success) where success=True if frame grabbed
        """
        # Grab frame from ZED
        runtime_params = sl.RuntimeParameters()
        err = self.zed.grab(runtime_params)
        
        if err != sl.ERROR_CODE.SUCCESS:
            return None, [], False
        
        # Get left camera image
        image_zed = sl.Mat()
        objects = sl.Objects()

        self.zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        frame = image_zed.get_data()[:, :, :3]  # BGR for OpenCV
        render_frame = frame.copy()
        
        # Run YOLO inference
        results = self.model.predict(source=frame, conf=self.yolo_conf_threshold,
                                    imgsz=self.yolo_imgsz, verbose=False)

        # Process detections
        detections = []
        for result in results:
            for box in result.boxes:
                bbox = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = bbox

                # Calculate distances using two methods
                bbox_width_px = x2 - x1
                bbox_height_px = y2 - y1
                bbox_center = (x1+x2)*0.5
                # Method 1: Focal length / pinhole model
                distance_from_bb = (self.vehicle_height * self.focal_length / bbox_height_px) - self.offset_bb_measure

                # Draw YOLO 2D Box (Green)
                cv2.rectangle(render_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)

                # Show pixel dimensions in the image
                size_label = f"W:{bbox_width_px}px H:{bbox_height_px}px Dist_bb:{distance_from_bb:.2f}m"
                cv2.putText(render_frame, size_label, (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Create detection result (simplified - no ZED stereo yet)
                detection = DetectionResult(
                    track_id=len(detections),  # Simple ID
                    bbox_2d=bbox,
                    distance_from_bb=distance_from_bb,
                    distance_from_stereo=distance_from_bb,  # Use BB method as fallback
                    bbox_center=bbox_center,
                    confidence=float(box.conf[0]),
                    position_3d=np.array([0, 0, distance_from_bb]),
                    velocity_z=0.0
                )
                detections.append(detection)

                # Draw label
                label = f"ID:{detection.track_id} Dist:{distance_from_bb:.2f} bbox:{bbox_center:.2f}[Pixels]"
                cv2.putText(render_frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display Window
        # if DEBUG_MODE == True:
        cv2.imshow("F1TENTH Racing Monitor", render_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None, [], False

        return frame, detections, True

    
    
    def close(self):
        """Cleanup resources."""
        if self.zed is not None:
            self.zed.close()
    
    def __del__(self):
        """Ensure cleanup on deletion."""
        self.close()
