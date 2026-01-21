"""
Multi-method car distance calculator class.
Supports three approaches: stereo depth, focal length, and empirical calibration.
"""

import numpy as np
from scipy.interpolate import interp1d
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
import rclpy
from rclpy.node import Node


# try:
#     import pyzed.sl as sl
# except ImportError:
#     sl = None  # Fallback if pyzed not installed

@dataclass
class DistanceResult:
    """Container for distance estimation results from all three methods."""
    stereo_depth: Optional[float] = None
    focal_length: Optional[float] = None
    calibration: Optional[float] = None
    hybrid: Optional[float] = None
    confidence: float = 0.0 #TODO 
    timestamp: float = 0.0
    
    
    def to_dict(self) -> dict:
        """Convert to dictionary for easy logging/publishing."""
        return {
            'stereo_depth': self.stereo_depth,
            'focal_length': self.focal_length,
            'calibration': self.calibration,
            'hybrid': self.hybrid,
            'confidence': self.confidence,
            'timestamp': self.timestamp,
        }


class CarDistanceCalculator:
    """
    Multi-method distance calculator with PyZED integration.
    
    Attributes:
        sensor_width_px: Camera sensor width in pixels (
        sensor_width_mm: Physical sensor width in mm 
        focal_length_mm: Focal length in mm 
        known_car_height: Known height of target car in meters (0.20m)
        camera_matrix: OpenCV camera intrinsic matrix (optional, for distortion)
        dist_coeffs: Lens distortion coefficients (optional)
        calibration_func: Pre-trained empirical calibration function
    """
    
    #TODO change this to be loaded from the congig YAML fileS
    def __init__(self, 
                 sensor_width_px: int,
                 sensor_width_mm: float,
                 focal_length_mm: float,
                 known_car_height: float,
                 camera_matrix: Optional[np.ndarray] = None,
                 dist_coeffs: Optional[np.ndarray] = None):
        """
        Initialize the distance calculator with camera parameters.
        
        Args:
            sensor_width_px: Sensor width in pixels
            sensor_width_mm: Physical sensor width in mm
            focal_length_mm: Focal length in mm
            known_car_height: Known car height in meters
            camera_matrix: Camera intrinsic matrix (optional)
            dist_coeffs: Distortion coefficients (optional)
        """
        self.sensor_width_px = sensor_width_px
        self.sensor_width_mm = sensor_width_mm
        self.focal_length_mm = focal_length_mm
        self.known_car_height = known_car_height
        
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        # self.use_pyzed = use_pyzed
        
        # Calculate focal length in pixels
        px_per_mm = sensor_width_px / sensor_width_mm
        self.focal_length_px = focal_length_mm * px_per_mm
        
        # PyZED camera object (lazy initialized)
        # self.zed = None
        # if sl.use_pyzed and sl is not None:
        #     self._init_pyzed()
        
        # TODO Calibration function (will be set with set_calibration_data) --> Do this tomorrow and get even more examples maybe
        self.calibration_func = None
        
        
    @classmethod
    def from_ros_node(cls, ros_node: Node):
        """
        Factory method: Create calculator from ROS2 node parameters.
        
        Args:
            ros_node: Node that has already called declare_parameter()
        """
        return cls(
            sensor_width_px=ros_node.sensor_width_px,
            sensor_width_mm=ros_node.sensor_width_mm,
            focal_length_mm=ros_node.focal_length_mm,
            known_car_height=ros_node.known_car_height
        )
        
    # ==================== METHOD 1: STEREO DEPTH ====================
    
    def get_distance_from_stereo_depth(self,
                                       bbox: Tuple[float, float, float, float],
                                       depth_map: np.ndarray) -> Optional[float]:
        """
        Extract distance using ZED-2 native stereo depth.
        
        Args:
            bbox: (x1, y1, x2, y2) - YOLO bounding box coordinates
            depth_map: Depth map from ZED-2 (in meters), shape (height, width)
        
        Returns:
            Distance in meters, or None if invalid
        """
        x1, y1, x2, y2 = [int(v) for v in bbox]
        
        # Clamp to image bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(depth_map.shape - 1, x2)
        y2 = min(depth_map.shape - 1, y2)
        
        # Get center of bounding box
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        
        # Extract depth at center
        if 0 <= center_y < depth_map.shape and 0 <= center_x < depth_map.shape:
            distance = float(depth_map[center_y, center_x])
            
            # Filter invalid depth readings (ZED returns 0 or very large values)
            if 0.3 < distance < 20:
                return distance
        
        # Fallback: use median depth within bounding box for robustness
        if x2 > x1 and y2 > y1:
            bbox_depth = depth_map[y1:y2, x1:x2]
            valid_depths = bbox_depth[(bbox_depth > 0.3) & (bbox_depth < 20)]
            
            if len(valid_depths) > 0:
                return float(np.median(valid_depths))
        
        return None
    
    # ==================== METHOD 2: FOCAL LENGTH ====================
    
    def get_distance_from_focal_length(self,
                                       bbox: Tuple[float, float, float, float],
                                       apply_distortion_correction: bool = False) -> Optional[float]:
        """
        Calculate distance using pinhole camera model.
        
        Formula: distance = (focal_length * real_height) / image_height
        
        Args:
            bbox: (x1, y1, x2, y2) - YOLO bounding box
            apply_distortion_correction: Apply lens distortion correction before calculation
        
        Returns:
            Distance in meters, or None if invalid
        """
        x1, y1, x2, y2 = bbox
        bbox_height_px = y2 - y1
        
        if bbox_height_px <= 0:
            return None
        
        # Apply distortion correction if available
        if apply_distortion_correction and self.camera_matrix is not None:
            bbox = self._apply_camera_calibration(bbox)
            x1, y1, x2, y2 = bbox
            bbox_height_px = y2 - y1
        
        # Pinhole camera formula
        distance = (self.focal_length_px * self.known_car_height) / bbox_height_px
        
        # Sanity check
        if 0.3 < distance < 20:
            return distance
        
        return None
    
    def _apply_camera_calibration(self, bbox: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """Apply lens distortion correction to bounding box."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return bbox
        
        points = np.array([
            [bbox, bbox],
            [bbox, bbox]
        ], dtype=np.float32)
        
        # Undistort points
        undistorted = cv2.undistortPoints(
            points.reshape(-1, 1, 2),
            self.camera_matrix,
            self.dist_coeffs
        )
        
        undistorted = undistorted.reshape(-1)
        return tuple(undistorted)
    
    # ==================== METHOD 3: EMPIRICAL CALIBRATION ====================
    
    def set_calibration_data(self,
                            calibration_data: list,
                            interpolation_kind: str = 'cubic') -> None:
        """
        Set empirical calibration data from known distance/height measurements.
        
        Args:
            calibration_data: List of dicts with 'distance_m' and 'bbox_height_px'
            interpolation_kind: Type of interpolation ('linear', 'cubic', etc.)
        
        Example:
            calibration_data = [
                {'distance_m': 5.0, 'bbox_height_px': 120},
                {'distance_m': 10.0, 'bbox_height_px': 60},
                {'distance_m': 15.0, 'bbox_height_px': 40},
                {'distance_m': 20.0, 'bbox_height_px': 30},
            ]
            calculator.set_calibration_data(calibration_data)
        """
        distances = np.array([d['distance_m'] for d in calibration_data])
        heights = np.array([d['bbox_height_px'] for d in calibration_data])
        
        # Sort by height for interpolation
        sort_idx = np.argsort(heights)
        heights_sorted = heights[sort_idx]
        distances_sorted = distances[sort_idx]
        
        # Create interpolation function
        self.calibration_func = interp1d(
            heights_sorted,
            distances_sorted,
            kind=interpolation_kind,
            fill_value='extrapolate'
        )
    
    def get_distance_from_calibration(self,
                                      bbox: Tuple[float, float, float, float]) -> Optional[float]:
        """
        Use empirical calibration to estimate distance.
        
        Args:
            bbox: (x1, y1, x2, y2) YOLO bounding box
        
        Returns:
            Distance in meters, or None if calibration not set
        """
        if self.calibration_func is None:
            return None
        
        bbox_height_px = bbox[3] - bbox[1]
        
        if bbox_height_px <= 0:
            return None
        
        distance = float(self.calibration_func(bbox_height_px))
        
        # Clamp to reasonable range
        return float(np.clip(distance, 0.03, 20.0))
    
    # ==================== HYBRID METHOD ====================
    
    def estimate_distance_hybrid(self,
                                 bbox: Tuple[float, float, float, float],
                                 depth_map: Optional[np.ndarray] = None,
                                 use_all_methods: bool = True,
                                 stereo_weight: float = 0.6,
                                 focal_weight: float = 0.25,
                                 calibration_weight: float = 0.15) -> DistanceResult:
        """
        Use multiple methods with weighted averaging for robustness.
        
        Args:
            bbox: Bounding box coordinates
            depth_map: Depth map from ZED-2 (optional for method 1)
            use_all_methods: If False, use only available methods
            stereo_weight: Weight for stereo depth method
            focal_weight: Weight for focal length method
            calibration_weight: Weight for calibration method
        
        Returns:
            DistanceResult with all estimates and confidence
        """
        result = DistanceResult()
        methods_used = {}
        
        # Method 1: Stereo depth
        if depth_map is not None:
            stereo_dist = self.get_distance_from_stereo_depth(bbox, depth_map)
            if stereo_dist is not None:
                result.stereo_depth = stereo_dist
                methods_used['stereo'] = (stereo_dist, stereo_weight)
        
        # Method 2: Focal length
        focal_dist = self.get_distance_from_focal_length(bbox)
        if focal_dist is not None:
            result.focal_length = focal_dist
            methods_used['focal'] = (focal_dist, focal_weight)
        
        # Method 3: Calibration
        if self.calibration_func is not None:
            calib_dist = self.get_distance_from_calibration(bbox)
            if calib_dist is not None:
                result.calibration = calib_dist
                methods_used['calibration'] = (calib_dist, calibration_weight)
        
        # Compute weighted average (hybrid)
        if methods_used:
            # Unpack each (distance, weight) tuple correctly
            weighted_sum = sum(dist * weight for dist, weight in methods_used.values())
            weight_total = sum(weight for dist, weight in methods_used.values())

            if weight_total > 0.0:
                result.hybrid = weighted_sum / weight_total

            # Calculate confidence based on agreement between methods
            if len(methods_used) > 1:
                distances = [dist for dist, weight in methods_used.values()]
                variance = np.var(distances)
                result.confidence = 1.0 / (1.0 + variance)
            else:
                # Only one method used
                result.confidence = 0.7
        else:
            result.confidence = 0.0

        return result
        
    def calculate_all_distances(self,
                               bbox: Tuple[float, float, float, float],
                               depth_map: Optional[np.ndarray] = None,
                               timestamp: float = 0.0) -> DistanceResult:
        """
        Convenience method: calculate all three distances and hybrid estimate.
        
        Args:
            bbox: Bounding box coordinates
            depth_map: Depth map from ZED-2 (optional)
            timestamp: Timestamp for the measurement
        
        Returns:
            DistanceResult with all calculations
        """
        result = self.estimate_distance_hybrid(bbox, depth_map)
        result.timestamp = timestamp
        return result
