"""Car Distance Estimator package for ROS2."""

# Export the main classes so they can be imported
from .race_tracker import RaceTracker, DetectionResult

__all__ = ['CarDistanceCalculator', 'DistanceResult', 'RaceTracker', 'DetectionResult']
__version__ = '0.0.1'