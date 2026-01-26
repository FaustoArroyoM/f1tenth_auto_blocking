# test_raceline_manager.py
import rclpy
from std_msgs.msg import Float32
import time
import threading

def publish_mock_data():
    """Publish fake bbox_center and distance_car values."""
    rclpy.init()
    node = rclpy.create_node('mock_publisher')
    
    pub_bbox = node.create_publisher(Float32, '/race_tracker/bbox_center', 10)
    pub_distance = node.create_publisher(Float32, '/race_tracker/distance_car', 10)
    
    # Simulate different scenarios
    scenarios = [
        # (bbox_center, distance_car, description)
        (200, 0.5, "Car on left lane, close"),
        (640, 0.5, "Car on middle lane, close"),
        (1100, 0.5, "Car on right lane, close"),
        (None, 5.0, "No car detected (far)"),
        (500, 0.8, "Car slightly left, close"),
    ]
    
    for bbox, dist, desc in scenarios:
        print(f"\n📍 Scenario: {desc}")
        
        for i in range(30):  # Publish for 1 second (30 Hz)
            if bbox is not None:
                msg_bbox = Float32(data=float(bbox))
                pub_bbox.publish(msg_bbox)
            
            msg_dist = Float32(data=float(dist))
            pub_distance.publish(msg_dist)
            
            rclpy.spin_once(node, timeout_sec=0.033)
            time.sleep(0.033)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_mock_data()
