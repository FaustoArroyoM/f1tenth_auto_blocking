import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class Safety(Node):
    def __init__(self):
        # Initialize this node with the name 'safety_node'
        super().__init__('safety_node')
        
        # Initialize the car's velocity to 0.0
        self.v_car = 0.0
        
        # Task 2: Initialize the emergency brake status. '0' means brake is not activated.
        self.emergency_brake = 0

        # Subscribe to the LaserScan topic "/scan". When a message is received, 'scan_callback' will be executed.
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        
        # Task 1: Subscribe to the Odometry topic "/odom". When a message is received, 'odom_callback' will be executed.
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        # Publisher to publish messages of type AckermannDriveStamped to the topic "/drive".
        self.brake_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

    # Callback function for the odometry subscription
    def odom_callback(self, odom_msg):
        # Extract the linear velocity of the car from the odometry message and store it
        self.v_car = odom_msg.twist.twist.linear.x

    # Callback function for the laser scan subscription
    def scan_callback(self, scan_msg):
        # Convert the list of range measurements from the LaserScan message to a numpy array
        ranges = np.array(scan_msg.ranges)
        
        # Calculate the angles corresponding to each range measurement using the angular properties from the LaserScan message
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))

        # Define a threshold for Time To Collision (TTC)
        ttc_threshold = 0.7
    
        
        # Calculate TTC for each valid range reading
        TTC = [rng / max(self.v_car * np.cos(angle), 0.0001) for rng, angle in zip(ranges, angles) if not np.isnan(rng) and rng > 0.0]

        # Determine the minimum TTC from the list
        min_ttc = min(TTC, default=float('inf'))

        # Determine if an emergency brake should be applied based on the minimum TTC
        should_brake = min_ttc < ttc_threshold
        
        # Create a new AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()

        # Task 2: Decide the action based on whether braking is necessary or not
        if should_brake:
            # Log that the emergency brake has been activated
            self.get_logger().info("Emergency brake activated")
            # Set the speed in the AckermannDriveStamped message to 0.0 to stop the car
            ackermann_msg.drive.speed = 0.0
            # Set the emergency brake status to '1' indicating it's activated
            self.emergency_brake = 1
        else:
            # If emergency brake is not active, set the speed to 2.0 m/s
            if not self.emergency_brake:
                ackermann_msg.drive.speed = 2.0

        # Publish the AckermannDriveStamped message, which either commands the car to stop or move at 2 m/s
        self.brake_pub.publish(ackermann_msg)

        # If the minimum TTC is less than 2 seconds, log the value for debugging purposes
        if min_ttc < 2:
            self.get_logger().info(f"Minimum TTC: {min_ttc}")


def main(args=None):
    rclpy.init(args=args)
    node = Safety()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
