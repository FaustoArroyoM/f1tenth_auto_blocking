import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String



 # TODO do we want to change the dataclass for DetectionResult to have a success=Trur/False bool?? 
class RacelineManagerNode(Node):

    def __init__(self):
        super().__init__('raceline_manager_node')

        self._init_parameters()

        self.iterations_without_detection = 0        
        self.iterations_without_changing_ID = 0
        self.bbox_center = None
        self.distance_car = None    

        self.bbox_center_subscription = self.create_subscription(Float32,'/race_tracker/bbox_center',  self._bbox_center_callback, 10)
        self.distance_car_subscription = self.create_subscription(Float32,'/race_tracker/distance_car',  self._distance_car_callback, 10)

        self.pub_raceline_ID = self.create_publisher(Int32, '/active_raceline', 10)
        self.timer = self.create_timer(self.node_timer_rate, self._process_loop)  # TODO ~30 Hz
        
        



    def _init_parameters(self):
        
        self.declare_parameter('node_timer_rate', 0.033)
        self.node_timer_rate = self.get_parameter('node_timer_rate').value
        
        self.declare_parameter('blocking_distance_threshold', 1.0)
        self.blocking_distance_threshold = self.get_parameter('blocking_distance_threshold').value
        
        # Declare default raceline ID (0: Inner line, 1: Middle line, 2: Outer line)
        self.declare_parameter('raceline_ID', 1)
        self.raceline_ID = self.get_parameter('raceline_ID').value

        # Center buffer for middle (ideal) raceline        
        self.declare_parameter('bbox_center_buffer', 0.15)
        self.bbox_center_buffer = self.get_parameter('bbox_center_buffer').value
    
        self.declare_parameter('camera_image_width', 1280)
        self.image_width = self.get_parameter('camera_image_width').value

        self.declare_parameter('iterations_threshold_for_changing_ID', 5)
        self.iterations_threshold_for_changing_ID = self.get_parameter('iterations_threshold_for_changing_ID').value

        self.declare_parameter('iterations_before_defaulting_ID', 30)
        self.iterations_before_defaulting_ID = self.get_parameter('iterations_before_defaulting_ID').value


    def _bbox_center_callback(self, msg):
        self.get_logger().info('Detection at bbox_center: "%f"' % msg.data)
        self.bbox_center = msg.data



    def _distance_car_callback(self, msg):
        self.get_logger().info('Detection at distance_car: "%f"' % msg.data)
        self.distance_car = msg.data




    def _process_loop(self):
        """        
        1. Increment iteration counters to track detection and change cooldown states
        
        2. If car is detected within blocking distance threshold (< 1.0m):
        - Normalize bbox center X position to 0-1 range based on camera width
        - Check bbox center is valid
        - Determine target raceline based on normalized position:
            * bbox_center < 0.5 - buffer → raceline 2 (inner/left lane)
            * bbox_center > 0.5 + buffer → raceline 0 (outer/right lane)
            * else → raceline 1 (middle/center lane)
        - If target raceline differs from current AND change debounce threshold
            elapsed, update raceline and reset iteration counters
        - Otherwise keep current raceline
        
        3. If car not detected for 30 iterations (1 second at 30 Hz):
        - Default to middle raceline (ID=1) to prevent lane drift
        - Reset detection counter
        
        4. If car is beyond blocking distance threshold:
        - Keep current raceline without updating
        - Still default to middle after 30 iterations without detection
        
        Safeguards:
        - Change debouncing: Only allow lane changes every 5 iterations to prevent oscillation
        - Detection timeout: Auto-reset to middle lane if car disappears for 1 second
        - Validation: Check bbox_center is not None/0 before using it
        """

    
        self.iterations_without_detection += 1
        self.iterations_without_changing_ID += 1
        
        # Check if car is within distance threshold (1m) 
        if self.distance_car < self.blocking_distance_threshold and self.distance_car is not None:
            
            # Normalize bbox center position and decide raceline ID based on buffer
            bbox_center_norm = self.bbox_center / self.image_width  

            if self.bbox_center != 0 and self.bbox_center is not None: # TODO check tomorrow in the lab 

                if bbox_center_norm < 0.5 - self.bbox_center_buffer:
                    raceline_ID = 0  # Inner line
                elif bbox_center_norm > 0.5 + self.bbox_center_buffer:
                    raceline_ID = 2  # Outer line
                else:
                    raceline_ID = 1  # Middle line

                if raceline_ID != self.raceline_ID and self.iterations_without_changing_ID >= self.iterations_threshold_for_changing_ID:
                    self.get_logger().info('Raceline ID changed from "%d" to "%d"' % (self.raceline_ID, raceline_ID))
                    self.raceline_ID = raceline_ID
                    msg = Int32(data=self.raceline_ID)
                    self.pub_raceline_ID.publish(msg)
                    self.iterations_without_changing_ID = 0
                
                self.iterations_without_detection = 0
                
            else:
                self.get_logger().info('No detection, keeping raceline with ID: "%d"' % self.raceline_ID)
                
                # Default to middle raceline if no detection for 30 iterations
                if self.iterations_without_detection >= self.iterations_before_defaulting_ID:
                    self.raceline_ID = 1
                    msg = Int32(data=self.raceline_ID)
                    self.pub_raceline_ID.publish(msg)
                    self.get_logger().info('No detection for 30 iterations, defaulting to middle raceline with ID: "%d"' % self.raceline_ID)
                    self.iterations_without_detection = 0
                    
        else:
            self.get_logger().info('Car too far (distance: "%f"), keeping raceline with ID: "%d"' % (self.distance_car, self.raceline_ID))
            if self.iterations_without_detection >= self.iterations_before_defaulting_ID:
                if self.raceline_ID != 1 and self.iterations_without_changing_ID >= self.iterations_threshold_for_changing_ID:
                    self.raceline_ID = 1
                    msg = Int32(data=self.raceline_ID)
                    self.pub_raceline_ID.publish(msg)
                    self.iterations_without_changing_ID = 0

                self.get_logger().info('No detection for 30 iterations, defaulting to middle raceline with ID: "%d"' % self.raceline_ID)
                self.iterations_without_detection = 0
            
        






def main(args=None):
    rclpy.init(args=args)
    node = RacelineManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()