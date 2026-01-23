import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

class RacelineManagerNode(Node):

    def __init__(self):
        super().__init__('raceline_manager_node')

        self._init_parameters()


        self.subscription = self.create_subscription(Float32,'/race_tracker/bbox_center',  self._bbox_center_callback, 10)
        self.subscription = self.create_subscription(Float32,'/race_tracker/distance_car',  self._distance_car_callback, 10)

        self.pub_raceline_ID = self.create_publisher(Int32, '/active_raceline', 10)
        self.timer = self.create_timer(0.033, self._process_loop)  # ~30 Hz



    def _init_parameters(self):
        # Declare default raceline ID (0: Inner line, 1: Middle line, 2: Outer line)
        self.declare_parameter('raceline_ID', 1)
        self.raceline_ID = self.get_parameter('raceline_ID').value

        self.declare_parameter('car_detection', False)
        self.car_detection = self.get_parameter('car_detection').value
        
        self.declare_parameter('bbox_center', 1)
        self.bbox_center = self.get_parameter('bbox_center').value
        
                
        self.declare_parameter('bbox_center_buffer', 0.15)
        self.bbox_center_buffer = self.get_parameter('bbox_center_buffer').value

        self.declare_parameter('image_width', 1280)
        self.image_width = self.get_parameter('image_width').value


    def _process_loop(self):
        
        # Normalize bbox center position and decide raceline ID based on buffer
        bbox_center_norm = self.bbox_center / self.image_width  
        
        if bbox_center_norm < 0.5 - self.bbox_center_buffer:
            raceline_ID = 0  # Inner line
        elif bbox_center_norm > 0.5 + self.bbox_center_buffer:
            raceline_ID = 2  # Outer line
        else:
            raceline_ID = 1  # Middle line

        if raceline_ID != self.raceline_ID:
            self.get_logger().info('Raceline ID changed from "%d" to "%d"' % (self.pub_raceline_ID, raceline_ID))
            self.raceline_ID = raceline_ID
            msg = Int32(self.raceline_ID)
            self.pub_raceline_ID.publish(msg)
            
        else:
            self.get_logger().info('Current raceline is ID: "%d"' % self.pub_raceline_ID)



    def _bbox_center_callback(self, msg):
        self.get_logger().info('Detection at bbox_center: "%f"' % msg.data)
        self.bbox_center = msg.data



    def _distance_car_callback(self, msg):
        self.get_logger().info('Detection at distance_car: "%f"' % msg.data)
        self.distance_car = msg.data





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