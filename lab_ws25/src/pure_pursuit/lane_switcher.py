#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import termios
import tty

class LaneSwitcher(Node):
    def __init__(self):
        super().__init__('lane_switcher')
        self.publisher_ = self.create_publisher(Int32, '/active_raceline', 10)
        
        print("\n--- F1TENTH Lane Switcher ---")
        print("Press [0] for Inner Lane  (ID: 0)")
        print("Press [1] for Middle Lane (ID: 1)")
        print("Press [2] for Outer Lane  (ID: 2)")
        print("Press [q] to Quit")
        print("-----------------------------\n")

    def publish_lane(self, lane_id):
        msg = Int32()
        msg.data = lane_id
        self.publisher_.publish(msg)
        lane_name = ["INNER", "MIDDLE", "OUTER"][lane_id]
        self.get_logger().info(f'Published Lane Change: {lane_name} (ID: {lane_id})')

def get_key():
    """Reads a single keypress from the terminal without needing Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main(args=None):
    rclpy.init(args=args)
    node = LaneSwitcher()

    try:
        while rclpy.ok():
            key = get_key()
            if key == '0':
                node.publish_lane(0)
            elif key == '1':
                node.publish_lane(1)
            elif key == '2':
                node.publish_lane(2)
            elif key == 'q' or key == '\x03': # q or Ctrl+C
                break
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()