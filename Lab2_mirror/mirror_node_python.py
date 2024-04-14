import rclpy
from rclpy.node import Node
import baxter_core_msgs.msg._joint_command
import sensor_msgs.msg._joint_state

def main(args=None):
    rclpy.init(args=args)
    node = Node('mirror_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()