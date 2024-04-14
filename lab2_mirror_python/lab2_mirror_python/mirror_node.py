import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState

class MirrorNode(Node):
	def __init__(self):
		super().__init__('mirror_node')
		
		self.joint_positions = []
        # Defining subcriber
		self.create_subscription(JointState, "/robot/joint_states", self.get_state_callback, 10)
        
        # ~ # Defining publisher
        # ~ self.publisher_ = self.create_publisher(String, 'world_greeting', 10)
        # ~ self.timer = self.create_timer(5, self.timer_callback)
    
    # Subscriber callback
	def get_state_callback(self, msg):
		self.joint_positions = msg.position
		self.get_logger().info('Received joint positions:')
		for i, name in enumerate(msg.name):
			print(f"  Joint {name}: {self.joint_positions[i]}")
        
	# ~ # Publisher callback
    # ~ def timer_callback(self):
        # ~ msg = String()
        # ~ msg.data = 'Hello World'
        # ~ self.publisher_.publish(msg)
        # ~ self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
	rclpy.init(args=args)

	mirror_node = MirrorNode()    
	rclpy.spin(mirror_node)

	rclpy.shutdown()

if __name__ == '__main__':
    main()
