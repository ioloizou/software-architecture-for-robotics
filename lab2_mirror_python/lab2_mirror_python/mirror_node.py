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
		self.left_limb_names = ["left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2"]
        
        # Defining subcriber
		self.create_subscription(JointState, "/robot/joint_states", self.get_state_callback, 10)
        
        # Defining publisher
		self.publisher_ = self.create_publisher(JointCommand, "/robot/limb/left/joint_command", 10)
    
    # Subscriber callback
	def get_state_callback(self, msg):
		self.joint_positions = msg.position
		# ~ for i, name in enumerate(msg.name):
			# ~ print(f"  Joint {name}: {self.joint_positions[i]}")

		# Prepare and publish left limb command message
		left_limb_command = JointCommand()
		left_limb_command.mode = 1  # Set to position mode
		left_limb_command.names = self.left_limb_names
		left_limb_command.command = self.joint_positions # I need to put them in a specific position in the array
		self.publisher_.publish(left_limb_command)

def main(args=None):
	rclpy.init(args=args)

	mirror_node = MirrorNode()    
	rclpy.spin(mirror_node)

	rclpy.shutdown()

if __name__ == '__main__':
    main()
