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
		self.limb_names = ["_e0", "_e1", "_s0", "_s1", "_w0", "_w1", "_w2"]
        
        # Defining subcriber
		self.create_subscription(JointState, "/robot/joint_states", self.get_state_callback, 10)
        
        # Defining publisher
		self.publisher_ = self.create_publisher(JointCommand, "/robot/limb/left/joint_command", 10)
    
    # Subscriber callback
	def get_state_callback(self, msg):
		self.joint_positions = msg.position
		mirror_msg = JointCommand()
		mirror_msg.mode = 1  # Set to position mode
			
		for name in self.limb_names:
			# Prepare and publish left limb command message
			left_joint_name = "left" + name
			right_joint_name = "right" + name
			
			index = msg.name.index(right_joint_name)
			position = msg.position[index]
			
			mirror_msg.names.append(left_joint_name)
			if name in ["_s1", "_e1", "_w1"]:
				mirror_msg.command.append(position)
			else:
				mirror_msg.command.append(-position)

		self.publisher_.publish(mirror_msg)
		
		
def main(args=None):
	rclpy.init(args=args)

	mirror_node = MirrorNode()    
	rclpy.spin(mirror_node)

	rclpy.shutdown()

if __name__ == '__main__':
    main()
