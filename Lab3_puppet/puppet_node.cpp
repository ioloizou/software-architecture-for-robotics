// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ik_client.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;
using baxter_core_msgs::msg::JointCommand;
using baxter_core_msgs::srv::SolvePositionIK;

namespace lab3_puppet
{

class PuppetNode : public rclcpp::Node
{
public:
  PuppetNode(rclcpp::NodeOptions options) : Node("puppet", options)
  {
    // init whatever is needed for your node

    // init command message for left arm

    // init publisher to left arm command
    publisher_ = this->create_publisher<JointCommand>("/robot/limb/left/joint_command", 10);

    // init timer - the function publishCommand() should called with the given rate
    timer_ = this->create_wall_timer(20ms, std::bind(&PuppetNode::publishCommand, this));

    // IK service wrapper into IKNode
    ik_node.init("ik_node","/ExternalTools/left/PositionKinematicsNode/IKService");
  }

private:

  // declare member variables for command publisher and timer

  rclcpp::Publisher<JointCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  ServiceNodeSync<SolvePositionIK> ik_node;

  // TF 2 stuff
  tf2_ros::Buffer tf_buffer{get_clock()};                // stores all previous elementary transforms in a tree
  tf2_ros::TransformListener tf_listener{tf_buffer};   // subscribes to /tf

  void publishCommand()
  {
    JointCommand puppet_msg;

    geometry_msgs::msg::TransformStamped tf_left_gripper_to_base;


    // check if the transform from base to left_gripper_desired is available
    if(tf_buffer.canTransform("left_gripper_desired", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
      // get this transform with tf_buffer.lookupTransform("base", "left_gripper_desired", ...
      tf_left_gripper_to_base = tf_buffer.lookupTransform("base", "left_gripper_desired", tf2::TimePointZero);

      // build service request SolvePositionIK::Request from obtained transform
      SolvePositionIK::Request req;

      // Create a PoseStamped message to hold the desired pose
      geometry_msgs::msg::PoseStamped desired_pose;
      desired_pose.header = tf_left_gripper_to_base.header; // Set the desired pose frame

      // Get position from the transform
      desired_pose.pose.position.x = tf_left_gripper_to_base.transform.translation.x;
      desired_pose.pose.position.y = tf_left_gripper_to_base.transform.translation.y;
      desired_pose.pose.position.z = tf_left_gripper_to_base.transform.translation.z;
      desired_pose.pose.orientation.x = tf_left_gripper_to_base.transform.rotation.x;
      desired_pose.pose.orientation.y = tf_left_gripper_to_base.transform.rotation.y;
      desired_pose.pose.orientation.z = tf_left_gripper_to_base.transform.rotation.z;
      desired_pose.pose.orientation.w = tf_left_gripper_to_base.transform.rotation.w;

      // Add it to request
      req.pose_stamp.push_back(desired_pose);

//      std::cout<<"My x is : "<<tf_left_gripper_to_base.transform.translation.x<<std::endl;

      // call service and get response
      if(SolvePositionIK::Response res; ik_node.call(req, res))
      {
        // call to IK was successfull, check if the solution is valid
        if (res.result_type[0]!=0)
        {
            // copy response data to joint command and publish to left arm
            puppet_msg.mode = 1;

            sensor_msgs::msg::JointState joint = res.joints[0];
            for (size_t i=0; i<joint.name.size(); i++)
            {
                puppet_msg.names.push_back(joint.name[i]);
                puppet_msg.command.push_back(joint.position[i]);
            }
            publisher_->publish(puppet_msg);
        }
      }
    }
  }
};
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)


//ros2 interface show baxter_core_msgs/srv/SolvePositionIK
//# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.
//geometry_msgs/PoseStamped[] pose_stamp
//	std_msgs/Header header
//		builtin_interfaces/Time stamp
//			int32 sec
//			uint32 nanosec
//		string frame_id
//	Pose pose
//		Point position
//			float64 x
//			float64 y
//			float64 z
//		Quaternion orientation
//			float64 x 0
//			float64 y 0
//			float64 z 0
//			float64 w 1

//# (optional) Joint Angle Seed(s) for IK solver.
//# * specify a JointState seed for each pose_stamp, using name[] and position[]
//# * empty arrays or a non-default seed_mode will cause user seed to not be used
//sensor_msgs/JointState[] seed_angles
//	#
//	#
//	#
//	#
//	std_msgs/Header header
//		builtin_interfaces/Time stamp
//			int32 sec
//			uint32 nanosec
//		string frame_id
//	string[] name
//	float64[] position
//	float64[] velocity
//	float64[] effort

//# Seed Type Mode
//# * default (SEED_AUTO) mode: iterate through seed types until first valid
//#                             solution is found
//# * setting any other mode:   try only that seed type
//uint8 SEED_AUTO = 0
//uint8 SEED_USER = 1
//uint8 SEED_CURRENT = 2
//uint8 SEED_NS_MAP = 3

//uint8 seed_mode
//---
//# joints[i]      == joint angle solution for each pose_state[i]
//sensor_msgs/JointState[] joints
//	#
//	#
//	#
//	#
//	std_msgs/Header header
//		builtin_interfaces/Time stamp
//			int32 sec
//			uint32 nanosec
//		string frame_id
//	string[] name
//	float64[] position
//	float64[] velocity
//	float64[] effort

//# NOTE: isValid will be deprecated by result_type in future versions
//bool[] is_valid

//# result_type[i] == seed type used to find valid solution, joints[i];
//# otherwise,     == RESULT_INVALID (no valid solution found).
//uint8 RESULT_INVALID = 0
//uint8[] result_type



//-----------------------------

//ros2 interface show baxter_core_msgs/msg/JointCommand
//int32 mode
//float64[] command
//string[] names

//int32 POSITION_MODE =1
//int32 VELOCITY_MODE =2
//int32 TORQUE_MODE =3
//int32 RAW_POSITION_MODE =4

