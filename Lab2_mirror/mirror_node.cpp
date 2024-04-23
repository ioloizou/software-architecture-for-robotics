// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>

// access time units such as 100ms
using namespace std::chrono_literals;

// some shortcuts for message classes
using sensor_msgs::msg::JointState;
using baxter_core_msgs::msg::JointCommand;

// a useful function to get the index of a string in a vector of strings
// returns the size of the vector if not found
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
  const auto elem = std::find(names.begin(), names.end(), name);
  return std::distance(names.begin(), elem);
}

namespace lab2_mirror
{

class MirrorNode : public rclcpp::Node
{
public:
  MirrorNode(rclcpp::NodeOptions options) : Node("mirror", options)
  {
    // init whatever is needed for your node
    // these suffixes may be useful
    const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};
    
    // init command message
    left_command_msg_.set__mode(1);
    left_command_msg_.command.resize(suffixes.size(), 0);   
    for (auto &suffix: suffixes)
    {
      std::string name = "left" + suffix;
      left_command_msg_.names.push_back(name);
    }
    // init subscriber
    joint_states_subscriber_ = this->create_subscription<JointState>("robot/joint_states", 10, std::bind(&MirrorNode::joint_state_callback, this, std::placeholders::_1));
    // init publisher
    right_command_publisher_ = this->create_publisher<JointCommand>("robot/limb/left/joint_command",10);
    
  }
  
private:
  void joint_state_callback(const JointState::SharedPtr joint_state_msg_)
  {
    std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

    for (size_t i=0; i<suffixes.size(); i++)
    {
      std::string name = "right" + suffixes[i];
      int index = findIndex(name, joint_state_msg_->name);

      if ((suffixes[i]== "_s1") || (suffixes[i]== "_e1") ||(suffixes[i]== "_w1")){
        left_command_msg_.command.at(i) = joint_state_msg_->position.at(index);
      }
      else{
        left_command_msg_.command.at(i) = -joint_state_msg_->position.at(index);
      }
    }

    right_command_publisher_->publish(left_command_msg_);
  }
  // declare any subscriber / publisher / member variables and functions
  rclcpp::Publisher<JointCommand>::SharedPtr right_command_publisher_;
  rclcpp::Subscription<JointState>::SharedPtr joint_states_subscriber_;
  JointCommand left_command_msg_;
};

}

// boilerplate main function

int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lab2_mirror::MirrorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

// ros2 interface show baxter_core_msgs/msg/JointCommand
// int32 mode
// float64[] command
// string[] names

// int32 POSITION_MODE =1
// int32 VELOCITY_MODE =2
// int32 TORQUE_MODE =3
// int32 RAW_POSITION_MODE =4

// ros2 interface show sensor_msgs/msg/JointState
// # This is a message that holds data to describe the state of a set of torque controlled joints.
// #
// # The state of each joint (revolute or prismatic) is defined by:
// #  * the position of the joint (rad or m),
// #  * the velocity of the joint (rad/s or m/s) and
// #  * the effort that is applied in the joint (Nm or N).
// #
// # Each joint is uniquely identified by its name
// # The header specifies the time at which the joint states were recorded. All the joint states
// # in one message have to be recorded at the same time.
// #
// # This message consists of a multiple arrays, one for each part of the joint state.
// # The goal is to make each of the fields optional. When e.g. your joints have no
// # effort associated with them, you can leave the effort array empty.
// #
// # All arrays in this message should have the same size, or be empty.
// # This is the only way to uniquely associate the joint name with the correct
// # states.

// std_msgs/Header header
// 	builtin_interfaces/Time stamp
// 		int32 sec
// 		uint32 nanosec
// 	string frame_id

// string[] name
// float64[] position
// float64[] velocity
// float64[] effort
