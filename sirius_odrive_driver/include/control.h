// ROS
#include <ros/ros.h>
#include <ros/console.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/JointState.h>

namespace sirius
{
class OdriveHWInterface : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  void read(const ros::Time&, const ros::Duration&);
  void write(const ros::Time&, const ros::Duration&);

protected:
  ros::NodeHandle nh_;
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  std::string motor0;
  std::string motor1;
  int number_of_motors;
  std::vector<int> odrive_motors;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher[2];
  std::vector<sensor_msgs::JointState> joint_state;
  std::vector<double> target_velocity;
  void motor0_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void motor1_callback(const sensor_msgs::JointState::ConstPtr& msg);
};

}  // namespace sirius
