// ROS
#include <ros/ros.h>
#include <ros/console.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.hpp>

#include <sirius_msgs/JointState.h>

namespace roboclaw
{
class RoboclawHWInterface : public hardware_interface::RobotHW
{
  public:
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time&, const ros::Duration&);
    void write(const ros::Time&, const ros::Duration&);

  protected:
    ros::NodeHandle nh_;
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    std::string motor1;
    std::string motor2;
    int number_of_motors;
    std::vector<int> roboclaw_motors;
    realtime_tools::RealtimePublisher<sirius_msgs::JointState> publisher[2];
    std::vector<sirius_msgs::JointState> joint_state;
    std::vector<double> target_velocity;
    void motor1_callback(const sirius_msgs::JointState::ConstPtr& msg);
    void motor2_callback(const sirius_msgs::JointState::ConstPtr& msg);
};

}  // namespace roboclaw
