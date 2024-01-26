// ROS
#include <control.h>

namespace sirius
{
bool OdriveHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (robot_hw_nh.hasParam("motor0"))
  {
    robot_hw_nh.getParam("motor0", motor0);
    odrive_motors.push_back(1);
  }
  if (robot_hw_nh.hasParam("motor1"))
  {
    robot_hw_nh.getParam("motor1", motor1);
    odrive_motors.push_back(2);
  }
  number_of_motors = odrive_motors.size();
  robot_hw_nh.setParam("motors", odrive_motors);

  joint_state.resize(number_of_motors);
  target_velocity.resize(number_of_motors);
  for (int motor = 0; motor < number_of_motors; motor++)
  {
    std::string joint;
    std::string topic;
    if (odrive_motors[motor] == 1)
    {
      joint = motor0;
      publisher[motor].init(robot_hw_nh, "motor0/set_joint_state", 1);
      robot_hw_nh.subscribe("motor0/joint_state", 1, &OdriveHWInterface::motor0_callback, this);
    }
    else
    {
      joint = motor1;
      publisher[motor].init(robot_hw_nh, "motor1/set_joint_state", 1);
      robot_hw_nh.subscribe("motor1/joint_state", 1, &OdriveHWInterface::motor1_callback, this);
    }

    joint_state[motor].position.resize(1);
    joint_state[motor].velocity.resize(1);
    joint_state[motor].effort.resize(1);
    joint_state[motor].position[0] = 0;
    joint_state[motor].velocity[0] = 0;
    joint_state[motor].effort[0] = 0;

    publisher[motor].msg_.velocity.resize(1);
    target_velocity[motor] = 0;
    // State
    hardware_interface::JointStateHandle state_handle(
        joint, &(joint_state[motor].position[0]), &(joint_state[motor].velocity[0]), &(joint_state[motor].effort[0]));
    joint_state_interface.registerHandle(state_handle);
    // Velocity
    hardware_interface::JointHandle velocity_handle(state_handle, &target_velocity[motor]);
    velocity_joint_interface.registerHandle(velocity_handle);
  }
  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);

  return true;
};

void OdriveHWInterface::motor0_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state[0] = *msg;
};
void OdriveHWInterface::motor1_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state[number_of_motors - 1] = *msg;
};
void OdriveHWInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/){};
void OdriveHWInterface::write(const ros::Time& time, const ros::Duration& /*period*/)
{
  for (int motor = 0; motor < number_of_motors; motor++)
  {
    if (publisher[motor].trylock())
    {
      publisher[motor].msg_.velocity[0] = target_velocity[motor];
      publisher[motor].msg_.header.stamp = time;
      publisher[motor].unlockAndPublish();
    }
  }
};
};  // namespace sirius

PLUGINLIB_EXPORT_CLASS(sirius::OdriveHWInterface, hardware_interface::RobotHW)