// ROS
#include <control.h>

namespace roboclaw
{
bool RoboclawHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    if (robot_hw_nh.hasParam("motor1"))
    {
        robot_hw_nh.getParam("motor1", motor1);
        roboclaw_motors.push_back(1);
    }
    if (robot_hw_nh.hasParam("motor2"))
    {
        robot_hw_nh.getParam("motor2", motor2);
        roboclaw_motors.push_back(2);
    }
    number_of_motors = roboclaw_motors.size();
    robot_hw_nh.setParam("roboclaw/motors", roboclaw_motors);

    joint_state.resize(number_of_motors);
    target_velocity.resize(number_of_motors);
    for (int motor = 0; motor < number_of_motors; motor++)
    {
        std::string joint;
        std::string topic;
        if (roboclaw_motors[motor] == 1)
        {
            joint = motor1;
            publisher[motor].init(robot_hw_nh, "roboclaw/motor1/set_joint_state", 1);
            robot_hw_nh.subscribe("roboclaw/motor1/joint_state", 1, &RoboclawHWInterface::motor1_callback, this);
        }
        else
        {
            joint = motor2;
            publisher[motor].init(robot_hw_nh, "roboclaw/motor2/set_joint_state", 1);
            robot_hw_nh.subscribe("roboclaw/motor2/joint_state", 1, &RoboclawHWInterface::motor2_callback, this);
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
        hardware_interface::JointStateHandle state_handle(joint,
                                                          &(joint_state[motor].position[0]),
                                                          &(joint_state[motor].velocity[0]),
                                                          &(joint_state[motor].effort[0]));
        joint_state_interface.registerHandle(state_handle);
        // Velocity
        hardware_interface::JointHandle velocity_handle(state_handle, &target_velocity[motor]);
        velocity_joint_interface.registerHandle(velocity_handle);
    }
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);

    return true;
};

void RoboclawHWInterface::motor1_callback(const sirius_msgs::JointState::ConstPtr& msg)
{
    joint_state[0] = *msg;
};
void RoboclawHWInterface::motor2_callback(const sirius_msgs::JointState::ConstPtr& msg)
{
    joint_state[number_of_motors - 1] = *msg;
};
void RoboclawHWInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/){};
void RoboclawHWInterface::write(const ros::Time& time, const ros::Duration& /*period*/)
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
};  // namespace roboclaw

PLUGINLIB_EXPORT_CLASS(roboclaw::RoboclawHWInterface, hardware_interface::RobotHW)