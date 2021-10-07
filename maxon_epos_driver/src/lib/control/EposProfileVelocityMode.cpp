/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:42:41
 */

#include "maxon_epos_driver/control/EposProfileVelocityMode.hpp"

EposProfileVelocityMode::~EposProfileVelocityMode()
{}

void EposProfileVelocityMode::init(ros::NodeHandle &motor_nh, NodeHandle &node_handle)
{
    ControlModeBase::init(motor_nh, node_handle);

    ros::NodeHandle velocity_position_nh(motor_nh, "velocity_profile");
    if (velocity_position_nh.hasParam("acceleration")) {
        int acceleration, deceleration;
        velocity_position_nh.getParam("acceleration", acceleration);
        velocity_position_nh.getParam("deceleration", deceleration);
        ROS_INFO("Setting Velocity Profile: accel(%d), decel(%d).", acceleration, deceleration);
        VCS_NODE_COMMAND(SetVelocityProfile, m_epos_handle, acceleration, deceleration);
    }

}

void EposProfileVelocityMode::activate()
{
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfileVelocityMode, m_epos_handle);
}

void EposProfileVelocityMode::read()
{}

void EposProfileVelocityMode::write(const double position, const double velocity, const double current)
{
    if (command_ == velocity) {
        return;
    } else {
        command_ = velocity;
    }
    int rpm;
    if (m_use_ros_unit) {
        rpm = static_cast<int>(command_ / M_PI * 30.0);
        ROS_DEBUG_STREAM("Target Velocity: " << command_ << " rad/s = " << rpm << " rpm");
    } else {
        rpm = static_cast<int>(command_);
        ROS_DEBUG_STREAM("Target Velocity: " << command_ << " rpm");
    }
    VCS_NODE_COMMAND(MoveWithVelocity, m_epos_handle, rpm);
}
