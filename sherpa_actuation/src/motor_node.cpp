#include <sherpa_actuation/motor_node.h>
#include <ros/ros.h>

namespace sherpa_actuation {

MotorNode::MotorNode(ros::NodeHandle& nh)
    : nh_(nh), timeout_duration_(0.5), emergency_stopped_(false) {
  motor_controller_ = std::make_unique<MotorController>();
}

bool MotorNode::initialize() {
  // Initialize motor controller
  if (!motor_controller_->initialize()) {
    ROS_ERROR("Failed to initialize motor controller");
    return false;
  }
  
  // Get parameters from ROS parameter server
  nh_.param<double>("cmd_vel_timeout", timeout_duration_, 0.5);
  
  // Set up subscribers
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MotorNode::cmdVelCallback, this);
  emergency_stop_sub_ = nh_.subscribe("emergency_stop", 1, &MotorNode::emergencyStopCallback, this);
  
  // Set up watchdog timer
  watchdog_timer_ = nh_.createTimer(ros::Duration(0.1), &MotorNode::watchdogCallback, this);
  
  // Record time of initialization
  last_cmd_time_ = ros::Time::now();
  
  ROS_INFO("SHERPA motor node initialized, waiting for velocity commands");
  return true;
}

void MotorNode::run() {
  // This method is primarily for spinOnce usage
  // For normal operation, just use ros::spin() in the main function
  ros::spinOnce();
}

void MotorNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Update last command time
  last_cmd_time_ = ros::Time::now();
  
  // Don't process commands if in emergency stop mode
  if (emergency_stopped_) {
    ROS_WARN_THROTTLE(1.0, "Command received while in emergency stop mode");
    return;
  }
  
  // Process velocity command
  if (!motor_controller_->processVelocityCommand(*msg)) {
    ROS_ERROR("Failed to process velocity command");
  }
}

void MotorNode::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
  emergency_stopped_ = msg->data;
  
  if (emergency_stopped_) {
    ROS_WARN("Emergency stop activated");
    motor_controller_->stopAllMotors(true);
  } else {
    ROS_INFO("Emergency stop deactivated");
  }
}

void MotorNode::watchdogCallback(const ros::TimerEvent& event) {
  // If we haven't received a command recently and not in emergency stop,
  // stop the motors as a safety measure
  ros::Duration time_since_last_cmd = ros::Time::now() - last_cmd_time_;
  
  if (time_since_last_cmd.toSec() > timeout_duration_ && !emergency_stopped_) {
    ROS_WARN_THROTTLE(1.0, "Command timeout, stopping motors");
    motor_controller_->stopAllMotors(false);  // Stop without braking
  }
}

} // namespace sherpa_actuation

// Main function
int main(int argc, char** argv) {
  ros::init(argc, argv, "sherpa_motor_node");
  ros::NodeHandle nh("~");
  
  sherpa_actuation::MotorNode motor_node(nh);
  
  if (!motor_node.initialize()) {
    ROS_ERROR("Failed to initialize motor node");
    return 1;
  }
  
  ros::spin();
  return 0;
}
