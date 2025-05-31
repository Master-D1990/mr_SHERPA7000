#ifndef SHERPA_ACTUATION_MOTOR_NODE_H
#define SHERPA_ACTUATION_MOTOR_NODE_H

#include <sherpa_actuation/motor_controller.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <memory>

namespace sherpa_actuation {

/**
 * @brief ROS Node for controlling the SHERPA motors
 */
class MotorNode {
public:
  /**
   * @brief Constructor for MotorNode
   * 
   * @param nh ROS node handle
   */
  MotorNode(ros::NodeHandle& nh);
  
  /**
   * @brief Initialize the motor node
   * 
   * @return true if initialization successful
   */
  bool initialize();
  
  /**
   * @brief Run the node
   */
  void run();

private:
  ros::NodeHandle& nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber emergency_stop_sub_;
  ros::Publisher motor_status_pub_;
  ros::Timer watchdog_timer_;
  
  std::unique_ptr<MotorController> motor_controller_;
  ros::Time last_cmd_time_;
  double timeout_duration_;
  bool emergency_stopped_;
  
  // Callback for velocity commands
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  
  // Callback for emergency stop
  void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
  
  // Watchdog timer callback
  void watchdogCallback(const ros::TimerEvent& event);
};

} // namespace sherpa_actuation

#endif // SHERPA_ACTUATION_MOTOR_NODE_H
