#ifndef SHERPA_ACTUATION_MOTOR_CONTROLLER_H
#define SHERPA_ACTUATION_MOTOR_CONTROLLER_H

#include <sherpa_actuation/pca9685.h>
#include <sherpa_actuation/tb6612fng.h>
#include <sherpa_control/gpio_controller.h>
#include <memory>
#include <vector>
#include <geometry_msgs/Twist.h>

namespace sherpa_actuation {

/**
 * @brief Motor position enumeration
 */
enum class MotorPosition {
  REAR_LEFT = 0,   // Motor 1
  FRONT_LEFT = 1,  // Motor 2
  REAR_RIGHT = 2,  // Motor 3
  FRONT_RIGHT = 3  // Motor 4
};

/**
 * @brief Class to control all motors of the SHERPA robot
 * 
 * This class manages the four motors of the SHERPA robot using
 * two TB6612FNG motor drivers controlled via a PCA9685 PWM controller
 */
class MotorController {
public:
  /**
   * @brief Constructor for MotorController
   * 
   * @param i2c_device Path to I2C device (e.g., "/dev/i2c-1")
   * @param pca9685_address I2C address of the PCA9685 (default: 0x40)
   */
  MotorController(const std::string& i2c_device = "/dev/i2c-1",
                  uint8_t pca9685_address = 0x40);
  
  /**
   * @brief Initialize the motor controller
   * 
   * @return true if initialization successful
   */
  bool initialize();
  
  /**
   * @brief Set the speed of a specific motor
   * 
   * @param motor Motor position
   * @param speed Speed value (-1.0 to 1.0, negative for reverse)
   * @return true if successful
   */
  bool setMotorSpeed(MotorPosition motor, double speed);
  
  /**
   * @brief Stop all motors
   * 
   * @param brake Whether to brake (true) or coast (false)
   * @return true if successful
   */
  bool stopAllMotors(bool brake = true);
  
  /**
   * @brief Process velocity command
   * 
   * @param twist Twist message containing velocity commands
   * @return true if successful
   */
  bool processVelocityCommand(const geometry_msgs::Twist& twist);
  
  /**
   * @brief Check if the controller is operational
   * 
   * @return true if operational
   */
  bool isOperational() const;

private:
  std::shared_ptr<PCA9685> pwm_controller_;
  std::shared_ptr<TB6612FNG> motor_driver_1_;  // Controls motors 1 & 2 (left side)
  std::shared_ptr<TB6612FNG> motor_driver_2_;  // Controls motors 3 & 4 (right side)
  std::shared_ptr<sherpa_control::GPIOController> gpio_controller_; // Controls GPIO pins for motor direction
  bool initialized_;
  
  // Acceleration limiting variables
  double linear_x_last_ = 0.0;  // Last commanded linear x velocity
  double linear_y_last_ = 0.0;  // Last commanded linear y velocity
  double angular_z_last_ = 0.0; // Last commanded angular z velocity
  double max_accel_ = 0.6;      // Maximum acceleration in units/second
  ros::Time last_command_time_; // Time of the last velocity command
  
  // Configuration for TB6612FNG PWM pins on PCA9685
  // Motor driver 1 (left motors)
  static constexpr uint8_t MD1_APWM = 0;  // Motor 1 (Rear Left) PWM
  static constexpr uint8_t MD1_BPWM = 1;  // Motor 2 (Front Left) PWM

  
  // Motor driver 2 (right motors)
  static constexpr uint8_t MD2_APWM = 2; // Motor 3 (Rear Right) PWM
  static constexpr uint8_t MD2_BPWM = 3; // Motor 4 (Front Right) PWM

  
  // Helper methods
  Direction getDirection(double speed);
  double limitSpeed(double speed);
  double limitAcceleration(double new_speed, double last_speed);
};

} // namespace sherpa_actuation

#endif // SHERPA_ACTUATION_MOTOR_CONTROLLER_H
