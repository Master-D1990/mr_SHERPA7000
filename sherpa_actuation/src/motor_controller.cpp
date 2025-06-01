#include <sherpa_actuation/motor_controller.h>
#include <ros/ros.h>
#include <cmath>

namespace sherpa_actuation {

// Define static constexpr member variables (required in C++11)
constexpr uint8_t MotorController::MD1_APWM;
constexpr uint8_t MotorController::MD1_BPWM;
constexpr uint8_t MotorController::MD2_APWM;
constexpr uint8_t MotorController::MD2_BPWM;


MotorController::MotorController(const std::string& i2c_device, uint8_t pca9685_address)
    : initialized_(false) {
  // Create the PWM controller
  pwm_controller_ = std::make_shared<PCA9685>(i2c_device, pca9685_address, 100.0); // 100 Hz PWM frequency
  
  // Create GPIO controller for direct pin control
  gpio_controller_ = std::make_shared<sherpa_control::GPIOController>();
  
  // Create motor drivers (only using PWM channels)
  motor_driver_1_ = std::make_shared<TB6612FNG>(
    pwm_controller_,
    MD1_APWM,
    MD1_BPWM
  );
  
  motor_driver_2_ = std::make_shared<TB6612FNG>(
    pwm_controller_,
    MD2_APWM,
    MD2_BPWM
  );
}

bool MotorController::initialize() {
  // Initialize PWM controller
  if (!pwm_controller_->initialize()) {
    ROS_ERROR("Failed to initialize PCA9685 PWM controller");
    return false;
  }
  
  // Initialize GPIO controller for direct pin control
  if (!gpio_controller_->initialize()) {
    ROS_ERROR("Failed to initialize GPIO controller");
    return false;
  }
  
  // Initialize motor drivers
  if (!motor_driver_1_->initialize()) {
    ROS_ERROR("Failed to initialize first TB6612FNG motor driver");
    return false;
  }
  
  if (!motor_driver_2_->initialize()) {
    ROS_ERROR("Failed to initialize second TB6612FNG motor driver");
    return false;
  }
  
  // Stop all motors initially
  if (!stopAllMotors(true)) {
    ROS_ERROR("Failed to set initial motor state");
    return false;
  }
  
  initialized_ = true;
  ROS_INFO("SHERPA motor controller initialized successfully");
  return true;
}

bool MotorController::setMotorSpeed(MotorPosition motor, double speed) {
  // Make sure the controller is initialized
  if (!initialized_) {
    ROS_ERROR("Motor controller not initialized");
    return false;
  }
  
  // Limit the speed
  speed = limitSpeed(speed);
  
  // Determine direction
  Direction direction = getDirection(speed);
  
  // Use absolute speed value
  double abs_speed = std::fabs(speed);
  
  bool success = true;
  
  // Set GPIO pins for direction control and PWM for speed
  switch (motor) {
    case MotorPosition::FRONT_LEFT:   // TB6612 #1, Motor 1
      // Set GPIO pins for direction - TB6612FNG requires BOTH IN1 and IN2 pins
      if (direction == Direction::FORWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN2_PIN, false);
      } else if (direction == Direction::BACKWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN2_PIN, true);
      } else if (direction == Direction::BRAKE) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN2_PIN, true);
      } else { // STANDBY
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN2_PIN, false);
      }
      
      // Set PWM for speed control via TB6612FNG - don't pass direction, just speed
      success &= motor_driver_1_->setMotorA(abs_speed, Direction::FORWARD);  // Always forward, GPIO handles direction
      return success;
      
    case MotorPosition::REAR_LEFT:  // TB6612 #1, Motor 2
      // Set GPIO pins for direction
      if (direction == Direction::FORWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN2_PIN, false);
      } else if (direction == Direction::BACKWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN2_PIN, true);
      } else if (direction == Direction::BRAKE) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN2_PIN, true);
      } else { // STANDBY
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN2_PIN, false);
      }
      
      // Set PWM for speed control via TB6612FNG - don't pass direction, just speed
      success &= motor_driver_1_->setMotorB(abs_speed, Direction::FORWARD);  // Always forward, GPIO handles direction
      return success;
      
    case MotorPosition::FRONT_RIGHT:  // TB6612 #2, Motor 3
      // Set GPIO pins for direction
      if (direction == Direction::FORWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN2_PIN, true);
      } else if (direction == Direction::BACKWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN2_PIN, false);
      } else if (direction == Direction::BRAKE) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN2_PIN, true);
      } else { // STANDBY
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN2_PIN, false);
      }
      
      // Set PWM for speed control via TB6612FNG - don't pass direction, just speed
      success &= motor_driver_2_->setMotorA(abs_speed, Direction::FORWARD);  // Always forward, GPIO handles direction
      return success;
      
    case MotorPosition::REAR_RIGHT: // TB6612 #2, Motor 4
      // Set GPIO pins for direction
      if (direction == Direction::FORWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN2_PIN, true);
      } else if (direction == Direction::BACKWARD) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN2_PIN, false);
      } else if (direction == Direction::BRAKE) {
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN1_PIN, true);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN2_PIN, true);
      } else { // STANDBY
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN1_PIN, false);
        success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN2_PIN, false);
      }
      
      // Set PWM for speed control via TB6612FNG - don't pass direction, just speed
      success &= motor_driver_2_->setMotorB(abs_speed, Direction::FORWARD);  // Always forward, GPIO handles direction
      return success;
      
    default:
      ROS_ERROR("Invalid motor position");
      return false;
  }
}

bool MotorController::stopAllMotors(bool brake) {
  bool success = true;
  
  // Stop PWM signals via TB6612FNG - set all speeds to 0
  success &= motor_driver_1_->setMotorA(0.0, Direction::FORWARD); // Direction is ignored now
  success &= motor_driver_1_->setMotorB(0.0, Direction::FORWARD); // Direction is ignored now
  success &= motor_driver_2_->setMotorA(0.0, Direction::FORWARD); // Direction is ignored now
  success &= motor_driver_2_->setMotorB(0.0, Direction::FORWARD); // Direction is ignored now
  
  // Set all GPIO control pins appropriately
  bool pin_state = brake ? true : false;  // For BRAKE: HIGH, for STANDBY: LOW
  
  // TB6612 #1 Motor 1 & 2 pins
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN1_PIN, pin_state);
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M1IN2_PIN, pin_state);
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN1_PIN, pin_state);
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M2IN2_PIN, pin_state);
  
  // TB6612 #2 Motor 3 & 4 pins
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN1_PIN, pin_state);
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M3IN2_PIN, pin_state);
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN1_PIN, pin_state);
  success &= gpio_controller_->setGPIOState(sherpa_control::GPIOController::M4IN2_PIN, pin_state);
  
  return success;
}

bool MotorController::processVelocityCommand(const geometry_msgs::Twist& twist) {
  // Extract linear and angular velocity components
  double linear_x = twist.linear.x;   // Forward/backward velocity
  double angular_z = twist.angular.z;  // Rotational velocity
  
  // Calculate wheel velocities using a differential drive model
  // Left side = linear_x - angular_z * wheelbase/2
  // Right side = linear_x + angular_z * wheelbase/2
  
  // For simplicity, we'll assume a normalized wheelbase of 1.0
  double left_speed = linear_x - angular_z * 0.5;
  double right_speed = linear_x + angular_z * 0.5;
  
  // Normalize values if either exceeds +/-1.0
  double max_speed = std::max(std::fabs(left_speed), std::fabs(right_speed));
  if (max_speed > 1.0) {
    left_speed /= max_speed;
    right_speed /= max_speed;
  }
  
  // Set motor speeds
  bool success = true;
  success &= setMotorSpeed(MotorPosition::REAR_LEFT, left_speed);
  success &= setMotorSpeed(MotorPosition::FRONT_LEFT, left_speed);
  success &= setMotorSpeed(MotorPosition::REAR_RIGHT, right_speed);
  success &= setMotorSpeed(MotorPosition::FRONT_RIGHT, right_speed);
  
  return success;
}

bool MotorController::isOperational() const {
  return initialized_;
}

Direction MotorController::getDirection(double speed) {
  if (speed > 0.0) {
    return Direction::FORWARD;
  } else if (speed < 0.0) {
    return Direction::BACKWARD;
  } else {
    return Direction::BRAKE;
  }
}

double MotorController::limitSpeed(double speed) {
  if (speed > 1.0) return 1.0;
  if (speed < -1.0) return -1.0;
  return speed;
}

} // namespace sherpa_actuation