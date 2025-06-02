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
    : initialized_(false), last_command_time_(ros::Time::now()) {
  // Create the PWM controller
  pwm_controller_ = std::make_shared<PCA9685>(i2c_device, pca9685_address, 1000.0); // 1000 Hz PWM frequency für optimale DC-Approximation
  
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
  double linear_y = twist.linear.y;   // Left/right (strafing) velocity
  double angular_z = twist.angular.z; // Rotational velocity

  // Calculate time delta since last command
  ros::Time now = ros::Time::now();
  double dt = (now - last_command_time_).toSec();

  // Set minimum and maximum time delta
  dt = std::max(dt, 0.01);   // Minimum 10ms to prevent division by zero
  dt = std::min(dt, 0.1);    // Maximum 100ms to prevent excessive acceleration after long pauses

  // Calculate mecanum wheel kinematics for target speed
  double target_front_left  = linear_x - linear_y - angular_z;
  double target_front_right = linear_x + linear_y + angular_z;
  double target_rear_left   = linear_x + linear_y - angular_z;
  double target_rear_right  = linear_x - linear_y + angular_z;

  // Calculate mecanum wheel kinematics for last speed
  double last_front_left  = linear_x_last_ - linear_y_last_ - angular_z_last_;
  double last_front_right = linear_x_last_ + linear_y_last_ + angular_z_last_;
  double last_rear_left   = linear_x_last_ + linear_y_last_ - angular_z_last_;
  double last_rear_right  = linear_x_last_ - linear_y_last_ + angular_z_last_;

  // Calculate speed changes for each wheel
  double delta_fl = target_front_left - last_front_left;
  double delta_fr = target_front_right - last_front_right;
  double delta_rl = target_rear_left - last_rear_left;
  double delta_rr = target_rear_right - last_rear_right;

  // Calculate total absolute acceleration
  double total_accel = std::fabs(delta_fl) + std::fabs(delta_fr) + 
                       std::fabs(delta_rl) + std::fabs(delta_rr);

  // Calculate allowed acceleration based on time delta
  double allowed_accel = max_accel_ * dt * 4.0; // 4.0 because we have 4 wheels

  // If total acceleration exceeds allowed, scale all wheel accelerations
  if (total_accel > allowed_accel && total_accel > 0.001) {
    double scale_factor = allowed_accel / total_accel;
    // Scale the wheel speed changes
    delta_fl *= scale_factor;
    delta_fr *= scale_factor;
    delta_rl *= scale_factor;
    delta_rr *= scale_factor;
    // Calculate new target speeds with limited acceleration
    target_front_left = last_front_left + delta_fl;
    target_front_right = last_front_right + delta_fr;
    target_rear_left = last_rear_left + delta_rl;
    target_rear_right = last_rear_right + delta_rr;
    // Calculate back to twist components for next cycle
    linear_x = (target_front_left + target_front_right + target_rear_left + target_rear_right) / 4.0;
    linear_y = (target_front_right - target_front_left + target_rear_left - target_rear_right) / 4.0;
    angular_z = (target_front_right - target_front_left - target_rear_left + target_rear_right) / 4.0;
  }

  // Remember current velocity components and time for next cycle
  linear_x_last_ = linear_x;
  linear_y_last_ = linear_y;
  angular_z_last_ = angular_z;
  last_command_time_ = now;

  // --- KORREKTE MECANUM-KINEMATIK ---
  double front_left  = linear_x - linear_y - angular_z;
  double front_right = linear_x + linear_y + angular_z;
  double rear_left   = linear_x + linear_y - angular_z;
  double rear_right  = linear_x - linear_y + angular_z;

  // Set motor speeds
  bool success = true;
  success &= setMotorSpeed(MotorPosition::FRONT_LEFT,  front_left);
  success &= setMotorSpeed(MotorPosition::FRONT_RIGHT, front_right);
  success &= setMotorSpeed(MotorPosition::REAR_LEFT,   rear_left);
  success &= setMotorSpeed(MotorPosition::REAR_RIGHT,  rear_right);

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
  // Geschwindigkeitsbegrenzung entfernt
  return speed;
}

double MotorController::limitAcceleration(double new_speed, double last_speed) {
  // This method is kept for compatibility but is no longer used
  // Acceleration limiting is now done in processVelocityCommand by limiting
  // the total combined wheel acceleration
  
  // Calculate time delta since last command
  ros::Time now = ros::Time::now();
  double dt = (now - last_command_time_).toSec();
  
  // Ensure dt is not too small to prevent division by zero or erratic behavior
  dt = std::max(dt, 0.01);  // Minimum 10ms
  
  // Calculate allowed change based on time and max acceleration
  double allowed_change = max_accel_ * dt;
  
  // Calculate actual change
  double change = new_speed - last_speed;
  
  // Limit the change based on allowed change
  if (change > allowed_change) {
    return last_speed + allowed_change;
  } else if (change < -allowed_change) {
    return last_speed - allowed_change;
  }
  
  return new_speed;
}

} // namespace sherpa_actuation