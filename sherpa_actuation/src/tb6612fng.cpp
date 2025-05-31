#include <sherpa_actuation/tb6612fng.h>
#include <ros/ros.h>

namespace sherpa_actuation {

TB6612FNG::TB6612FNG(std::shared_ptr<PCA9685> pwm_controller,
                     uint8_t ain1_channel, uint8_t ain2_channel, uint8_t apwm_channel,
                     uint8_t bin1_channel, uint8_t bin2_channel, uint8_t bpwm_channel,
                     int8_t stby_channel)
    : pwm_controller_(pwm_controller),
      ain1_channel_(ain1_channel),
      ain2_channel_(ain2_channel),
      apwm_channel_(apwm_channel),
      bin1_channel_(bin1_channel),
      bin2_channel_(bin2_channel),
      bpwm_channel_(bpwm_channel),
      stby_channel_(stby_channel),
      in_standby_(false) {
}

bool TB6612FNG::initialize() {
  // Set default state: motors stopped and not in standby
  if (!setMotorA(0.0, Direction::BRAKE)) return false;
  if (!setMotorB(0.0, Direction::BRAKE)) return false;
  
  // If standby pin is configured, make sure it's active (not in standby)
  if (stby_channel_ >= 0) {
    if (!setStandby(false)) return false;
  }
  
  ROS_INFO("TB6612FNG motor driver initialized");
  return true;
}

bool TB6612FNG::setMotorA(double speed, Direction direction) {
  // Clamp speed
  if (speed < 0.0) speed = 0.0;
  if (speed > 1.0) speed = 1.0;
  
  // Set direction pins
  if (!setDirectionA(direction)) return false;
  
  // Set speed (PWM)
  return pwm_controller_->setDutyCycle(apwm_channel_, speed);
}

bool TB6612FNG::setMotorB(double speed, Direction direction) {
  // Clamp speed
  if (speed < 0.0) speed = 0.0;
  if (speed > 1.0) speed = 1.0;
  
  // Set direction pins
  if (!setDirectionB(direction)) return false;
  
  // Set speed (PWM)
  return pwm_controller_->setDutyCycle(bpwm_channel_, speed);
}

bool TB6612FNG::setStandby(bool standby) {
  if (stby_channel_ < 0) {
    // No standby pin configured
    return true;
  }
  
  in_standby_ = standby;
  
  // For standby: set pin low; for active: set pin high
  return pwm_controller_->setDutyCycle(stby_channel_, standby ? 0.0 : 1.0);
}

bool TB6612FNG::setDirectionA(Direction direction) {
  bool success = true;
  
  switch (direction) {
    case Direction::FORWARD:
      success &= pwm_controller_->setDutyCycle(ain1_channel_, 1.0);
      success &= pwm_controller_->setDutyCycle(ain2_channel_, 0.0);
      break;
      
    case Direction::BACKWARD:
      success &= pwm_controller_->setDutyCycle(ain1_channel_, 0.0);
      success &= pwm_controller_->setDutyCycle(ain2_channel_, 1.0);
      break;
      
    case Direction::BRAKE:
      success &= pwm_controller_->setDutyCycle(ain1_channel_, 1.0);
      success &= pwm_controller_->setDutyCycle(ain2_channel_, 1.0);
      break;
      
    case Direction::STANDBY:
      success &= pwm_controller_->setDutyCycle(ain1_channel_, 0.0);
      success &= pwm_controller_->setDutyCycle(ain2_channel_, 0.0);
      break;
  }
  
  return success;
}

bool TB6612FNG::setDirectionB(Direction direction) {
  bool success = true;
  
  switch (direction) {
    case Direction::FORWARD:
      success &= pwm_controller_->setDutyCycle(bin1_channel_, 1.0);
      success &= pwm_controller_->setDutyCycle(bin2_channel_, 0.0);
      break;
      
    case Direction::BACKWARD:
      success &= pwm_controller_->setDutyCycle(bin1_channel_, 0.0);
      success &= pwm_controller_->setDutyCycle(bin2_channel_, 1.0);
      break;
      
    case Direction::BRAKE:
      success &= pwm_controller_->setDutyCycle(bin1_channel_, 1.0);
      success &= pwm_controller_->setDutyCycle(bin2_channel_, 1.0);
      break;
      
    case Direction::STANDBY:
      success &= pwm_controller_->setDutyCycle(bin1_channel_, 0.0);
      success &= pwm_controller_->setDutyCycle(bin2_channel_, 0.0);
      break;
  }
  
  return success;
}

} // namespace sherpa_actuation
