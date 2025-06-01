#include <sherpa_actuation/tb6612fng.h>
#include <ros/ros.h>

namespace sherpa_actuation {

TB6612FNG::TB6612FNG(std::shared_ptr<PCA9685> pwm_controller,
                     uint8_t apwm_channel,
                     uint8_t bpwm_channel,
                     int8_t stby_channel)
    : pwm_controller_(pwm_controller),
      apwm_channel_(apwm_channel),
      bpwm_channel_(bpwm_channel),
      stby_channel_(stby_channel),
      in_standby_(false) {
}

bool TB6612FNG::initialize() {
  // Set default state: motors stopped (PWM signals only)
  if (!pwm_controller_->setDutyCycle(apwm_channel_, 0.0)) return false;
  if (!pwm_controller_->setDutyCycle(bpwm_channel_, 0.0)) return false;
  
  // If standby pin is configured, make sure it's active (not in standby)
  if (stby_channel_ >= 0) {
    if (!setStandby(false)) return false;
  }
  
  ROS_INFO("TB6612FNG motor driver initialized (PWM only mode)");
  return true;
}

bool TB6612FNG::setMotorA(double speed, Direction direction) {
  // Clamp speed
  if (speed < 0.0) speed = 0.0;
  if (speed > 1.0) speed = 1.0;
  
  // Set speed (PWM) only - direction is controlled by GPIO elsewhere
  return pwm_controller_->setDutyCycle(apwm_channel_, speed);
}

bool TB6612FNG::setMotorB(double speed, Direction direction) {
  // Clamp speed
  if (speed < 0.0) speed = 0.0;
  if (speed > 1.0) speed = 1.0;
  
  // Set speed (PWM) only - direction is controlled by GPIO elsewhere
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
  // This function is deprecated: direction is now controlled by GPIO pins directly
  // This stub exists for backward compatibility
  ROS_DEBUG("TB6612FNG::setDirectionA called but ignored - direction now controlled by GPIO");
  return true;
}

bool TB6612FNG::setDirectionB(Direction direction) {
  // This function is deprecated: direction is now controlled by GPIO pins directly
  // This stub exists for backward compatibility
  ROS_DEBUG("TB6612FNG::setDirectionB called but ignored - direction now controlled by GPIO");
  return true;
}

} // namespace sherpa_actuation
