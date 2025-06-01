#ifndef SHERPA_ACTUATION_TB6612FNG_H
#define SHERPA_ACTUATION_TB6612FNG_H

#include <sherpa_actuation/pca9685.h>
#include <memory>

namespace sherpa_actuation {

/**
 * @brief Motor direction enum
 */
enum class Direction {
  FORWARD,
  BACKWARD,
  BRAKE,
  STANDBY
};

/**
 * @brief Class to control a TB6612FNG dual motor driver
 * 
 * This class controls a SparkFun Motor Driver - Dual TB6612FNG (1A)
 * which can control two DC motors
 */
class TB6612FNG {
public:
  /**
   * @brief Constructor for TB6612FNG motor driver
   * 
   * @param pwm_controller Reference to PCA9685 controller
   * @param apwm_channel PWM channel for PWMA input
   * @param bpwm_channel PWM channel for PWMB input
   * @param stby_channel PWM channel for STBY pin (optional)
   */
  TB6612FNG(std::shared_ptr<PCA9685> pwm_controller,
            uint8_t apwm_channel,
            uint8_t bpwm_channel,
            int8_t stby_channel = -1);  // -1 means no standby control
            
  /**
   * @brief Initialize the motor driver
   * 
   * @return true if initialization successful
   */
  bool initialize();
  
  /**
   * @brief Set the speed and direction of motor A
   * 
   * @param speed Speed value (0.0-1.0)
   * @param direction Direction of rotation
   * @return true if successful
   */
  bool setMotorA(double speed, Direction direction);
  
  /**
   * @brief Set the speed and direction of motor B
   * 
   * @param speed Speed value (0.0-1.0)
   * @param direction Direction of rotation
   * @return true if successful
   */
  bool setMotorB(double speed, Direction direction);
  
  /**
   * @brief Set standby mode
   * 
   * @param standby True to enter standby, false to exit standby
   * @return true if successful
   */
  bool setStandby(bool standby);

private:
  std::shared_ptr<PCA9685> pwm_controller_;
  uint8_t apwm_channel_;
  uint8_t bpwm_channel_;
  int8_t stby_channel_;
  bool in_standby_;
  
  // Helper methods to set direction pins
  bool setDirectionA(Direction direction);
  bool setDirectionB(Direction direction);
};

} // namespace sherpa_actuation

#endif // SHERPA_ACTUATION_TB6612FNG_H
