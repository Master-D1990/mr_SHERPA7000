#ifndef SHERPA_ACTUATION_PCA9685_H
#define SHERPA_ACTUATION_PCA9685_H

#include <string>
#include <cstdint>

namespace sherpa_actuation {

/**
 * @brief Class to control a PCA9685 PWM controller via I2C
 * 
 * This class provides an interface to the PCA9685 16-channel PWM controller
 * commonly used for servo and motor control
 */
class PCA9685 {
public:
  /**
   * @brief Constructor for PCA9685
   * 
   * @param device_path Path to I2C device (e.g., "/dev/i2c-1")
   * @param address I2C address of the PCA9685 (default: 0x40)
   * @param frequency PWM frequency in Hz (default: 50 Hz)
   */
  PCA9685(const std::string& device_path = "/dev/i2c-1", 
          uint8_t address = 0x40,
          double frequency = 50.0);

  /**
   * @brief Destructor for PCA9685
   */
  ~PCA9685();

  /**
   * @brief Initialize the PCA9685 controller
   * 
   * @return true if initialization successful
   */
  bool initialize();
  
  /**
   * @brief Set the PWM frequency
   * 
   * @param frequency_hz Frequency in Hz
   * @return true if successful
   */
  bool setFrequency(double frequency_hz);
  
  /**
   * @brief Set the PWM value for a specific channel
   * 
   * @param channel Channel number (0-15)
   * @param on_value On-time (0-4095)
   * @param off_value Off-time (0-4095)
   * @return true if successful
   */
  bool setPwm(uint8_t channel, uint16_t on_value, uint16_t off_value);
  
  /**
   * @brief Set the PWM duty cycle for a specific channel
   * 
   * @param channel Channel number (0-15)
   * @param duty_cycle Duty cycle (0.0-1.0)
   * @return true if successful
   */
  bool setDutyCycle(uint8_t channel, double duty_cycle);

private:
  int i2c_fd_; // File descriptor for I2C device
  std::string device_path_;
  uint8_t address_;
  double frequency_;
  
  // Register addresses
  static constexpr uint8_t MODE1 = 0x00;
  static constexpr uint8_t MODE2 = 0x01;
  static constexpr uint8_t PRESCALE = 0xFE;
  static constexpr uint8_t LED0_ON_L = 0x06;
  
  // Helper methods
  bool writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
};

} // namespace sherpa_actuation

#endif // SHERPA_ACTUATION_PCA9685_H
