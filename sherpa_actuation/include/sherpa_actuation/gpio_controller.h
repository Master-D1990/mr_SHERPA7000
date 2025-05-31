#ifndef SHERPA_ACTUATION_GPIO_CONTROLLER_H
#define SHERPA_ACTUATION_GPIO_CONTROLLER_H

#include <memory>
#include <map>
#include <string>

namespace sherpa_actuation {

/**
 * @brief Class to control GPIO pins for motor direction control
 * 
 * This class initializes and controls GPIO pins
 * using the system's GPIO interface (sysfs or gpiod)
 */
class GPIOController {
public:
  /**
   * @brief Constructor for GPIOController
   */
  GPIOController();
  
  /**
   * @brief Destructor for GPIOController
   */
  ~GPIOController();
  
  /**
   * @brief Initialize the GPIO pins
   * 
   * @return true if initialization successful
   */
  bool initialize();
  
  /**
   * @brief Set the state of a GPIO pin
   * 
   * @param pin_number GPIO pin number
   * @param state true for HIGH, false for LOW
   * @return true if operation successful
   */
  bool setGPIOState(int pin_number, bool state);
  
  /**
   * @brief Get the state of a GPIO pin
   * 
   * @param pin_number GPIO pin number
   * @return true if HIGH, false if LOW or error
   */
  bool getGPIOState(int pin_number);
  
  // GPIO pin numbers for TB6612FNG motor drivers
  // M1 (Motor 1 & 2)
  static constexpr int M1IN1_PIN = 17;  // GPIO17 for Motor 1
  static constexpr int M1IN2_PIN = 27;  // GPIO27 for Motor 2
  
  // M2 (Motor 3 & 4)
  static constexpr int M2IN1_PIN = 22;  // GPIO22 for Motor 3
  static constexpr int M2IN2_PIN = 23;  // GPIO23 for Motor 4
  
  // M3 (Motor 5 & 6)
  static constexpr int M3IN1_PIN = 5;   // GPIO5 for Motor 5
  static constexpr int M3IN2_PIN = 6;   // GPIO6 for Motor 6

private:
  bool initialized_;
  std::map<int, int> gpio_file_descriptors_;  // Map of GPIO pin numbers to file descriptors
  
  /**
   * @brief Export a GPIO pin for usage
   * 
   * @param pin_number GPIO pin number
   * @return true if successful
   */
  bool exportGPIO(int pin_number);
  
  /**
   * @brief Set the direction of a GPIO pin (input/output)
   * 
   * @param pin_number GPIO pin number
   * @param direction "in" for input, "out" for output
   * @return true if successful
   */
  bool setGPIODirection(int pin_number, const std::string& direction);
  
  /**
   * @brief Unexport a GPIO pin when done using it
   * 
   * @param pin_number GPIO pin number
   * @return true if successful
   */
  bool unexportGPIO(int pin_number);
};

} // namespace sherpa_actuation

#endif // SHERPA_ACTUATION_GPIO_CONTROLLER_H
