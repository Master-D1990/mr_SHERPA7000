#ifndef SHERPA_CONTROL_GPIO_CONTROLLER_H
#define SHERPA_CONTROL_GPIO_CONTROLLER_H

#include <map>
#include <string>

namespace sherpa_control {

class GPIOController {
public:
  // GPIO Pin definitions for motor control
  // TB6612 #1 (Motor 1 & 2)
  static constexpr int M1IN1_PIN = 17;  // Motor 1 IN1
  static constexpr int M1IN2_PIN = 27;  // Motor 1 IN2
  static constexpr int M2IN1_PIN = 22;  // Motor 2 IN1
  static constexpr int M2IN2_PIN = 23;  // Motor 2 IN2
  
  // TB6612 #2 (Motor 3 & 4)
  static constexpr int M3IN1_PIN = 5;   // Motor 3 IN1
  static constexpr int M3IN2_PIN = 6;   // Motor 3 IN2
  static constexpr int M4IN1_PIN = 13;  // Motor 4 IN1
  static constexpr int M4IN2_PIN = 19;  // Motor 4 IN2
  
  // TB6612 #3 (Motor 5 & 6)
  static constexpr int M5IN1_PIN = 12;  // Motor 5 IN1
  static constexpr int M5IN2_PIN = 16;  // Motor 5 IN2
  static constexpr int M6IN1_PIN = 20;  // Motor 6 IN1
  static constexpr int M6IN2_PIN = 21;  // Motor 6 IN2
  
  GPIOController();
  ~GPIOController();
  
  bool initialize();
  bool setGPIOState(int pin_number, bool state);
  bool getGPIOState(int pin_number);
  
private:
  bool exportGPIO(int pin_number);
  bool unexportGPIO(int pin_number);
  bool setGPIODirection(int pin_number, const std::string& direction);
  
  bool initialized_;
  std::map<int, int> gpio_file_descriptors_;
};

} // namespace sherpa_control

#endif // SHERPA_CONTROL_GPIO_CONTROLLER_H