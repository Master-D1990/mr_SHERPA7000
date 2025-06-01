#include <sherpa_control/gpio_controller.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>
#include <errno.h>

namespace sherpa_control {

GPIOController::GPIOController() : initialized_(false) {}

GPIOController::~GPIOController() {
  // Clean up any exported GPIO pins
  if (initialized_) {
    // Unexport all used GPIO pins
    unexportGPIO(M1IN1_PIN);
    unexportGPIO(M1IN2_PIN);
    unexportGPIO(M2IN1_PIN);
    unexportGPIO(M2IN2_PIN);
    unexportGPIO(M3IN1_PIN);
    unexportGPIO(M3IN2_PIN);
    unexportGPIO(M4IN1_PIN);
    unexportGPIO(M4IN2_PIN);
    unexportGPIO(M5IN1_PIN);
    unexportGPIO(M5IN2_PIN);
    unexportGPIO(M6IN1_PIN);
    unexportGPIO(M6IN2_PIN);
    
    // Close all open file descriptors
    for (auto& fd_pair : gpio_file_descriptors_) {
      if (fd_pair.second >= 0) {
        close(fd_pair.second);
      }
    }
    
    gpio_file_descriptors_.clear();
  }
}

bool GPIOController::initialize() {
  if (initialized_) {
    return true;  // Already initialized
  }
  
  bool success = true;
  
  // TB6612 #1 (Motor 1 & 2)
  success &= exportGPIO(M1IN1_PIN);
  success &= setGPIODirection(M1IN1_PIN, "out");
  success &= exportGPIO(M1IN2_PIN);
  success &= setGPIODirection(M1IN2_PIN, "out");
  success &= exportGPIO(M2IN1_PIN);
  success &= setGPIODirection(M2IN1_PIN, "out");
  success &= exportGPIO(M2IN2_PIN);
  success &= setGPIODirection(M2IN2_PIN, "out");
  
  // TB6612 #2 (Motor 3 & 4)
  success &= exportGPIO(M3IN1_PIN);
  success &= setGPIODirection(M3IN1_PIN, "out");
  success &= exportGPIO(M3IN2_PIN);
  success &= setGPIODirection(M3IN2_PIN, "out");
  success &= exportGPIO(M4IN1_PIN);
  success &= setGPIODirection(M4IN1_PIN, "out");
  success &= exportGPIO(M4IN2_PIN);
  success &= setGPIODirection(M4IN2_PIN, "out");
  
  // TB6612 #3 (Motor 5 & 6)
  success &= exportGPIO(M5IN1_PIN);
  success &= setGPIODirection(M5IN1_PIN, "out");
  success &= exportGPIO(M5IN2_PIN);
  success &= setGPIODirection(M5IN2_PIN, "out");
  success &= exportGPIO(M6IN1_PIN);
  success &= setGPIODirection(M6IN1_PIN, "out");
  success &= exportGPIO(M6IN2_PIN);
  success &= setGPIODirection(M6IN2_PIN, "out");
  
  if (success) {
    ROS_INFO("GPIO pins successfully initialized for motor control");
    initialized_ = true;
  } else {
    ROS_ERROR("Failed to initialize GPIO pins");
  }
  
  return success;
}

bool GPIOController::exportGPIO(int pin_number) {
  // Export GPIO via sysfs interface
  int fd = open("/sys/class/gpio/export", O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Failed to open GPIO export file: %s", strerror(errno));
    return false;
  }
  
  // Convert pin number to string
  std::stringstream ss;
  ss << pin_number;
  std::string pin_str = ss.str();
  
  // Write the pin number to export
  if (write(fd, pin_str.c_str(), pin_str.length()) != static_cast<ssize_t>(pin_str.length())) {
    // If write failed but the file exists, the pin may already be exported
    // This is not always an error condition
    if (errno != EBUSY) {  // EBUSY means it's already exported
      ROS_WARN("Failed to export GPIO pin %d: %s", pin_number, strerror(errno));
      close(fd);
      return false;
    }
  }
  
  close(fd);
  
  // Wait for the system to create the files
  usleep(100000);  // 100ms delay to allow for file creation
  
  return true;
}

bool GPIOController::setGPIODirection(int pin_number, const std::string& direction) {
  std::stringstream ss;
  ss << "/sys/class/gpio/gpio" << pin_number << "/direction";
  std::string path = ss.str();
  
  int fd = open(path.c_str(), O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Failed to open GPIO direction file for pin %d: %s", pin_number, strerror(errno));
    return false;
  }
  
  if (write(fd, direction.c_str(), direction.length()) != static_cast<ssize_t>(direction.length())) {
    ROS_ERROR("Failed to set GPIO direction for pin %d: %s", pin_number, strerror(errno));
    close(fd);
    return false;
  }
  
  close(fd);
  return true;
}

bool GPIOController::setGPIOState(int pin_number, bool state) {
  if (!initialized_) {
    ROS_ERROR("GPIO controller not initialized");
    return false;
  }
  
  // Check if we already have an open file descriptor
  if (gpio_file_descriptors_.find(pin_number) == gpio_file_descriptors_.end() || 
      gpio_file_descriptors_[pin_number] < 0) {
    // Open GPIO value file
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << pin_number << "/value";
    std::string path = ss.str();
    
    int fd = open(path.c_str(), O_WRONLY);
    if (fd < 0) {
      ROS_ERROR("Failed to open GPIO value file for pin %d: %s", pin_number, strerror(errno));
      return false;
    }
    
    // Store the file descriptor for later use
    gpio_file_descriptors_[pin_number] = fd;
  }
  
  int fd = gpio_file_descriptors_[pin_number];
  const char* value = state ? "1" : "0";
  
  // Set value to beginning of file
  if (lseek(fd, 0, SEEK_SET) == -1) {
    ROS_ERROR("Failed to seek GPIO value file for pin %d: %s", pin_number, strerror(errno));
    return false;
  }
  
  if (write(fd, value, 1) != 1) {
    ROS_ERROR("Failed to write GPIO value for pin %d: %s", pin_number, strerror(errno));
    return false;
  }
  
  return true;
}

bool GPIOController::getGPIOState(int pin_number) {
  if (!initialized_) {
    ROS_ERROR("GPIO controller not initialized");
    return false;
  }
  
  // Open GPIO value file for reading
  std::stringstream ss;
  ss << "/sys/class/gpio/gpio" << pin_number << "/value";
  std::string path = ss.str();
  
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    ROS_ERROR("Failed to open GPIO value file for reading pin %d: %s", pin_number, strerror(errno));
    return false;
  }
  
  char value;
  if (read(fd, &value, 1) != 1) {
    ROS_ERROR("Failed to read GPIO value for pin %d: %s", pin_number, strerror(errno));
    close(fd);
    return false;
  }
  
  close(fd);
  return (value == '1');
}

bool GPIOController::unexportGPIO(int pin_number) {
  // Close any open file descriptor first
  if (gpio_file_descriptors_.find(pin_number) != gpio_file_descriptors_.end()) {
    if (gpio_file_descriptors_[pin_number] >= 0) {
      close(gpio_file_descriptors_[pin_number]);
      gpio_file_descriptors_[pin_number] = -1;
    }
  }
  
  // Unexport GPIO via sysfs interface
  int fd = open("/sys/class/gpio/unexport", O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Failed to open GPIO unexport file: %s", strerror(errno));
    return false;
  }
  
  // Convert pin number to string
  std::stringstream ss;
  ss << pin_number;
  std::string pin_str = ss.str();
  
  // Write the pin number to unexport
  if (write(fd, pin_str.c_str(), pin_str.length()) != static_cast<ssize_t>(pin_str.length())) {
    ROS_ERROR("Failed to unexport GPIO pin %d: %s", pin_number, strerror(errno));
    close(fd);
    return false;
  }
  
  close(fd);
  return true;
}

} // namespace sherpa_control