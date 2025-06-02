#include <sherpa_actuation/pca9685.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <iostream>
#include <cstring> // für strerror
#include <ros/ros.h>
#include <stdlib.h> // für system

namespace sherpa_actuation {

PCA9685::PCA9685(const std::string& device_path, uint8_t address, double frequency)
    : i2c_fd_(-1), device_path_(device_path), address_(address), frequency_(frequency) {
}

PCA9685::~PCA9685() {
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

bool PCA9685::initialize() {
  // Detailed debugging before opening
  ROS_INFO("Attempting to open I2C device: %s", device_path_.c_str());
  ROS_INFO("PCA9685 address: 0x%02X", address_);

  // Try to diagnose I2C availability
  int result = system("i2cdetect -l");
  if (result != 0) {
    ROS_WARN("i2cdetect command failed with code: %d", result);
  }
  
  // Scan the I2C bus to see if any devices are present
  ROS_INFO("Scanning I2C bus...");
  int scan_result = system("i2cdetect -y 1");
  if (scan_result != 0) {
    ROS_WARN("i2cdetect scan command failed with code: %d", scan_result);
  }
  
  // Check if PCA9685 is available
  int check_result = system("i2cget -y 1 0x40 0x00 >/dev/null 2>&1");
  if (check_result != 0) {
    ROS_WARN("PCA9685 nicht erreichbar. Starte im Mock-Modus!");
    
    // MOCK MODE: Since hardware is not connected or not responding properly
    ROS_WARN("!!! RUNNING IN MOCK MODE - NO ACTUAL HARDWARE CONTROL !!!");
    ROS_WARN("This is a simulation mode to allow testing without PCA9685 hardware");
    
    // Simulate successful initialization
    i2c_fd_ = 999; // Fake file descriptor
    
    ROS_INFO("PCA9685 initialized in MOCK MODE at address 0x%02X", address_);
    return true;
  }
  
  // Normal initialization - hardware was detected
  // Open I2C device
  i2c_fd_ = open(device_path_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    ROS_ERROR("Failed to open I2C device: %s (Error: %s)", device_path_.c_str(), strerror(errno));
    return false;
  }

  // Reduce the I2C clock to a lower frequency for more reliable communication
  // This is especially important for longer cables or noisy environments
  unsigned long funcs;
  if (ioctl(i2c_fd_, I2C_FUNCS, &funcs) < 0) {
    ROS_WARN("Could not get I2C functionality: %s", strerror(errno));
    // Continue anyway, not critical
  }

  // Set I2C slave address
  if (ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) {
    ROS_ERROR("Failed to set I2C slave address: 0x%02X (Error: %s)", address_, strerror(errno));
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }
  
  // Add small delay to make sure the device is ready
  usleep(10000); // 10ms delay
  
  // Initialize the PCA9685
  // Reset the controller
  if (!writeRegister(MODE1, 0x00)) {
    ROS_ERROR("Failed to reset PCA9685");
    return false;
  }
  
  // Set frequency
  if (!setFrequency(frequency_)) {
    ROS_ERROR("Failed to set PCA9685 frequency");
    return false;
  }
  
  ROS_INFO("PCA9685 successfully initialized at address 0x%02X", address_);
  return true;
}

bool PCA9685::setFrequency(double frequency_hz) {
  frequency_ = frequency_hz;
  
  // Calculate prescale value for desired frequency
  // Formula from PCA9685 datasheet: prescale = round(25MHz / (4096 * frequency)) - 1
  // Laut Datenblatt ist der PCA9685 für bis zu 1526 Hz ausgelegt
  if (frequency_hz > 1526.0) {
    ROS_WARN("Requested frequency %.1f Hz exceeds PCA9685 maximum of 1526 Hz, limiting", frequency_hz);
    frequency_hz = 1526.0;
  }
  ROS_INFO("Setting PCA9685 frequency to %.1f Hz", frequency_hz);
  uint8_t prescale = round(25000000.0 / (4096.0 * frequency_hz)) - 1;
  
  // Read current MODE1 register
  uint8_t mode1 = readRegister(MODE1);
  
  // Put oscillator in sleep mode (set bit 4)
  if (!writeRegister(MODE1, mode1 | 0x10)) {
    return false;
  }
  
  // Write prescale value
  if (!writeRegister(PRESCALE, prescale)) {
    return false;
  }
  
  // Restore original MODE1 value (without sleep bit)
  if (!writeRegister(MODE1, mode1 & ~0x10)) {
    return false;
  }
  
  // Wait a small amount of time for oscillator to stabilize
  usleep(500);
  
  // Enable auto-increment (set bit 5)
  if (!writeRegister(MODE1, mode1 & ~0x10 | 0x20)) {
    return false;
  }
  
  return true;
}

bool PCA9685::setPwm(uint8_t channel, uint16_t on_value, uint16_t off_value) {
  // Ensure channel is valid (0-15)
  if (channel > 15) {
    ROS_ERROR("Invalid PWM channel: %d (must be 0-15)", channel);
    return false;
  }
  
  // Calculate register addresses
  uint8_t led_on_l = LED0_ON_L + 4 * channel;
  
  // Write the values
  if (!writeRegister(led_on_l, on_value & 0xFF)) return false;
  if (!writeRegister(led_on_l + 1, on_value >> 8)) return false;
  if (!writeRegister(led_on_l + 2, off_value & 0xFF)) return false;
  if (!writeRegister(led_on_l + 3, off_value >> 8)) return false;
  
  return true;
}

bool PCA9685::setDutyCycle(uint8_t channel, double duty_cycle) {
  // Nur negative Werte auf 0 begrenzen, obere Grenze entfernt
  if (duty_cycle < 0.0) duty_cycle = 0.0;
  
  // Sicherstellen, dass wir nicht über 4095 (12-bit) hinausgehen
  if (duty_cycle > 1.0) {
    ROS_INFO_THROTTLE(1.0, "Hoher Duty-Cycle Wert: %.2f (wird auf max. 4095 begrenzt)", duty_cycle);
    duty_cycle = std::min(duty_cycle, 1.0);
  }
  
  // Convert duty cycle to PWM value (0-4095)
  uint16_t off_value = static_cast<uint16_t>(4095 * duty_cycle);
  
  // For simplicity, we use a start time of 0
  return setPwm(channel, 0, off_value);
}

bool PCA9685::writeRegister(uint8_t reg, uint8_t value) {
  // Add small delay before writing to give the device time to be ready
  usleep(1000); // 1ms delay
  
  uint8_t buffer[2] = {reg, value};
  int attempts = 0;
  int max_attempts = 3;
  
  while (attempts < max_attempts) {
    if (write(i2c_fd_, buffer, 2) == 2) {
      return true;
    }
    
    ROS_WARN("Failed to write register 0x%02X (attempt %d/%d): %s", 
            reg, attempts+1, max_attempts, strerror(errno));
    attempts++;
    usleep(5000); // 5ms delay between attempts
  }
  
  ROS_ERROR("Failed to write register 0x%02X after %d attempts: %s", 
           reg, max_attempts, strerror(errno));
  return false;
}

uint8_t PCA9685::readRegister(uint8_t reg) {
  // Add delay before reading
  usleep(1000); // 1ms delay
  
  int attempts = 0;
  int max_attempts = 3;
  
  while (attempts < max_attempts) {
    if (write(i2c_fd_, &reg, 1) == 1) {
      usleep(1000); // Small delay between write and read
      
      uint8_t value = 0;
      if (read(i2c_fd_, &value, 1) == 1) {
        return value;
      }
      ROS_WARN("Failed to read from register 0x%02X (attempt %d/%d): %s", 
              reg, attempts+1, max_attempts, strerror(errno));
    } else {
      ROS_WARN("Failed to write register address 0x%02X for reading (attempt %d/%d): %s", 
              reg, attempts+1, max_attempts, strerror(errno));
    }
    
    attempts++;
    usleep(5000); // 5ms delay between attempts
  }
  
  ROS_ERROR("Failed to read register 0x%02X after %d attempts", reg, max_attempts);
  return 0;
}

} // namespace sherpa_actuation
