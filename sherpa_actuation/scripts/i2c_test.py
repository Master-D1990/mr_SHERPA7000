#!/usr/bin/env python3
"""
Simple I2C test script for the SHERPA robot
Tests if the PCA9685 PWM controller can be detected on the I2C bus
"""
import smbus
import time
import sys
import traceback

def scan_i2c_bus():
    """Scan the I2C bus for devices and print their addresses"""
    print("Scanning I2C bus...")
    
    try:
        bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
        
        found_devices = []
        for address in range(0x03, 0x78):
            try:
                bus.read_byte(address)
                print(f"Found device at: 0x{address:02X}")
                found_devices.append(address)
            except Exception as e:
                if address == 0x40:  # Only print error for the address we're interested in
                    print(f"Error at address 0x40: {e}")
                pass
        
        if not found_devices:
            print("No I2C devices found!")
        
        return found_devices
    except Exception as e:
        print(f"Error initializing SMBus: {e}")
        return []

def test_pca9685(address=0x40):
    """Test basic communication with PCA9685"""
    print(f"Testing PCA9685 at address 0x{address:02X}...")
    try:
        bus = smbus.SMBus(1)
        
        # Read MODE1 register (should be at address 0x00)
        mode1 = bus.read_byte_data(address, 0x00)
        print(f"PCA9685 MODE1 register value: 0x{mode1:02X}")
        
        # Try to write to the MODE1 register
        print("Attempting to reset PCA9685...")
        bus.write_byte_data(address, 0x00, 0x00)  # Reset MODE1 register
        time.sleep(0.1)
        
        # Read it back
        mode1 = bus.read_byte_data(address, 0x00)
        print(f"PCA9685 MODE1 register after reset: 0x{mode1:02X}")
        
        print("PCA9685 communication successful!")
        return True
    except Exception as e:
        print(f"Error communicating with PCA9685: {e}")
        return False

def main():
    print("===== SHERPA I2C Test =====")
    
    try:
        # Check if I2C device exists
        print("Checking I2C device file...")
        try:
            import os
            if os.path.exists("/dev/i2c-1"):
                print("I2C device file exists: /dev/i2c-1")
            else:
                print("I2C device file does not exist: /dev/i2c-1")
                print("However, this might not be a problem if the kernel provides direct access")
        except Exception as e:
            print(f"Error checking I2C device file: {e}")
        
        # Run i2cdetect command
        print("\nRunning i2cdetect...")
        os.system("i2cdetect -y 1")
        
        # First scan the bus
        print("\nScanning I2C bus with Python smbus...")
        devices = scan_i2c_bus()
        
        if 0x40 in devices:
            print("\nPCA9685 found at default address (0x40)")
            test_pca9685()
        else:
            print("\nPCA9685 not found at default address (0x40)")
            if devices:
                print("Attempting to test other detected devices as possible PCA9685...")
                for addr in devices:
                    if test_pca9685(addr):
                        print(f"\nPCA9685 likely at address 0x{addr:02X} instead of 0x40")
                        print(f"You should update your configuration to use this address!")
                        break
    except Exception as e:
        print(f"Error in main function: {e}")
        traceback.print_exc()
        
    print("\n===== Hardware Connection Checklist =====")
    print("1. Ensure the PCA9685 is properly powered (3.3V or 5V)")
    print("2. Check that SDA and SCL pins are connected to the correct GPIO pins")
    print("3. Verify that pull-up resistors are present on SDA and SCL lines")
    print("4. Check for any loose connections or damaged wiring")
    print("5. Ensure the PCA9685 board is not damaged")
    
if __name__ == "__main__":
    main()
