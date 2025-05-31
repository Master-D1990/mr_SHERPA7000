#!/usr/bin/env python3
"""
Simple test script for checking I2C functionality
"""
import os
import subprocess
import sys

def run_command(cmd):
    print(f"Running command: {cmd}")
    try:
        result = subprocess.run(cmd, shell=True, check=False, 
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True)
        print("STDOUT:", result.stdout)
        print("STDERR:", result.stderr)
        print(f"Exit code: {result.returncode}")
        return result
    except Exception as e:
        print(f"Error running command: {e}")
        return None

def check_i2c_device():
    print("===== I2C Device Check =====")
    
    # Check if I2C device exists
    if os.path.exists("/dev/i2c-1"):
        print("I2C device file exists: /dev/i2c-1")
    else:
        print("I2C device file does not exist: /dev/i2c-1")
    
    # Run i2cdetect to list available buses
    print("\n--- Available I2C buses ---")
    run_command("i2cdetect -l")
    
    # Scan I2C bus 1
    print("\n--- Scanning I2C bus 1 ---")
    run_command("i2cdetect -y 1")
    
if __name__ == "__main__":
    try:
        check_i2c_device()
    except Exception as e:
        print(f"Unhandled exception: {e}")
        import traceback
        traceback.print_exc()
