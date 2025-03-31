#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test program for the MT6701 magnetic encoder.
"""

import time
import math
import argparse
from mt6701_driver import MT6701
import matplotlib.pyplot as plt
import numpy as np


def test_reading_speed(encoder, num_samples=100):
    """Tests the reading speed of the encoder.
    
    Args:
        encoder: MT6701 encoder instance.
        num_samples: Number of samples to measure.
    
    Returns:
        Average reading time in ms.
    """
    print(f"Measuring reading speed ({num_samples} samples)...")
    times = []
    
    for _ in range(num_samples):
        start = time.time()
        encoder.read_angle_radians()
        end = time.time()
        times.append((end - start) * 1000)  # convert to ms
    
    avg_time = sum(times) / len(times)
    print(f"Average reading time: {avg_time:.3f} ms")
    return avg_time


def visualize_readings(encoder, duration=5, rate=50):
    """Visualizes angle values over time.
    
    Args:
        encoder: MT6701 encoder instance.
        duration: Duration in seconds.
        rate: Sampling rate in Hz.
    """
    print(f"Collecting data for {duration} seconds at {rate} Hz...")
    
    angles = []
    timestamps = []
    
    start_time = time.time()
    end_time = start_time + duration
    
    while time.time() < end_time:
        angle = encoder.read_angle_radians()
        angles.append(angle)
        timestamps.append(time.time() - start_time)
        
        # Wait for next sample
        time.sleep(1.0/rate)
    
    # Visualization
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, angles, 'b-')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.title('MT6701 Angle over Time')
    plt.grid(True)
    
    # Add second Y-axis for degrees
    ax1 = plt.gca()
    ax2 = ax1.twinx()
    ax2.set_ylabel('Angle (°)')
    ax2.set_ylim(0, 360)
    
    plt.tight_layout()
    plt.savefig('mt6701_readings.png')
    print(f"Visualization saved as 'mt6701_readings.png'")
    plt.show()


def main():
    """Main function for the test program."""
    parser = argparse.ArgumentParser(description='MT6701 Encoder Test Program')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number (default: 1)')
    parser.add_argument('--address', type=int, default=0x06, help='I2C address (default: 0x06)')
    parser.add_argument('--visual', action='store_true', help='Perform visualization')
    parser.add_argument('--duration', type=int, default=5, help='Visualization duration in seconds')
    parser.add_argument('--rate', type=int, default=50, help='Sampling rate for visualization')
    
    args = parser.parse_args()
    
    try:
        print(f"Initializing MT6701 on bus {args.bus}, address 0x{args.address:02x}...")
        encoder = MT6701(bus_num=args.bus, address=args.address)
        
        # Basic functionality check
        print("Testing encoder functions...")
        
        # Read raw value
        raw_value = encoder.read_angle_raw()
        print(f"Raw value (0-4095): {raw_value}")
        
        # Angle in radians
        angle_rad = encoder.read_angle_radians()
        print(f"Angle (rad): {angle_rad:.4f}")
        print(f"Angle (°): {math.degrees(angle_rad):.2f}°")
        
        # Test reading speed
        test_reading_speed(encoder)
        
        # Test calibration
        print("\nTesting calibration...")
        print(f"Current angle: {encoder.read_angle_radians():.4f} rad")
        print("Calibrating to 0 rad...")
        encoder.calibrate(0.0)
        print(f"Angle after calibration: {encoder.read_angle_radians():.4f} rad")
        
        print("Calibrating to π/2 rad...")
        encoder.calibrate(math.pi/2)
        print(f"Angle after calibration: {encoder.read_angle_radians():.4f} rad")
        
        print("Resetting calibration...")
        encoder.reset_calibration()
        print(f"Angle after reset: {encoder.read_angle_radians():.4f} rad")
        
        # Visualization if requested
        if args.visual:
            visualize_readings(encoder, duration=args.duration, rate=args.rate)
        
        print("\nTest completed.")
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1
    
    return 0


if __name__ == "__main__":
    main()