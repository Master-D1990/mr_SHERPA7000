#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Driver for the MT6701 magnetic encoder via I2C.
"""

import rospy
import math
import time
import smbus2
from std_msgs.msg import Float32


class MT6701:
    """Driver for the MT6701 magnetic encoder via I2C.
    
    This class provides functionality to read angles in radians (0 to 2π)
    from the MT6701 magnetic encoder sensor.
    """
    
    # I2C address of MT6701
    DEFAULT_ADDR = 0x06
    
    # Register addresses
    REG_ANGLE_H = 0x03
    REG_ANGLE_L = 0x04
    
    def __init__(self, bus_num=1, address=DEFAULT_ADDR, node_name=None):
        """Initializes the MT6701 encoder.
        
        Args:
            bus_num: I2C bus number (default: 1).
            address: I2C address of the sensor (default: 0x06).
            node_name: Name for ROS node (or None for default).
        """
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        self.node_name = node_name if node_name else f"mt6701_{address:x}"
        self.publisher = None
        self.offset = 0.0  # Calibration offset in radians
        
    def read_angle_raw(self):
        """Reads the raw angle (0-4095) from the sensor.
        
        Returns:
            Raw angle value (0-4095).
        """
        try:
            # 12-bit resolution (0-4095) from two registers
            angle_h = self.bus.read_byte_data(self.address, self.REG_ANGLE_H)
            angle_l = self.bus.read_byte_data(self.address, self.REG_ANGLE_L)
            
            # Combine bytes to a 12-bit value
            angle_raw = ((angle_h & 0x0F) << 8) | angle_l
            return angle_raw
        except Exception as e:
            rospy.logerr(f"Error reading encoder (address 0x{self.address:02x}): {e}")
            return 0
    
    def read_angle_radians(self):
        """Reads the angle in radians (0 to 2π).
        
        Returns:
            Angle in radians (0 to 2π).
        """
        angle_raw = self.read_angle_raw()
        # Conversion: 0-4095 → 0-2π
        angle_rad = (angle_raw / 4096.0) * 2.0 * math.pi
        
        # Apply offset
        angle_rad = (angle_rad + self.offset) % (2.0 * math.pi)
        
        return angle_rad
    
    def calibrate(self, reference_angle=0.0):
        """Calibrates the sensor so that the current position
        corresponds to the specified reference angle.
        
        Args:
            reference_angle: Reference angle in radians.
        """
        current_angle = self.read_angle_radians()
        # Calculate offset so that current position = reference angle
        self.offset = (reference_angle - current_angle) % (2.0 * math.pi)
        rospy.loginfo(f"Encoder {self.node_name} calibrated. Offset: {self.offset} rad")
    
    def reset_calibration(self):
        """Resets the calibration."""
        self.offset = 0.0
        rospy.loginfo(f"Calibration for encoder {self.node_name} reset")
    
    def start_publishing(self, topic=None, rate=10):
        """Starts publishing angle values as ROS topic.
        
        Args:
            topic: Topic name (or None for default).
            rate: Update rate in Hz.
            
        Returns:
            True if successfully started.
        """
        if self.publisher is not None:
            rospy.logwarn(f"Encoder {self.node_name} is already publishing")
            return False
        
        # Set topic name
        topic_name = topic if topic else f"{self.node_name}/angle"
        
        # Create publisher
        self.publisher = rospy.Publisher(topic_name, Float32, queue_size=10)
        rospy.loginfo(f"Encoder {self.node_name} publishing to topic: {topic_name}")
        
        # Start publishing loop
        def publish_loop():
            r = rospy.Rate(rate)
            while not rospy.is_shutdown():
                angle = self.read_angle_radians()
                self.publisher.publish(Float32(angle))
                r.sleep()
        
        # Start thread
        import threading
        self.publish_thread = threading.Thread(target=publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        return True
    
    def stop_publishing(self):
        """Stops publishing angle values.
        
        Returns:
            True if publishing was stopped.
        """
        if hasattr(self, 'publish_thread') and self.publish_thread.is_alive():
            # Thread stop is handled by rospy.is_shutdown() within the thread
            self.publisher = None
            rospy.loginfo(f"Encoder {self.node_name} stops publishing")
            return True
        return False