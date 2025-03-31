#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS node for MT6701 magnetic encoder.
"""

import rospy
import argparse
import sys
from mt6701_driver import MT6701
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse


class MT6701Node:
    """ROS node for MT6701 magnetic encoder."""
    
    def __init__(self, bus_num=1, address=0x06, node_name=None, rate=10):
        """Initializes the MT6701 node.
        
        Args:
            bus_num: I2C bus number.
            address: I2C address of the sensor.
            node_name: Name of the ROS node.
            rate: Publishing rate in Hz.
        """
        # Initialize ROS node
        self.node_name = node_name if node_name else f"mt6701_node_{address:x}"
        rospy.init_node(self.node_name, anonymous=True)
        
        # Read parameters from ROS parameter server (override defaults)
        self.bus_num = rospy.get_param('~bus_num', bus_num)
        self.address = rospy.get_param('~address', address)
        self.rate = rospy.get_param('~rate', rate)
        self.topic = rospy.get_param('~topic', f"{self.node_name}/angle")
        
        # Create MT6701 instance
        self.encoder = MT6701(
            bus_num=self.bus_num,
            address=self.address,
            node_name=self.node_name
        )
        
        # Setup publisher and services
        self.publisher = rospy.Publisher(self.topic, Float32, queue_size=10)
        
        # Service for calibration
        self.calibrate_service = rospy.Service(
            f"{self.node_name}/calibrate",
            Trigger,
            self.handle_calibrate
        )
        
        # Service for resetting calibration
        self.reset_service = rospy.Service(
            f"{self.node_name}/reset_calibration",
            Trigger,
            self.handle_reset_calibration
        )
        
        rospy.loginfo(f"MT6701 Node started: {self.node_name}")
        rospy.loginfo(f"I2C Bus: {self.bus_num}, Address: 0x{self.address:02x}")
        rospy.loginfo(f"Publishing to: {self.topic} at {self.rate} Hz")
    
    def handle_calibrate(self, req):
        """Callback for the calibration service.
        
        Args:
            req: Service request.
            
        Returns:
            TriggerResponse: Service response.
        """
        try:
            self.encoder.calibrate(0.0)  # Calibrate to 0 radians
            return TriggerResponse(
                success=True,
                message=f"Encoder {self.node_name} calibrated."
            )
        except Exception as e:
            return TriggerResponse(
                success=False,
                message=f"Calibration failed: {str(e)}"
            )
    
    def handle_reset_calibration(self, req):
        """Callback for the reset-calibration service.
        
        Args:
            req: Service request.
            
        Returns:
            TriggerResponse: Service response.
        """
        try:
            self.encoder.reset_calibration()
            return TriggerResponse(
                success=True,
                message=f"Calibration for {self.node_name} reset."
            )
        except Exception as e:
            return TriggerResponse(
                success=False,
                message=f"Reset failed: {str(e)}"
            )
            
    def run(self):
        """Main loop for the node."""
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            try:
                # Read and publish angle
                angle = self.encoder.read_angle_radians()
                self.publisher.publish(Float32(angle))
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error: {str(e)}")
                rate.sleep()


def main():
    """Main function to start the node."""
    parser = argparse.ArgumentParser(description='MT6701 Magnetic Encoder ROS Node')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number (default: 1)')
    parser.add_argument('--address', type=int, default=0x06, help='I2C address of the sensor (default: 0x06)')
    parser.add_argument('--rate', type=int, default=10, help='Publishing rate in Hz (default: 10)')
    parser.add_argument('--name', type=str, default=None, help='Name of the ROS node')
    
    # Pass arguments to ROS
    args, unknown = parser.parse_known_args()
    
    # Create and start node
    node = MT6701Node(
        bus_num=args.bus,
        address=args.address,
        node_name=args.name,
        rate=args.rate
    )
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()