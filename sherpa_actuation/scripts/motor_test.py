#!/usr/bin/env python3
"""
Motor test for SHERPA robot - Tests each motor individually and then all motors together
"""
import rospy
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Bool
import sys

class MotorTester:
    def __init__(self):
        rospy.init_node('motor_test_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        self.twist = Twist()
        
        # Wait for publishers to be ready
        rospy.sleep(1)
        
    def stop_motors(self):
        """Stop all motors"""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Motors stopped")
        
    def emergency_stop(self, activate=True):
        """Activate or deactivate emergency stop"""
        msg = Bool()
        msg.data = activate
        self.emergency_stop_pub.publish(msg)
        if activate:
            rospy.loginfo("Emergency stop activated")
        else:
            rospy.loginfo("Emergency stop deactivated")
            
    def drive_forward(self, speed=0.5, duration=5.0):
        """Drive robot forward"""
        self.twist.linear.x = speed
        self.twist.angular.z = 0.0
        rospy.loginfo(f"Driving forward at speed {speed} for {duration} seconds")
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(duration)
        self.stop_motors()
        time.sleep(1.0)  # Pause between actions
        
    def drive_backward(self, speed=0.5, duration=5.0):
        """Drive robot backward"""
        self.twist.linear.x = -speed
        self.twist.angular.z = 0.0
        rospy.loginfo(f"Driving backward at speed {speed} for {duration} seconds")
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(duration)
        self.stop_motors()
        time.sleep(1.0)  # Pause between actions
        
    def turn_left(self, speed=0.5, duration=5.0):
        """Turn robot left (counterclockwise)"""
        self.twist.linear.x = 0.0
        self.twist.angular.z = speed
        rospy.loginfo(f"Turning left at speed {speed} for {duration} seconds")
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(duration)
        self.stop_motors()
        time.sleep(1.0)  # Pause between actions
        
    def turn_right(self, speed=0.5, duration=5.0):
        """Turn robot right (clockwise)"""
        self.twist.linear.x = 0.0
        self.twist.angular.z = -speed
        rospy.loginfo(f"Turning right at speed {speed} for {duration} seconds")
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(duration)
        self.stop_motors()
        time.sleep(1.0)  # Pause between actions
    

    def run_test_sequence(self, speed=0.5):
        """Run a complete test sequence for all motor functions"""
        rospy.loginfo("Starting motor test sequence")
        
        # Make sure emergency stop is deactivated
        self.emergency_stop(False)
        time.sleep(1.0)
        
        # Basic movement tests
        self.drive_forward(speed=speed, duration=5.0)
        self.drive_backward(speed=speed, duration=5.0)
        self.turn_left(speed=speed, duration=5.0)
        self.turn_right(speed=speed, duration=5.0)
        
        # Individual wheel tests
        
        # Test emergency stop
        rospy.loginfo("Testing emergency stop...")
        self.twist.linear.x = speed
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(1.0)
        self.emergency_stop(True)  # This should stop the motors
        time.sleep(2.0)
        self.emergency_stop(False)  # Deactivate emergency stop
        time.sleep(1.0)
        
        # Final test: speed ramp
        rospy.loginfo("Testing speed ramp up and down...")
        for i in range(10):
            self.twist.linear.x = speed * (i / 10.0)
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.3)
        for i in range(10, -1, -1):
            self.twist.linear.x = speed * (i / 10.0)
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.3)
        
        self.stop_motors()
        rospy.loginfo("Test sequence complete")

if __name__ == '__main__':
    try:
        tester = MotorTester()
        
        if len(sys.argv) > 1 and sys.argv[1] == '--speed':
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.75
            tester.run_test_sequence(speed=speed)
        else:
            print("Usage: motor_test.py [--speed SPEED_VALUE]")
            print("Example: motor_test.py --speed 0.75")
            # Get speed from parameter server if available, else use default 0.75
            speed = rospy.get_param('~speed', 0.75)
            tester.run_test_sequence(speed=speed)
            
    except rospy.ROSInterruptException:
        pass
