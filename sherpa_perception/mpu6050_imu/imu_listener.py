#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    x = msg.linear_acceleration.x
    y = msg.linear_acceleration.y
    theta = msg.angular_velocity.z
    print(f"x: {x:.3f}  y: {y:.3f}  theta: {theta:.3f}")

def main():
    rospy.init_node('imu_listener')
    rospy.Subscriber('/imu/data_raw', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
