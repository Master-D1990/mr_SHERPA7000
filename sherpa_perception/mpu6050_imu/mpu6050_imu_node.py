#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math
import time

try:
    import smbus
except ImportError:
    import smbus2 as smbus

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Axis mapping: y = forward, x = left

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def main():
    rospy.init_node('mpu6050_imu_node')
    pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(50)
    bus = smbus.SMBus(1)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    time.sleep(0.1)

    accel_scale = 16384.0  # LSB/g
    gyro_scale = 131.0     # LSB/(deg/s)

    while not rospy.is_shutdown():
        # Read accelerometer
        acc_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)
        acc_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H+2)
        acc_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H+4)
        # Read gyroscope
        gyro_x = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)
        gyro_y = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H+2)
        gyro_z = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H+4)

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'
        # ROS: x=vorwärts, y=links, z=oben
        # Sensor: x=rechts, y=vorwärts, z=oben
        # ROS: x=vorwärts, y=links, z=oben
        # Mapping: Sensor-y -> ROS-x (vorwärts), Sensor-x -> ROS-y (links)
        imu_msg.linear_acceleration.x = acc_y / accel_scale  # y->x (vorwärts)
        imu_msg.linear_acceleration.y = acc_x / accel_scale  # x->y (links)
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = math.radians(gyro_z / gyro_scale)  # theta (Drehung)
        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
