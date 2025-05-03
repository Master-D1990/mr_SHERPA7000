#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
import struct

def publish_simple_voxels():
    rospy.init_node('simple_voxel_publisher', anonymous=True)
    cloud_pub = rospy.Publisher('/cloud_in', PointCloud2, queue_size=1)
    
    # Veröffentliche die TF-Transformation (statisch)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "base_link"
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0
    static_broadcaster.sendTransform(static_transformStamped)
    
    rate = rospy.Rate(1)  # 1 Hz
    
    # Erstelle 3 einfache Voxel-Positionen
    points = [
        [0.0, 0.0, 0.0],   # Ursprung
        [1.0, 0.0, 0.0],   # 1 Meter in X-Richtung
        [0.0, 1.0, 0.0]    # 1 Meter in Y-Richtung
    ]
    
    rospy.loginfo("Starte Veröffentlichung von 3 Voxeln für Octomap")
    
    while not rospy.is_shutdown():
        # Erstelle die PointCloud2-Nachricht
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        
        # Konvertiere die Liste zu einer PointCloud2-Nachricht
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        
        # Veröffentliche die Punktwolke
        cloud_pub.publish(cloud_msg)
        rospy.loginfo("3 Voxel-Punkte veröffentlicht")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_simple_voxels()
    except rospy.ROSInterruptException:
        pass