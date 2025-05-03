#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import math
import tf
from visualization_msgs.msg import Marker, MarkerArray

class Voxel3DMapper:
    def __init__(self):
        rospy.init_node('voxel_3d_mapper', anonymous=True)
        
        # Parameter
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.cloud_out_topic = rospy.get_param('~cloud_out_topic', '/cloud_in')  # Für Octomap
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.voxel_size = rospy.get_param('~voxel_size', 0.1)
        self.max_range = rospy.get_param('~max_range', 10.0)
        
        # TF Listener für Transformationen
        self.tf_listener = tf.TransformListener()
        
        # Publisher für die 3D-Punktwolke
        self.cloud_pub = rospy.Publisher(self.cloud_out_topic, PointCloud2, queue_size=2)
        
        # Publisher für Debug-Marker
        self.marker_pub = rospy.Publisher('/voxel_debug_markers', MarkerArray, queue_size=1)
        
        # Status-Publisher
        self.status_pub = rospy.Publisher('/voxel_mapper/status', std_msgs.msg.String, queue_size=1)
        
        # Subscriber für Laser-Scans
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        
        # Status-Variablen
        self.last_tf_check_time = rospy.Time(0)
        self.transform_available = False
        self.processed_scans = 0
        self.published_clouds = 0
        
        # Status-Timer
        rospy.Timer(rospy.Duration(5.0), self.publish_status)
        
        rospy.loginfo("3D Voxel Mapper gestartet. Warte auf Lokalisierung...")
    
    def publish_status(self, event=None):
        """Veröffentlicht regelmäßig Status-Informationen"""
        status_msg = std_msgs.msg.String()
        status_msg.data = f"Lokalisierung verfügbar: {self.transform_available}, " \
                         f"Verarbeitete Scans: {self.processed_scans}, " \
                         f"Veröffentlichte Punktwolken: {self.published_clouds}"
        self.status_pub.publish(status_msg)
        rospy.loginfo(status_msg.data)
        
        # Überprüfe den Status der Transformationen
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.1)
            )
            self.transform_available = True
            rospy.loginfo(f"Transformation {self.map_frame} -> {self.base_frame} verfügbar.")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.transform_available = False
            rospy.logwarn(f"Transformation nicht verfügbar: {e}")
        
    def scan_callback(self, scan_msg):
        """Verarbeitet eingehende LaserScan-Nachrichten"""
        try:
            self.processed_scans += 1
            
            # Prüfe, ob die Lokalisierung verfügbar ist (Transformation map -> base_link)
            try:
                # Überprüfe Transformation zur Scan-Zeit
                self.tf_listener.waitForTransform(
                    self.map_frame, self.base_frame,
                    scan_msg.header.stamp, rospy.Duration(0.05)
                )
                # Überprüfe, ob wir auch den Laser-Frame transformieren können
                self.tf_listener.waitForTransform(
                    self.map_frame, scan_msg.header.frame_id,
                    scan_msg.header.stamp, rospy.Duration(0.05)
                )
                
                # Wenn wir hier ankommen, ist die Lokalisierung in Ordnung
                self.transform_available = True
                
                # Verarbeite den Scan
                self.process_scan(scan_msg)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Transformation nicht verfügbar - Lokalisierung nicht zuverlässig
                self.transform_available = False
                rospy.logdebug_throttle(5.0, f"Lokalisierung nicht verfügbar: {e}")
                
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Fehler im Scan-Callback: {e}")
    
    def process_scan(self, scan_msg):
        """Verarbeitet den Scan und erstellt die 3D-Punktwolke"""
        try:
            # Bestimme den Frame des Laserscanners
            laser_frame = scan_msg.header.frame_id
            
            # Erstelle eine Punktwolke
            points_3d = []
            
            # Verarbeite jeden Laserstrahl einzeln
            angle = scan_msg.angle_min
            for i, r in enumerate(scan_msg.ranges):
                # Überspringe ungültige Messungen
                if r < scan_msg.range_min or r > scan_msg.range_max or r > self.max_range or not np.isfinite(r):
                    angle += scan_msg.angle_increment
                    continue
                
                # Berechne die x,y-Position im Laserscanner-Koordinatensystem
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                
                # Bei jedem 3. Punkt für bessere Performance
                if i % 3 == 0:
                    # Nur eine Schicht auf Höhe des Sensors
                    points_3d.append([x, y, 0.0])
                
                angle += scan_msg.angle_increment
            
            if not points_3d:
                return
            
            # Debug-Marker für visuelle Bestätigung
            marker_array = MarkerArray()
            for i, point in enumerate(points_3d[:20]):  # Nur die ersten 20 Punkte anzeigen
                marker = Marker()
                marker.header.frame_id = laser_frame
                marker.header.stamp = scan_msg.header.stamp
                marker.ns = "scan_points"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.voxel_size
                marker.scale.y = self.voxel_size
                marker.scale.z = self.voxel_size
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.lifetime = rospy.Duration(0.5)
                marker_array.markers.append(marker)
            
            # Veröffentliche die Debug-Marker
            if marker_array.markers:
                self.marker_pub.publish(marker_array)
            
            # Erstelle die PointCloud2-Nachricht
            header = std_msgs.msg.Header()
            header.stamp = scan_msg.header.stamp
            header.frame_id = laser_frame
            cloud_msg = pc2.create_cloud_xyz32(header, points_3d)
            
            # Veröffentliche die Punktwolke
            self.cloud_pub.publish(cloud_msg)
            self.published_clouds += 1
            
            rospy.loginfo_throttle(5.0, f"3D-Punktwolke mit {len(points_3d)} Punkten veröffentlicht.")
            
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Fehler bei der Verarbeitung des LaserScans: {e}")

if __name__ == '__main__':
    try:
        mapper = Voxel3DMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass