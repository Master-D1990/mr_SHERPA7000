#!/usr/bin/env python3
"""
ROS-Knoten für MT6701 Hall-Sensoren.
Unterstützt mehrere Sensoren gleichzeitig.
"""

import rospy
from std_msgs.msg import Float32
from hmt6701_driver import MT6701, scan_i2c_devices

class MT6701Node:
    """
    ROS-Knoten für MT6701 Hall-Sensoren.
    """
    
    def __init__(self):
        """Initialisiert den ROS-Knoten und die Sensoren."""
        # ROS-Knoten initialisieren
        rospy.init_node('mt6701_node', anonymous=True)
        
        # Parameter auslesen
        self.rate_hz = rospy.get_param('~rate', 10)  # Aktualisierungsrate in Hz
        self.frame_id = rospy.get_param('~frame_id', 'hall_sensor')
        
        # Sensorkonfiguration aus Parametern laden
        sensor_configs = rospy.get_param('~sensors', [{'bus': 1, 'address': 0x06, 'name': 'sensor1'}])
        
        # Sensoren und Publisher initialisieren
        self.sensors = []
        self.publishers = {}
        
        for config in sensor_configs:
            bus = config.get('bus', 1)
            address = config.get('address', 0x06)
            name = config.get('name', f'sensor_{len(self.sensors)}')
            
            # Sensor erstellen
            sensor = MT6701(bus=bus, address=address, name=name)
            self.sensors.append(sensor)
            
            # Publisher für diesen Sensor erstellen
            topic = f'/mt6701/{name}/angle'
            pub = rospy.Publisher(topic, Float32, queue_size=10)
            self.publishers[name] = pub
            
            rospy.loginfo(f"Initialized sensor '{name}' publishing to {topic}")
    
    def run(self):
        """Hauptschleife des ROS-Knotens."""
        rate = rospy.Rate(self.rate_hz)
        
        rospy.loginfo(f"MT6701 node running at {self.rate_hz} Hz")
        
        while not rospy.is_shutdown():
            for sensor in self.sensors:
                angle = sensor.get_angle()
                
                if angle is not None:
                    # Winkel als Float32-Nachricht veröffentlichen
                    msg = Float32()
                    msg.data = float(angle)
                    self.publishers[sensor.name].publish(msg)
                    
                    rospy.logdebug(f"[{sensor.name}] Angle: {angle:.2f}°")
                else:
                    rospy.logwarn(f"[{sensor.name}] Failed to read angle")
            
            rate.sleep()
    
    def shutdown(self):
        """Aufräumen beim Herunterfahren."""
        rospy.loginfo("Shutting down MT6701 node")
        for sensor in self.sensors:
            sensor.close()

def main():
    try:
        node = MT6701Node()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in MT6701 node: {e}")

if __name__ == "__main__":
    main()