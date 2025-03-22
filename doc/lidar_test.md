Um den RPLidar zu testen, folgende Befehle ausführen:

1. Zuerst roscore in einem Terminal starten (falls noch nicht laufend):
```bash
source /opt/ros/noetic/setup.bash
roscore
```

2. In einem zweiten Terminal:
```bash
cd /home/ros/catkin_ws
source devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
roslaunch /home/ros/catkin_ws/src/rplidar_ros/launch/view_rplidar.launch
```