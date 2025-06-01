#!/usr/bin/env python3
"""
Python-Startskript für Teleop auf dem Desktop
Setzt die ROS-Variablen für den Sherpa-Roboter und startet Teleop
"""
import os
import subprocess
import socket

# Sherpa (Pi) IP-Adresse
os.environ["ROS_MASTER_URI"] = "http://192.168.43.59:11311"
# Eigene (Desktop-)IP automatisch ermitteln
hostname = socket.gethostname()
desktop_ip = socket.gethostbyname(hostname)
os.environ["ROS_IP"] = desktop_ip

# Starte Teleop-Launchfile
subprocess.run(["roslaunch", "sherpa_teleop", "sherpa_teleop.launch"])
