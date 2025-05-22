#!/usr/bin/env python3

import pynput
from pynput.keyboard import Key, KeyCode

import rospy
from geometry_msgs.msg import Twist


class Teleop:

    directions = {
        KeyCode(char='w'): (1, 0, 0),   # vorwärts
        KeyCode(char='s'): (-1, 0, 0),  # rückwärts
        KeyCode(char='a'): (0, 1, 0),   # links (seitlich)
        KeyCode(char='d'): (0, -1, 0),  # rechts (seitlich)
        KeyCode(char='q'): (0, 0, 1),   # drehen links
        KeyCode(char='e'): (0, 0, -1),  # drehen rechts
    }

    def __init__(self):
        self._is_running = True
        self._listener = pynput.keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self._listener.start()
        self._command = [0.0, 0.0, 0.0]  # [x, y, z]

        self._rates = {
            'w': rospy.get_param('~forward_rate', 2.0),
            's': rospy.get_param('~backward_rate', 1.0),
            'a': rospy.get_param('~strafe_rate', 1.0),
            'd': rospy.get_param('~strafe_rate', 1.0),
            'q': rospy.get_param('~rotation_rate', 1.0),
            'e': rospy.get_param('~rotation_rate', 1.0)
        }
        self._cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self._cmd_pub = rospy.Publisher(self._cmd_topic, Twist, queue_size=10)

    def publish(self):
        msg = Twist()
        msg.linear.x = self._command[0]
        msg.linear.y = self._command[1]
        msg.angular.z = self._command[2]
        self._cmd_pub.publish(msg)

    def is_running(self):
        return self._is_running

    def _on_press(self, key):
        if key == Key.esc:
            self._is_running = False
        if key in Teleop.directions:
            cmd = Teleop.directions[key]
            # x, y, z
            if cmd[0] != 0:
                self._command[0] = cmd[0] * self._rates.get(key.char, 1.0)
            if cmd[1] != 0:
                self._command[1] = cmd[1] * self._rates.get(key.char, 1.0)
            if cmd[2] != 0:
                self._command[2] = cmd[2] * self._rates.get(key.char, 1.0)

    def _on_release(self, key):
        if key in Teleop.directions:
            cmd = Teleop.directions[key]
            if cmd[0] != 0:
                self._command[0] = 0
            if cmd[1] != 0:
                self._command[1] = 0
            if cmd[2] != 0:
                self._command[2] = 0

    def __str__(self):
        return f"Forward: {self._command[0]}, Strafe: {self._command[1]}, Rotate: {self._command[2]}"


def main():
    print("""
    Sherpa Teleop Steuerung:
    -----------------------
    W/S: vor/zurück
    A/D: links/rechts (seitlich)
    Q/E: drehen links/rechts
    ESC: Beenden
    """)
    rospy.init_node("better_teleop")
    teleop = Teleop()

    rate = rospy.Rate(20)
    while teleop.is_running():
        teleop.publish()
        rate.sleep()

    return 0


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
