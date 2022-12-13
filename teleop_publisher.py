#!/usr/bin/env python3

import getch
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from pynput.keyboard import Key, Listener
import sys

class teleop_pub():

    def __init__(self):
        self.teleop_publisher = None
        self.stop_command = False
        self.key_pressed = None

    def on_press(self,key):
        self.stop_command = False
        self.key_pressed = ord(getch.getch())
        print(self.key_pressed)
        self.teleop_publisher.publish(self.key_pressed)

    def on_release(self,key):
        self.stop_command = True 
        if self.key_pressed == 27:
            sys.exit()
            

    def key_thread(self):
        self.teleop_publisher = rospy.Publisher('key_pressed',Int16, queue_size=1)
        rospy.init_node('keypress', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as thread_listener:
                thread_listener.join()

if __name__ == '__main__':
    try:
        teleop_pub = teleop_pub()
        teleop_pub.key_thread()
    except rospy.ROSInterruptException:
        pass