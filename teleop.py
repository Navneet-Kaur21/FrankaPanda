#!/usr/bin/env python3

import rospy
import time
import getch
import sys
from pynput.keyboard import Key, Listener
from threading import *
from robot import panda_class


class panda_teleop:
    def __init__(self):
        rospy.init_node('teleop', anonymous=True)
        self.rate = rospy.Rate(30)
        self.stop_command = False
        self.key_pressed = None
        self.count = 0
        self.panda = panda_class()
        time.sleep(1.0)
        self.panda.recover()
        time.sleep(1.0)
        self.panda.go_relative([0.05,0,0],[0,0,0])
        time.sleep(1.0)
        self.t1 = Thread(target=self.thread_teleop)
        self.t1.start()

    def on_press(self,key):
        self.stop_command = False
        self.count = 0
        self.key_pressed = ord(getch.getch())

    def on_release(self,key):
        self.stop_command = True
        if self.key_pressed == 27:
            sys.exit()
            
    def thread_teleop(self):
        with Listener(
            on_press=self.on_press,
            on_release=self.on_release) as thread_listener:
            thread_listener.join()


    def run_teleop(self):
        while not rospy.is_shutdown():
            # self.panda.move_cart_relative([0.01,0,0],[0,0,0])
            # time.sleep(1.0)
            if self.stop_command==False and self.count==0:
                if self.key_pressed == 120:
                    self.panda.go_relative([0.01,0,0],[0,0,0])
                    # print(self.key_pressed)
                self.count+=1
            self.rate.sleep()

if __name__ == '__main__':
    panda_teleop = panda_teleop()
    panda_teleop.run_teleop()