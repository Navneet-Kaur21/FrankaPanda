#!/usr/bin/env python3

import getch
from std_msgs.msg import String
from std_msgs.msg import Int16
from pynput.keyboard import Key, Listener
import sys
from threading import *

class teleop_pub(Thread):

    def __init__(self):
        self.stop_command = False
        self.key_pressed = None
        self.count = 0

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

    def callback(self):
        while(True):
            if self.stop_command==False and self.count==0:
                if self.key_pressed == 120:
                    print('--x')
                self.count+=1

if __name__ == '__main__':
    teleop_pub = teleop_pub()
    t1 = Thread(target=teleop_pub.thread_teleop)
    t1.start()
    t2 = Thread(target=teleop_pub.callback)
    t2.start()