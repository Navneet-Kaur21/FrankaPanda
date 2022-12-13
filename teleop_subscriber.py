#!/usr/bin/env python3

import rospy
from robot import panda_class
import time
from std_msgs.msg import Int16
from teleop_publisher import teleop_pub


class panda_teleop(teleop_pub):

    def __init__(self):
        super().__init__()
        rospy.init_node('teleop', anonymous=True)
        print("load panda class")
        self.panda = panda_class()
        time.sleep(1.0)
        rospy.Subscriber("key_pressed", Int16, self.callback)
        rospy.spin()
        self.command = None
        
    def callback(self,data):  
        if self.stop_command != True:
            self.command = data.data
            self.teleop()
        else:
            print("Stoppped")

    def teleop(self):
        command_pressed = self.command
        if command_pressed == 120:
            print('x')
            # self.panda.move_cart_relative([0.01,0,0],[0,0,0])
            

    def listener(self):
        print('Initialised')
        

if __name__ == '__main__':
    panda_teleop = panda_teleop()
    panda_teleop.listener()

























# if k==120:
        #     # time.sleep(1)
        #     self.panda.move_cart_relative([0.01,0,0],[0,0,0])
        # elif k==99:
        #     # time.sleep(1)
        #     self.panda.move_cart_relative([-0.01,0,0],[0,0,0])
        # elif k==115:
        #     # time.sleep(1)
        #     self.panda.move_cart_relative([0,0.01,0],[0,0,0])
        # elif k==119:
        #     # time.sleep(1)
        #     self.panda.move_cart_relative([0,0,0.01],[0,0,0])
        # elif k==101:
        #     # time.sleep(1)
        #     self.panda.move_cart_relative([0,0,-0.01],[0,0,0])
        # elif k==100:
        #     # time.sleep(1)
        #     self.panda.move_cart_relative([0,-0.01,0],[0,0,0])