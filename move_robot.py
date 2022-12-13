#!/usr/bin/env python3

import rospy
from robot import panda_class
import time
import numpy as np
from std_msgs.msg import Int16
# from moveit_msgs.msg import ExecuteTrajectoryActionResult

MAX_TIME = 30

def callback(data):
    print(data)

if __name__ == "__main__":
    rospy.init_node('contact', anonymous=True)
    pub_step = rospy.Publisher('step', Int16, queue_size=10)
    # status = rospy.Subscriber("/arm/state", ExecuteTrajectoryActionResult, callback,  queue_size = 2)

    panda = panda_class()
    time.sleep(1)
    panda.recover()
    time.sleep(0.5)
    panda.open_gripper(width=0.08)
    # time.sleep(0.1)
    # panda.home()
    # panda.home()
    # time.sleep(0.1)
    # panda.grasp(force=30, width=0.01)
    # time.sleep(0.1)
    # panda.grasp(force=10, width=0.01)
    time.sleep(0.1)


    # # panda.grasp(force=30, width=0.02)
    # time.sleep(0.1)

    # input("Waiting input:")

    # for i in range(MAX_TIME):
    #     print(f"Step {i}")
    #     if np.random.rand() < 0.6:
    #         delta_pos = np.random.uniform(low=-0.03, high=0.03, size=(3,))
    #         delta_orn = np.zeros(3)
    #         curr_pose = panda.ee_pose.copy()
    #         new_pos = curr_pose[0:3] + delta_pos
    #         while new_pos[1]>0.17 or new_pos[1]<-0.19 or new_pos[2]>0.62 or new_pos[2]<0.47 or new_pos[0]<0.72:
    #             delta_pos = np.random.uniform(low=-0.03, high=0.03, size=(3,))
    #             delta_orn = np.zeros(3)
    #             curr_pose = panda.ee_pose.copy()
    #             new_pos = curr_pose[0:3] + delta_pos
    #     else:
    #         delta_pos = np.zeros(3)
    #         delta_orn = np.random.uniform(low=-7.0, high=7.0, size=(3,)) * (np.pi/180.)
    #     panda.move_cart_relative(pos=delta_pos, orn=delta_orn, speed=0.05)
    #     pub_step.publish(i)
    #     time.sleep(0.01)
    #     panda.recover()
    # pub_step.publish(-1)
    print('end')