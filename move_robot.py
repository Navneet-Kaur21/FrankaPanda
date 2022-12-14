#!/usr/bin/env python3

import rospy
from robot import panda_class
import time
import numpy as np
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped


L = 0.03
num_samples = 50
num_trajectories = 2



if __name__ == "__main__":
    rospy.init_node('planar_contact', anonymous=True)
    # sampler_pub = rospy.Publisher('/sampler/action', String, queue_size=10)
    # status = rospy.Subscriber("/arm/state", ExecuteTrajectoryActionResult, callback,  queue_size = 2)

    panda = panda_class()
    print("loaded")
    time.sleep(1)
    # panda.recover()
    time.sleep(0.5)
    # panda.home()
    print("done")

    arm_vel_msg = TwistStamped()
    publisher_arm_vel = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
    v = 0.02

    # for t in range(150):
    arm_vel_msg.twist.linear.x = v
    arm_vel_msg.twist.linear.y = 0.0
    arm_vel_msg.twist.linear.z = 0.0
    arm_vel_msg.twist.angular.x = 0.0
    arm_vel_msg.twist.angular.y = 0.0
    arm_vel_msg.twist.angular.z = 0.0

    for t in range(150):
        publisher_arm_vel.publish(arm_vel_msg)
        time.sleep(1/30.)

    arm_vel_msg.twist.linear.x = 0.0
    arm_vel_msg.twist.linear.y = 0.0
    arm_vel_msg.twist.linear.z = 0.0
    arm_vel_msg.twist.angular.x = 0.0
    arm_vel_msg.twist.angular.y = 0.0
    arm_vel_msg.twist.angular.z = 0.0
    publisher_arm_vel.publish(arm_vel_msg)
    

    # time.sleep(2.1)
    # panda.open_gripper()
    # time.sleep(2.0)
    # panda.close_gripper(force=10, width=0.01)
    # time.sleep(0.1)
   


    # publisher_arm_vel = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
    # arm_vel_msg = TwistStamped()
    # arm_vel_msg.twist.linear.x = 0.0
    # arm_vel_msg.twist.linear.y = 0.0
    # arm_vel_msg.twist.linear.z = -0.02
    # arm_vel_msg.twist.angular.x = 0.0
    # arm_vel_msg.twist.angular.y = 0.0
    # arm_vel_msg.twist.angular.z = 0.0

    # input("Press ENTER to start collecting trajectories ...")
    # sampler_pub.publish("start")
    # sampler_pub.publish("reset")

    # for i in range(150):
    #     publisher_arm_vel.publish(arm_vel_msg)
    #     time.sleep(1/30.)


    # sampler_pub.publish("record")
    # arm_vel_msg.twist.linear.x = 0.0
    # arm_vel_msg.twist.linear.y = 0.0
    # arm_vel_msg.twist.linear.z = 0.0
    # arm_vel_msg.twist.angular.x = 0.0
    # arm_vel_msg.twist.angular.y = 0.0
    # arm_vel_msg.twist.angular.z = 0.0
    # publisher_arm_vel.publish(arm_vel_msg)


    # target_pos = [0.6922287751733456, -0.00433686424461926, 0.5358276947801202]
    # target_orn = [0.019702270954605975, 0.00850235153422163, 0.004654963793949636, 0.999758901860692]
    # panda.go_absolute(target_pos, target_orn)
    # time.sleep(0.1)

    # # compute boundaries for planar movement
    # r_pos, _ = panda.get_pose()
    # x_min = r_pos[0] - 0.05
    # x_max = r_pos[0] + 0.2
    # y_min = r_pos[1] - 0.2
    # y_max = r_pos[1] + 0.2

    
    # input("Press ENTER to start collecting trajectories ...")
    # sampler_pub.publish("start")

    # k = 0
    # while k<num_trajectories:
    #     print(f"Collecting trajectory {k} ... ")

    #     panda.recover()
    #     time.sleep(0.1)
    #     panda.home()
    #     time.sleep(0.1)
    #     panda.go_relative(pos=[0,0,-0.015], orn=[0,0,0], speed=0.01)
    #     time.sleep(0.1)
    #     init_r_pos, init_r_orn = panda.get_pose(euler=True)
    #     sampler_pub.publish("reset")
        
    #     for i in range(num_samples):
    #         print(f"Step {i}")
    #         r_pos, r_orn = panda.get_pose(euler=True)
    #         delta_pos = np.random.uniform(low=-L, high=L, size=(2,))
    #         new_rpos = r_pos + np.concatenate([delta_pos, np.array([0])])
    #         new_rpos[0] = np.clip(new_rpos[0], x_min, x_max)
    #         new_rpos[1] = np.clip(new_rpos[1], y_min, y_max)
    #         new_rpos[2] = np.clip(new_rpos[2], 0.0, init_r_pos[2]*1.1)
    #         delta_orn = np.random.choice(np.arange(-7,8,1)) * (np.pi/180.)
    #         new_rorn = np.array([init_r_orn[0], init_r_orn[1], delta_orn])   #roll, pitch, yaw
    #         print(f"Pose = {new_rpos}")
    #         print(f"Orn = {new_rorn}")
    #         panda.go_absolute(target_pos=new_rpos, target_orn=new_rorn)
    #         time.sleep(0.01)
            
    #     k += 1
    #     sampler_pub.publish("record")


    

    # # for i in range(MAX_TIME):
    # #     print(f"Step {i}")
    # #     if np.random.rand() < 0.6:
    # #         delta_pos = np.random.uniform(low=-0.03, high=0.03, size=(3,))
    # #         delta_orn = np.zeros(3)
    # #         curr_pose = panda.ee_pose.copy()
    # #         new_pos = curr_pose[0:3] + delta_pos
    # #         while new_pos[1]>0.17 or new_pos[1]<-0.19 or new_pos[2]>0.62 or new_pos[2]<0.47 or new_pos[0]<0.72:
    # #             delta_pos = np.random.uniform(low=-0.03, high=0.03, size=(3,))
    # #             delta_orn = np.zeros(3)
    # #             curr_pose = panda.ee_pose.copy()
    # #             new_pos = curr_pose[0:3] + delta_pos
    # #     else:
    # #         delta_pos = np.zeros(3)
    # #         delta_orn = np.random.uniform(low=-7.0, high=7.0, size=(3,)) * (np.pi/180.)
    # #     panda.move_cart_relative(pos=delta_pos, orn=delta_orn, speed=0.05)
    # #     pub_step.publish(i)
    # #     time.sleep(0.01)
    # #     panda.recover()
    # # pub_step.publish(-1)
    # print('end')