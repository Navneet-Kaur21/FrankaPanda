#!/usr/bin/env python

import rospy
import timeit
import actionlib
import copy
import time
import numpy as np

from pynput.keyboard import Key, Listener
from rv_msgs.msg import ManipulatorState
from rv_msgs.msg import ActuateGripperAction, ActuateGripperGoal
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Empty

arm_current_pose = PoseStamped()
arm_target_pose = PoseStamped()
gripper_width = 0.1
step_gripper = 0.01
vel_arm = 0.02
vel_ang = 5.*np.pi/180.
step_vel_arm = 0.01
org_dir = True
rocky_move = False
cnt_rocky = 0


def get_current_pose(data):
	global arm_current_pose
	arm_current_pose.header.frame_id = data.ee_pose.header.frame_id
	arm_current_pose.pose.position.x = data.ee_pose.pose.position.x
	arm_current_pose.pose.position.y = data.ee_pose.pose.position.y
	arm_current_pose.pose.position.z = data.ee_pose.pose.position.z
	arm_current_pose.pose.orientation.x =  data.ee_pose.pose.orientation.x
	arm_current_pose.pose.orientation.y =  data.ee_pose.pose.orientation.y
	arm_current_pose.pose.orientation.z =  data.ee_pose.pose.orientation.z
	arm_current_pose.pose.orientation.w =  data.ee_pose.pose.orientation.w

# def move_arm(arm_target_pose):
# 	# Create goal from target pose
# 	goal = MoveToPoseGoal(goal_pose=arm_target_pose)
# 	# Send goal and wait for it to finish
# 	client_arm.send_goal(goal)
# 	client_arm.wait_for_result()


# def open_gripper(width=0.1, speed=0.02):
# 	mode = ActuateGripperGoal.MODE_STATIC
# 	goal = ActuateGripperGoal(mode=mode, width=width, speed=speed)
# 	client_gripper.send_goal(goal)
# 	client_gripper.wait_for_result()

def stop_arm():
	global arm_vel_msg
	arm_vel_msg.twist.linear.x = 0.0
	arm_vel_msg.twist.linear.y = 0.0
	arm_vel_msg.twist.linear.z = 0.0
	arm_vel_msg.twist.angular.x = 0.0
	arm_vel_msg.twist.angular.y = 0.0
	arm_vel_msg.twist.angular.z = 0.0

# def grasp(force, width, speed=0.01, e_inner=0.01, e_outer=0.01):
# 	global client_gripper
# 	mode = ActuateGripperGoal.MODE_GRASP
# 	goal = ActuateGripperGoal(mode=mode, width=width, e_outer=e_outer, e_inner=e_inner,speed=speed, force=force)
# 	client_gripper.send_goal(goal)
# 	client_gripper.wait_for_result()
	

def key_press(key):
	global arm_vel_msg, vel_arm, rocky_move
	if key.char == 'z':
		arm_vel_msg.twist.linear.x = vel_arm
	elif key.char == 'c':
		arm_vel_msg.twist.linear.x = -vel_arm	
	elif key.char == 'a':
		arm_vel_msg.twist.linear.y = vel_arm
	elif key.char == 'd':
		arm_vel_msg.twist.linear.y = -vel_arm
	elif key.char == 'q':
		arm_vel_msg.twist.linear.z = vel_arm
	elif key.char == 'e':
		arm_vel_msg.twist.linear.z = -vel_arm
	elif key.char == 'u':
		arm_vel_msg.twist.angular.x = vel_ang
	elif key.char == 'o':
		arm_vel_msg.twist.angular.x = -vel_ang	
	elif key.char == 'h':
		arm_vel_msg.twist.angular.y = vel_ang
	elif key.char == 'k':
		arm_vel_msg.twist.angular.y = -vel_ang
	elif key.char == 'b':
		arm_vel_msg.twist.angular.z = vel_ang
	elif key.char == 'm':
		arm_vel_msg.twist.angular.z = -vel_ang
	# elif key.char == 'm':
	# 	vel_arm = vel_arm+step_vel_arm if vel_arm+step_vel_arm<0.05 else 0.05
	# elif key.char == 'n':
	# 	vel_arm = vel_arm-step_vel_arm if vel_arm-step_vel_arm>0 else 0.0
	# elif key.char == 'g': 
	# 	grasp(force=10, width=0.04)
	# elif key.char == 'o': 
	# 	open_gripper()
	# elif key.char == 'z':
	# 	rocky_move = True		
	# elif key.char == 'q':
	# 	rock_vel = np.pi/180*5
	# 	# arm_vel_msg.twist.linear.x = 0.1 * rock_vel
	# 	# arm_vel_msg.twist.linear.y = -0.1 * rock_vel
	# 	arm_vel_msg.twist.angular.x = rock_vel
	# 	arm_vel_msg.twist.angular.y = rock_vel
	# elif key.char == 'w':
	# 	rock_vel = np.pi/180*5
	# 	# arm_vel_msg.twist.linear.x = 0.1 * rock_vel
	# 	# arm_vel_msg.twist.linear.y = -0.1 * rock_vel
	# 	arm_vel_msg.twist.angular.x = -rock_vel
	# 	arm_vel_msg.twist.angular.y = -rock_vel

	elif key == Key.esc:
		stop_arm()
		return False


def key_release(key):
	global rocky_move, cnt_rocky
	stop_arm()
	rocky_move = False
	cnt_rocky = 0
	# if key.char == 'g': 
	# 	slip_monitor.restart1 = True
	# 	slip_monitor.restart2 = True

 
if __name__ == "__main__":

	# initialise ros node
	rospy.init_node('teleop_franka')
	
	# set arm in home pose
	
	# subscribe to /arm/state topic to get current pose of the arm wrt the base link
	rospy.Subscriber("/arm/state", ManipulatorState, get_current_pose)

	rate = rospy.Rate(100)

	# create publisher to send vel command to the arm
	publisher_arm_vel = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
	arm_vel_msg = TwistStamped()
	
	recover_service = rospy.ServiceProxy('/arm/recover', Empty)
	
	listener = Listener(on_press=key_press, on_release=key_release)
	listener.start()

	while listener.running and not rospy.is_shutdown():			
		# print('x=%.3f, y=%.3f, z=%.3f'% (arm_current_pose.pose.position.x, arm_current_pose.pose.position.y, arm_current_pose.pose.position.z))
		# rospy.loginfo(arm_target_pose)
		# if rocky_move and cnt_rocky<40:
		# 	rock_vel = np.pi/180*5
		# 	arm_vel_msg.twist.angular.x = rock_vel if cnt_rocky<20 else -rock_vel
		# 	arm_vel_msg.twist.angular.y = rock_vel if cnt_rocky<20  else -rock_vel
		# 	cnt_rocky += 1
		# else:
		# 	rocky_move = False
		# 	cnt_rocky = 0
		publisher_arm_vel.publish(arm_vel_msg)
		recover_service()
		rate.sleep()

	publisher_arm_vel.publish(TwistStamped())
	publisher_arm_vel.publish(TwistStamped())
	publisher_arm_vel.publish(TwistStamped())
