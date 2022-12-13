#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np 

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal, MoveToJointPoseAction, MoveToJointPoseGoal, ManipulatorState
from rv_msgs.msg import ActuateGripperAction, ActuateGripperGoal
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from std_srvs.srv import Empty, SetBool

import tf
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler

# joint_poses_t0 = [0.036592972086524574, 0.31026971587618185, 0.05728292101527108, -1.7950449405872628, -0.1316667015552493, 3.4454869918911606, 0.8738273452685938]

# joint_poses_t0 = [-0.041813927371099624, 0.5327150121588193, 0.009314202842885522, -2.135148521597016, 0.020127739127791953, 2.643839102792483, 0.6894405189123418]
# joint_poses_t0 = [-0.04449284146443089, 0.4759570054162948, 0.014749742810142831, -2.1710620462417682, 0.02042605725483487, 2.643418743388098, 0.6437612784675599]
joint_poses_t0 = [-0.10140886784078447, 0.45442088723306606, -0.02734507475720127, -2.160592769957425, 0.02097944297393163, 2.644945687691275, 0.6437426534261965]

class panda_class():
    def __init__(self):
        self.robot_state_sub = rospy.Subscriber("/arm/state", ManipulatorState, self.callback,  queue_size = 2)
        self.listener = tf.TransformListener()
        self.recover_service = rospy.ServiceProxy('/arm/recover', Empty)
        self.ee_pose = [0, 0, 0, 1, 0, 0, 0]
        self.euler_angle = [0, 0, 0] # roll, pitch, yaw
        print('panda loaded')

    def callback(self, data):
        self.ee_pose = [data.ee_pose.pose.position.x, data.ee_pose.pose.position.y, data.ee_pose.pose.position.z,\
                        data.ee_pose.pose.orientation.x, data.ee_pose.pose.orientation.y, data.ee_pose.pose.orientation.z, data.ee_pose.pose.orientation.w]
        self.joint_poses = data.joint_poses
        self.euler_angle = np.array(euler_from_quaternion(self.ee_pose[3:]))
    
    def move_cart_relative(self, pos, orn, speed=0.1):
        # pos = [x,y,z] (m)
        # orn = [roll, pitch, yaw] (rad)
        if not np.array(self.ee_pose).sum()==1:
            print('moving robot!')
            target_pose = self.ee_pose.copy()
            target_pose[0] += pos[0]
            target_pose[1] += pos[1]
            target_pose[2] += pos[2]

            euler_angle = self.euler_angle.copy()
            euler_angle[0] += orn[0]
            euler_angle[1] += orn[1]
            euler_angle[2] += orn[2]
            quat_new = quaternion_from_euler(euler_angle[0], euler_angle[1], euler_angle[2])

            client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
            client.wait_for_server()
            # Create a target pose
            target = PoseStamped()
            target.header.frame_id = 'panda_link0'
            target.pose.position.x = target_pose[0]
            target.pose.position.y = target_pose[1]
            target.pose.position.z = target_pose[2]
            target.pose.orientation.x = quat_new[0]
            target.pose.orientation.y = quat_new[1]
            target.pose.orientation.z = quat_new[2]
            target.pose.orientation.w = quat_new[3]
            goal = MoveToPoseGoal(goal_pose=target, speed=speed)

            client.send_goal(goal)
            client.wait_for_result()
            # return res.result.result
        else:
            print('ee_pose not updated yet')
            # return 1

    def move_joints(self, joints, speed=None):
        client = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
        client.wait_for_server()
        # Create a target pose
        goal = MoveToJointPoseGoal(joints=joints, speed=speed)
        client.send_goal(goal)
        client.wait_for_result()

    def home(self):
        self.move_joints(joints=joint_poses_t0)
        return

    def recover(self):
        self.recover_service() 
        return

    def grasp(self, force, width, speed=0.01, e_inner=0.01, e_outer=0.01):
        client = actionlib.SimpleActionClient('/arm/gripper', ActuateGripperAction)
        client.wait_for_server()
        # Create a target pose
        mode = ActuateGripperGoal.MODE_GRASP
        goal = ActuateGripperGoal(mode=mode, width=width, e_outer=e_outer, e_inner=e_inner,speed=speed, force=force)

        client.send_goal(goal)
        client.wait_for_result()

    def open_gripper(self, width=0.1, speed=0.1):
        client = actionlib.SimpleActionClient('/arm/gripper', ActuateGripperAction)
        client.wait_for_server()
        # Create a target pose
        mode = ActuateGripperGoal.MODE_STATIC
        goal = ActuateGripperGoal(mode=mode, width=width, speed=speed)

        client.send_goal(goal)
        client.wait_for_result()