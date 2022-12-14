#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np 

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal, MoveToJointPoseAction, MoveToJointPoseGoal, ManipulatorState
from rv_msgs.msg import ActuateGripperAction, ActuateGripperGoal
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from std_srvs.srv import Empty, SetBool

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# joint_poses_t0 = [0.036592972086524574, 0.31026971587618185, 0.05728292101527108, -1.7950449405872628, -0.1316667015552493, 3.4454869918911606, 0.8738273452685938]

# joint_poses_t0 = [-0.041813927371099624, 0.5327150121588193, 0.009314202842885522, -2.135148521597016, 0.020127739127791953, 2.643839102792483, 0.6894405189123418]
# joint_poses_t0 = [-0.04449284146443089, 0.4759570054162948, 0.014749742810142831, -2.1710620462417682, 0.02042605725483487, 2.643418743388098, 0.6437612784675599]
# joint_poses_t0 = [-0.09901, 0.408706, 0.035858, -2.33048, -0.009225, 2.75008, 0.592313] # cube
# joint_poses_t0 = [
#             -0.0807112151808015,
#             0.12689950624857624,
#             0.20026483563505107,
#             -2.1055931307302225 + 0.1,
#             0.20446346430420115,
#             3.6963584305710264,
#             0.6289109912481573,
#         ]

joint_poses_t0 = [-1.61, -1.199, 1.557, -2.18, 1.084, 3.2078, 0.95]


class panda_class():
    def __init__(self):
        self.robot_state_sub = rospy.Subscriber("/arm/state", ManipulatorState, self.state_callback,  queue_size = 2)
        self.listener = tf.TransformListener()
        self.recover_service = rospy.ServiceProxy('/arm/recover', Empty)
        self.ee_pose = [0, 0, 0, 0, 0, 0, 1] #[x,y,z,qx,qy,qz,qw]
        self.euler_angle = [0, 0, 0] # x,y,z (roll, pitch, yaw)
        self.panda_vel_msg = TwistStamped()

    def state_callback(self, data):
        self.ee_pose = [data.ee_pose.pose.position.x, data.ee_pose.pose.position.y, data.ee_pose.pose.position.z,\
                        data.ee_pose.pose.orientation.x, data.ee_pose.pose.orientation.y, data.ee_pose.pose.orientation.z, data.ee_pose.pose.orientation.w]
        self.joint_poses = data.joint_poses
        self.euler_angle = np.array(euler_from_quaternion(self.ee_pose[3:]))
        # print(self.ee_pose)

    def move_joints(self, joints, speed=None):
        client = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
        print(client)
        client.wait_for_server()
        print("server done")
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

    def close_gripper(self, force, width, speed=0.01, e_inner=0.01, e_outer=0.01):
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

    def get_pose(self, euler=False):
        if euler:
            # euler angle in roll, pitch, yaw
            return self.ee_pose[0:3], self.euler_angle
        else:
            # return quaternion qx, qy, qz, qw
            return self.ee_pose[0:3], self.ee_pose[3:0]

    def go_absolute(self, target_pos, target_orn, speed=0.1):
        # speed in m/s
        new_pos = np.array(target_pos)
        if len(target_orn)==4:
            # in quaternion: qx, qy, qz, qw
            new_orn = np.array(target_orn)
        else:
            # in euler: roll, pitch, yaw
            new_orn = quaternion_from_euler(target_orn[0], target_orn[1], target_orn[2])
        target_pose = np.concatenate([new_pos, new_orn])
        client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
        client.wait_for_server()
        # Create a target pose
        target = PoseStamped()
        target.header.frame_id = 'panda_link0'
        target.pose.position.x = target_pose[0]
        target.pose.position.y = target_pose[1]
        target.pose.position.z = target_pose[2]
        target.pose.orientation.x = target_pose[3]
        target.pose.orientation.y = target_pose[4]
        target.pose.orientation.z = target_pose[5]
        target.pose.orientation.w = target_pose[6]
        goal = MoveToPoseGoal(goal_pose=target, speed=speed)
        client.send_goal(goal)
        client.wait_for_result()

    def go_relative(self, pos, orn, speed=0.1):
        # pos = [x,y,z]
        # orn = [yaw, pitch, roll]
        assert self.ee_pose!=[0, 0, 0, 0, 0, 0, 1], f"End effector state not updated yet"
        target_pose = self.ee_pose.copy()
        target_pose[0] += pos[0]
        target_pose[1] += pos[1]
        target_pose[2] += pos[2]

        euler_angle = self.euler_angle.copy()
        if len(orn)==4:
            # orientation in quaternion
            orn = euler_from_quaternion(orn)
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


    def 
        arm_vel_msg.twist.linear.x = 0.0
        arm_vel_msg.twist.linear.y = 0.0
        arm_vel_msg.twist.linear.z = 0.0
        arm_vel_msg.twist.angular.x = 0.0
        arm_vel_msg.twist.angular.y = 0.0
        arm_vel_msg.twist.angular.z = 0.0

