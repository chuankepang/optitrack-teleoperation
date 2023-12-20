#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf


moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('reset_pose', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
arm_group.set_joint_value_target([-0.0005140807405332737, 5.380206629408086,
                                  -1.826281681279279, -5.982128024354565, 6.283091916450444, -2.2773248255377094])
arm_group.go(wait=True)

moveit_commander.roscpp_initializer.roscpp_shutdown()
