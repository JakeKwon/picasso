#!/usr/bin/python
# coding: utf-8
import sys

import moveit_commander
import rospy
import roscpp

def home_left_arm():
    mgc = moveit_commander.MoveGroupCommander("left_arm")
    mgc.get_current_joint_values()
    jv = mgc.get_current_joint_values()
    jv[0]=1.0

    mgc.set_joint_value_target(jv)
    mgc.plan()
    p = mgc.plan()
    mgc.execute(p)

def home_right_arm():
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    jv = mgc.get_current_joint_values()
    jv[0] = -1

    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)

def home_head():
    mgc = moveit_commander.MoveGroupCommander("head")
    jv = mgc.get_current_joint_values()
    jv[1] = 0.5
    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)    


if __name__ == "__main__":
    
    node_name = "ready_pr2"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(node_name)
    rospy.loginfo(node_name + ": is initialized")

    rc = moveit_commander.RobotCommander()
    rospy.loginfo("robot commander is initialized")

    home_left_arm()
    rospy.loginfo("left arm homed")

    home_right_arm()
    rospy.loginfo("right arm homed")

    home_head()
    rospy.loginfo("head homed")    
    
    moveit_commander.roscpp_shutdown()
