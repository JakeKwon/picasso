#!/usr/bin/python
# coding: utf-8
import sys
import moveit_commander
import rospy
import roscpp

import copy
import geometry_msgs.msg

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
    
    mgc.set_joint_value_target(jv)
    mgc.plan()
    p = mgc.plan()
    mgc.execute(p)   

def home_move_cartesian():
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    import IPython
    IPython.embed()
    waypoints = []
    
    # start with the current pose
    waypoints.append(mgc.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x - .1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z -=.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += .05
    waypoints.append(copy.deepcopy(wpose))

    (plan3, fraction) = mgc.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             .01,        # eef_step
                             0.0)         # jump_threshold



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

    home_move_cartesian()
    rospy.loginfo("home_move_cartesian")   
 
    #import IPython
    #IPython.embed()
    
    moveit_commander.roscpp_shutdown()

