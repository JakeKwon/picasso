#!/usr/bin/python
# coding: utf-8


#things to do
# calibration: edges to waypoints ratio



import sys
import moveit_commander
import rospy
import roscpp

import copy
import geometry_msgs.msg

# edge detection
import cv2
import numpy as np
import transform_points
import nearest_neighbor


def edgeDetect(imgFile):
    img = cv2.imread(imgFile)
    img = cv2.blur(img,(3,3))
    #cv2.imshow('dave',img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    
    #cv2.imshow('edges',edges)

    lines = cv2.HoughLinesP(edges,1,np.pi/90,10,10,35,10)


    #for x1,y1,x2,y2 in lines[0]:
    #    cv2.line(img,(x1,y1),(x2,y2),(0,0,0),2)
    
    #cv2.imshow('houghlines3.jpg',img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    print lines[0]
    return lines[0]

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

def get_corners():
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    print mgc.get_current_pose().pose
    return mgc.get_current_pose().pose

def home_move_cartesian(edges, bottomLeft, topLeft, topRight):
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    #import IPython
    #IPython.embed()

    print "------------------------------------------"
    print transform_points(np.eye(4),np.eye(4))




    waypoints = []
    




    # start with the current pose
    waypoints.append(mgc.get_current_pose().pose)

    
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    

    wpose.position.x = waypoints[0].position.x - .1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))




    # first orient gripper and move forward (+x)
    #wpose = geometry_msgs.msg.Pose()
    #wpose.orientation.w = 1.0
    #wpose.position.x = waypoints[0].position.x - .1
    #wpose.position.y = waypoints[0].position.y
    #wpose.position.z = waypoints[0].position.z
    #waypoints.append(copy.deepcopy(wpose))

    # second move down
    #wpose.position.z -=.10
    #waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    #wpose.position.y += .05
    #waypoints.append(copy.deepcopy(wpose))




    (plan3, fraction) = mgc.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             .01,        # eef_step
                             0.0)         # jump_threshold



if __name__ == "__main__":
    
    # imgFile commandline goes herer

    node_name = "ready_pr2"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(node_name)
    rospy.loginfo(node_name + ": is initialized")

    rc = moveit_commander.RobotCommander()
    rospy.loginfo("robot commander is initialized")

    #home_left_arm()
    #rospy.loginfo("left arm homed")
    
    home_right_arm()
    rospy.loginfo("right arm homed")
    
    #home_head()
    #rospy.loginfo("head homed")
    

    edges = edgeDetect('dave.jpg')
    
    raw_input('press enter to get bottom left corner: ')
    bottomLeft = get_corners()
    
    raw_input('press enter to get top left corner: ')
    topLeft = get_corners()

    raw_input('press enter to get top right corner: ')
    topRight = get_corners()

    home_move_cartesian(edges, bottomLeft, topLeft, topRight)
    rospy.loginfo("home_move_cartesian")   

    #import IPython
    #IPython.embed()
    
    moveit_commander.roscpp_shutdown()

