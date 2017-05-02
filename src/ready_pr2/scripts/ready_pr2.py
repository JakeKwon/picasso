#!/usr/bin/python
# coding: utf-8


#things to do
# calibration: edges to waypoints ratio



import sys
import moveit_commander
import rospy
import roscpp
import roslaunch

import copy
import geometry_msgs.msg

# edge detection
import cv2
import numpy as np
import transform_points as t
import nearest_neighbor as n

def rgb_to_cmy(img):
    #cmy = np.matrix(np.ones(colors.shape))
    cmy = 1.0-img/256.0
    min_cmy=np.min(cmy,axis=2)
    for i in range(3):

      cmy[:,:,i] = (cmy[:,:,i]-min_cmy)/(1.0-min_cmy)
    return (cmy*255).astype('uint8')

def split_channels(img):
    cmy = rgb_to_cmy(img)
    c = np.copy(cmy)
    c[:,:,1:] = 0
    m = np.copy(cmy)
    m[:,:,0] = 0
    m[:,:,2] = 0
    y = np.copy(cmy)
    y[:,:,:2] = 0
    return [c,m,y]

def edgeDetect(img):
    img = cv2.flip(img,0)
    img = cv2.blur(img,(5,5))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,2000,2000,apertureSize =7)
    
    lines = cv2.HoughLinesP(edges,1,np.pi/90,10,10,40,10)
    img = np.ones(img.shape)
    for x1,y1,x2,y2 in lines[0]:
        cv2.line(img,(x1,y1),(x2,y2),(0,0,0),2)
    #print lines[0].shape
    #cv2.imshow('img',img)
    #cv2.waitKey()
    return lines[0],np.max(img.shape[0:2])
'''
    img = cv2.imread(imgFile)
    img = cv2.flip(img,0)
    img = cv2.blur(img,(3,3))
    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)

    lines = cv2.HoughLinesP(edges,1,np.pi/90,10,10,35,10)

    return lines[0],np.max(img.shape[0:2])
'''
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
def get_position(pose):
    vector = np.zeros(3)
    vector[0] = pose.position.x
    vector[1] = pose.position.y
    vector[2] = pose.position.z
    return vector
def home_move_cartesian(edges, topLeft, bottomLeft, bottomRight, max_side):
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    mgc.set_goal_orientation_tolerance(0.5)
    #import IPython
    #IPython.embed()

   
    waypoints = []
    



    # start with the current pose
    waypoints.append(mgc.get_current_pose().pose)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    
    a = np.array([0,0.8,.5])
    b = np.array([0,.5,.5])
    c = np.array([.3,.5,.5])
    
    orientation = topLeft.orientation


    #transformed_edges = t.transform_points(a, b, c, reshaped_edges.T)
    topLeft = get_position(topLeft)
    bottomLeft = get_position(bottomLeft)
    bottomRight = get_position(bottomRight)

    sorted_edges = n.sort_lines(edges)
    reshaped_edges = n.reshape(sorted_edges)
    min_side = min(np.linalg.norm(topLeft-bottomLeft),np.linalg.norm(bottomRight-bottomLeft))
    reshaped_edges *= (min_side/max_side)

    transformed_edges = t.transform_points(topLeft, bottomLeft, bottomRight,reshaped_edges.T)
    
    
    for i in range(transformed_edges.shape[1]):
	wpose.position.x = transformed_edges[0,i]
    	wpose.position.y = transformed_edges[1,i]
    	wpose.position.z = transformed_edges[2,i]
        wpose.orientation = orientation
    	waypoints.append(copy.deepcopy(wpose))
        if(i%2==1):
            wpose.position.x -=.03
            waypoints.append(copy.deepcopy(wpose))
    #print waypoints
    
  

    '''
    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x - .1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))
    print wpose.position
    # second move down
    wpose.position.z -=.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += .05
    waypoints.append(copy.deepcopy(wpose))
    '''



    (plan3, fraction) = mgc.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             .001,        # eef_step
                             0.0)         # jump_threshold
    print fraction
    
    mgc.execute(plan3)
    #rospy.sleep(5)

def gripper():
    mgc = moveit_commander.MoveGroupCommander("right_gripper")
    jv = mgc.get_current_joint_values()
    #jv = [1.0, 1.0, 0.477, 0.477, 0.477, 0.477, 0.04]
    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)

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
    
    #gripper()
    #rospy.loginfo("gripper")
    


    #home_right_arm()
    #rospy.loginfo("right arm homed")
    
    #home_head()
    #rospy.loginfo("head homed")
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,
     ["/opt/ros/indigo/share/pr2_mannequin_mode/launch/pr2_mannequin_mode.launch"])
    launch.start()
    
    raw_input('press enter to get top left corner: ')
    topLeft = get_corners()

    raw_input('press enter to get bottom left corner: ')
    bottomLeft = get_corners()

    raw_input('press enter to get bottom right corner: ')
    bottomRight = get_corners()

    launch.shutdown()
    #raw_input('press enter after killing mannequin mode and initiating joystick mode: ')
    

    img = cv2.imread('dave.jpg')
    cmy = split_channels(img)
    for i in range(3):
        edges,max_side = edgeDetect(cmy[i])
        home_move_cartesian(edges, topLeft, bottomLeft, bottomRight, max_side)
        rospy.loginfo("finished color")
        raw_input('press enter after changing color') 
    #edges,max_side = edgeDetect('dave.jpg')

      

    #import IPython
    #IPython.embed()
    
    moveit_commander.roscpp_shutdown()

