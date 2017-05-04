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
import scipy

def sort_lines(mat):
    num_lines = mat.shape[0]

    #mat = np.random.rand(num_lines,4)
    visited = np.zeros(num_lines).astype(bool)
    nearest_neighbor = np.zeros(num_lines).astype(int)-1
    point1 = mat[:,:2]
    point2 = mat[:,2:]
    middle = np.add(point1,point2)/2
    curr_point = 0

    #traverse the midpoints to find the next closest midpoint
    sorted_mat=np.zeros((num_lines,4))
    while np.sum(visited.astype(int))<num_lines-1:
        sorted_mat[np.sum(visited.astype(int)),:]= mat[curr_point]

        visited[curr_point]=True
        idx = np.arange(num_lines)[np.logical_not(visited)]
        dist = np.linalg.norm(middle[np.logical_not(visited)]-middle[curr_point],axis=1)
        nn = idx[np.argmin(dist)]

        nearest_neighbor[curr_point]=nn
        curr_point = nn

    sorted_mat[-1]= mat[np.logical_not(visited)]
    #print sorted_mat

    #sort the endpoints so that the closer points are first
    for  i in range(num_lines-1):
        smaller = np.argmin( np.linalg.norm(np.subtract(sorted_mat[i,2:],sorted_mat[i+1,:].reshape(2,2)),axis=1))
        if smaller:

            smaller = sorted_mat[i+1,2:].copy()
            sorted_mat[i+1,2:]=sorted_mat[i+1,:2].copy()
            sorted_mat[i+1,:2] = smaller

    return sorted_mat

def reshape(mat):
    mat = np.reshape(mat,(mat.shape[0]*2,2))
    mat = np.concatenate((mat,np.zeros((mat.shape[0],1))),axis=1)
    return mat
def transform_points(a,b,c,mat):
    i = (c-b)/np.linalg.norm(b-c)
    j = (a-b)/np.linalg.norm(a-b)
    k = np.cross(i,j)
    rot_mat = np.matrix([i,j,k]).T
    rotated_img = np.dot(rot_mat,mat)    
    return np.add(rotated_img,np.matrix(b).T)

def rgb_to_cmy(img):
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
    return lines[0],np.max(img.shape[0:2])

def get_corners():
    '''returns the current pose of the end effector'''
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    print mgc.get_current_pose().pose
    return mgc.get_current_pose().pose
def get_position(pose):
    '''returns the position of the end effector as a vector'''
    vector = np.zeros(3)
    vector[0] = pose.position.x
    vector[1] = pose.position.y
    vector[2] = pose.position.z
    return vector
def home_move_cartesian(edges, topLeft, bottomLeft, bottomRight, max_side):
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    mgc.set_goal_orientation_tolerance(0.5)
  
	waypoints = []
    
    # start with the current pose
    waypoints.append(mgc.get_current_pose().pose)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    orientation = topLeft.orientation
    
    #get positions of each corner as a vector
    topLeft = get_position(topLeft)
    bottomLeft = get_position(bottomLeft)
    bottomRight = get_position(bottomRight)
    
    #sort the lines for planning 
    sorted_edges = sort_lines(edges)
    reshaped_edges = reshape(sorted_edges)
    #transform the edges so that they are in 3D space
    min_side = min(np.linalg.norm(topLeft-bottomLeft),np.linalg.norm(bottomRight-bottomLeft))
    reshaped_edges *= (min_side/max_side)
    transformed_edges = transform_points(topLeft, bottomLeft, bottomRight,reshaped_edges.T)
    
    #add lines as waypoints
    for i in range(transformed_edges.shape[1]):
	wpose.position.x = transformed_edges[0,i]
    	wpose.position.y = transformed_edges[1,i]
    	wpose.position.z = transformed_edges[2,i]
        wpose.orientation = orientation
    	waypoints.append(copy.deepcopy(wpose))
        if(i%2==1):
            wpose.position.x -=.03
            waypoints.append(copy.deepcopy(wpose))  


    #path planning
    (plan3, fraction) = mgc.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             .001,        # eef_step
                             0.0)         # jump_threshold
    print fraction
    
    mgc.execute(plan3)
    #rospy.sleep(5)

if __name__ == "__main__":
    
    # imgFile commandline goes here

    node_name = "ready_pr2"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(node_name)
    rospy.loginfo(node_name + ": is initialized")

    rc = moveit_commander.RobotCommander()
    rospy.loginfo("robot commander is initialized")

    #put the PR2 in mannequin mode for calibration
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,
     ["/opt/ros/indigo/share/pr2_mannequin_mode/launch/pr2_mannequin_mode.launch"])
    launch.start()
    #get corners for calibration
    raw_input('press enter to get top left corner: ')
    topLeft = get_corners()

    raw_input('press enter to get bottom left corner: ')
    bottomLeft = get_corners()

    raw_input('press enter to get bottom right corner: ')
    bottomRight = get_corners()
    #shutdown mannequin mode
    launch.shutdown()
    
    #draw image
    img = cv2.imread('dave.jpg')
    cmy = split_channels(img)
    for i in range(3):
        edges,max_side = edgeDetect(cmy[i])
        home_move_cartesian(edges, topLeft, bottomLeft, bottomRight, max_side)
        rospy.loginfo("finished color")
        raw_input('press enter after changing color') 
    
    moveit_commander.roscpp_shutdown()

