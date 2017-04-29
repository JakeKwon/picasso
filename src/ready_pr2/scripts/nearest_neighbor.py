import numpy as np
import matplotlib.pyplot as plt
import cv2
import numpy as np

def edgeDetect():
    img = cv2.imread('dave.jpg')
    img = cv2.blur(img,(3,3))
    # cv2.imshow('dave',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    # cv2.imshow('dave',edges)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    cv2.imshow('edges',edges)

    lines = cv2.HoughLinesP(edges,1,np.pi/90,10,10,35,10)
    #print img.shape
    img = np.ones(img.shape)
    for x1,y1,x2,y2 in lines[0]:
        cv2.line(img,(x1,y1),(x2,y2),(0,0,0),2)

    # cv2.imshow('houghlines3.jpg',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return lines[0]


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
'''
mat = edgeDetect()
#print mat.shape
sorted_mat = sort_lines(mat)
print sorted_mat
reshaped = reshape(sorted_mat)
print reshaped

sorted_list = sorted_mat.reshape(sorted_mat.shape[0]*2,2)
x = np.append(sorted_list[:,0],sorted_list[0,0])
y = np.append(sorted_list[:,1],sorted_list[0,1])
print x
print y
plt.gca().invert_yaxis()

plt.axis('equal')
plt.plot(x,y)
plt.show()
'''
