import numpy as np
import matplotlib.pyplot as plt
import cv2
import numpy as np
def rgb_to_cmy(img):
    print "-----------------------------------------------"
    #cmy = np.matrix(np.ones(colors.shape))
    cmy = 1.0-img/256.0
    min_cmy=np.min(cmy,axis=2)
    print min_cmy
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

    #print img.shape
    #print np.sum(img)
    print img.dtype
    '''
    imgShape = img.shape[0:2]

    imgShape= np.multiply(imgShape,894.0/np.max(imgShape)).astype(int)
    
    img = cv2.resize(img,(imgShape[1],imgShape[0]))
    '''
    img = cv2.blur(img,(5,5))
   
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray,1500,1500,apertureSize =7)
    #cv2.imshow('edges',edges)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    #print img.shape
    lines = cv2.HoughLinesP(edges,1,np.pi/90,10,10,40,10)
    
    img = np.ones(img.shape)
   
    for x1,y1,x2,y2 in lines[0]:
        cv2.line(img,(x1,y1),(x2,y2),(0,0,0),2)
    print lines[0].shape
    cv2.imshow('img',img)
    cv2.waitKey()
    return lines[0]


def sort_lines(mat):
    num_lines = mat.shape[0]

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

    #sort the endpoints so that the closer points are first
    for  i in range(num_lines-1):
        smaller = np.argmin( np.linalg.norm(np.subtract(sorted_mat[i,2:],sorted_mat[i+1,:].reshape(2,2)),axis=1))
        if smaller:
            smaller = sorted_mat[i+1,2:].copy()
            sorted_mat[i+1,2:]=sorted_mat[i+1,:2].copy()
            sorted_mat[i+1,:2] = smaller

    return sorted_mat

img = cv2.imread('dave.jpg')
cmy = split_channels(img)
for i in range(3):
    mat = edgeDetect(cmy[i])
    cv2.imshow('image',cmy[i])
    cv2.waitKey()
    sorted_mat = sort_lines(mat)

    sorted_list = sorted_mat.reshape(sorted_mat.shape[0]*2,2)
    x = np.append(sorted_list[:,0],sorted_list[0,0])
    y = np.append(sorted_list[:,1],sorted_list[0,1])
    '''
    plt.gca().invert_yaxis()

    plt.axis('equal')
    plt.plot(x,y)
    plt.show()
    '''
