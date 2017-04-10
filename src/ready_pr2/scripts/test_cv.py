# edge detection
import cv2
import numpy as np

def edgeDetect():
    img = cv2.imread('rect.png')
    img = cv2.blur(img,(3,3))
    #cv2.imshow('dave',img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    
    cv2.imshow('edges',edges)

    lines = cv2.HoughLinesP(edges,1,np.pi/90,10,10,35,10)


    for x1,y1,x2,y2 in lines[0]:
        cv2.line(img,(x1,y1),(x2,y2),(0,0,0),2)
    
    cv2.imshow('houghlines3.jpg',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
edgeDetect()
