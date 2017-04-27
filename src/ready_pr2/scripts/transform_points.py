import numpy as np
import scipy

def transform_points(a,b,c,mat):
    i = (c-b)/np.linalg.norm(b-c)
    j = (a-b)/np.linalg.norm(a-b)
    k = np.cross(i,j)
    rot_mat = np.matrix([i,j,k]).T
    print "rotation matrix:\n",rot_mat
    print "img:\n", mat
    rotated_img = np.dot(rot_mat,mat)
    print "rotated img:\n",rotated_img

    return np.add(rotated_img,np.matrix(b).T)


'''    
def computeHomography(src_pts_nx2, dest_pts_nx2):

  A = np.array([[src_pts_nx2[1,1], src_pts_nx2[1,2], 1, 0, 0, 0,
        -dest_pts_nx2[1,1]*src_pts_nx2[1,1], -dest_pts_nx2[1,1]*src_pts_nx21,2), -dest_pts_nx2(1,1)],
      [0, 0, 0, src_pts_nx2(1,1), src_pts_nx2(1,2), 1, -dest_pts_nx2(1,2)*src_pts_nx2(1,1), -dest_pts_nx2(1,2)*src_pts_nx2(1,2),
      -dest_pts_nx2(1,2)]])
  for i  in range(1,len(src_pts_nx2)):
      A.extend([[src_pts_nx2(i,1), src_pts_nx2(i,2), 1, 0, 0, 0,
      -dest_pts_nx2(i,1)*src_pts_nx2(i,1), -dest_pts_nx2(i,1)*src_pts_nx2(i,2),
      -dest_pts_nx2(i,1)], [0, 0, 0, src_pts_nx2(i,1), src_pts_nx2(i,2), 1, 
      -dest_pts_nx2(i,2)*src_pts_nx2(i,1), -dest_pts_nx2(i,2)*src_pts_nx2(i,2),
      -dest_pts_nx2(i,2)]])
  
  A_m = A.T *A
  w, v = np.linalg.eig(A_m)
  ev = np.argmin(w)
  basis = A_m-(ev*np.eye(np.shape(A_m, 1)))
  H_elem = scipy.linalg.orth(basis)
  
  H3x3 = np.array([[H_elem(1), H_elem(2), H_elem(3)],
        [H_elem(4), H_elem(5), H_elem(6)],
        [H_elem(7), H_elem(8), 1]])
  
  return H3x3
'''
print "------------------------------------------"

a = np.array([0,1,1])
b = np.array([0,0,1])
c = np.array([1,0,1])
print transform_points(a,b,c,np.matrix([[1,0,0],[2,0,0],[3,0,0]]).T)
