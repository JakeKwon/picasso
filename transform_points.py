import numpy as np
import scipy

def computeHomography(src_pts_nx2, dest_pts_nx2):

  A = np.array([[src_pts_nx2(1,1), src_pts_nx2(1,2), 1, 0, 0, 0,
        -dest_pts_nx2(1,1)*src_pts_nx2(1,1), -dest_pts_nx2(1,1)*src_pts_nx2(1,2), -dest_pts_nx2(1,1)],
      [0, 0, 0, src_pts_nx2(1,1), src_pts_nx2(1,2) 1, -dest_pts_nx2(1,2)*src_pts_nx2(1,1), -dest_pts_nx2(1,2)*src_pts_nx2(1,2),
      -dest_pts_nx2(1,2)]])
  for i  in range(1,len(src_pts_nx2))
      A.extend([[src_pts_nx2(i,1), src_pts_nx2(i,2), 1, 0, 0, 0, ...
      -dest_pts_nx2(i,1)*src_pts_nx2(i,1), -dest_pts_nx2(i,1)*src_pts_nx2(i,2), ...
      -dest_pts_nx2(i,1)], [0 0 0 src_pts_nx2(i,1), src_pts_nx2(i,2), 1, ...
      -dest_pts_nx2(i,2)*src_pts_nx2(i,1), -dest_pts_nx2(i,2)*src_pts_nx2(i,2), ...
      -dest_pts_nx2(i,2)]])
  
  A_m = A.'*A
  w, v = np.linalg.eig(A_m)
  ev = np.argmin(w)
  basis = A_m-(ev*np.eye(np.shape(A_m, 1)))
  H_elem = scipy.linalg.orth(basis)
  
  H3x3 = np.array([[H_elem(1), H_elem(2), H_elem(3)],
        [H_elem(4), H_elem(5), H_elem(6)],
        [H_elem(7) H_elem(8) 1]])
  
  return H3x3
