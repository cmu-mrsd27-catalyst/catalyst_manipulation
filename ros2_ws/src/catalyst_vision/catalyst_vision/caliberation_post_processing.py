import numpy as np
from scipy.spatial.transform import Rotation as R


H_co_cl = np.array([[-0.001, 0, 1, 0],
                    [-1, -0.003, -0.001, 0.015],
                    [0.003, -1, 0, 0],
                    [0, 0, 0, 1]])

H_co_eef = np.array([[0.026, -0.012, 1.000, 0.065],
                    [1.000,0.016,-0.025,-0.021],
                    [-0.015 ,1.000 ,0.012 ,-0.043],
                    [0.000 ,0.000 ,0.000 ,1.000]])

H_cl_eef = H_co_eef @ np.linalg.inv(H_co_cl)
print(H_cl_eef)
r = R.from_matrix(H_cl_eef[:3,:3])
print("quat: ", r.as_quat())

