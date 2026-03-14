import numpy as np
from scipy.spatial.transform import Rotation as R


H_co_cl = np.array([[-0.001, 0, 1, 0],
                    [-1, -0.003, -0.001, 0.015],
                    [0.003, -1, 0, 0],
                    [0, 0, 0, 1]])

H_co_eef = np.array([[-0.01284776, 0.00232394, 0.99991476, 0.06463947733058763],
                    [0.99990543, -0.00487712, 0.01285898, -0.033911480077704004],
                    [0.00490658, 0.99998541, -0.00226106, -0.03843572202593569],
                    [0.000 ,0.000 ,0.000 , 1.000]])

H_cl_eef = H_co_eef @ np.linalg.inv(H_co_cl)
print(H_cl_eef)
r = R.from_matrix(H_cl_eef[:3,:3])
print("quat: ", r.as_quat())

# quat_co_eef = [0.49857056104408426, 0.5025514178414765, 0.5038511226924339 ,0.49497829674368826]
# r_from_quat = R.from_quat(quat_co_eef)
# rot_matrix = r_from_quat.as_matrix()

# print("\nReconstructed Rotation Matrix:\n", rot_matrix)

