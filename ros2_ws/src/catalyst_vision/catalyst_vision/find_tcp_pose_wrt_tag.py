# "tag_base", "pose_in_base": "position": {"x": 0.687474, "y": 0.003273, "z": 0.047775}, 
# "orientation": {"qx": 0.47596, "qy": -0.454539, "qz": -0.517307, "qw": 0.547038

# "tag_object", "pose_in_base": {"position": {"x": 0.681913, "y": -0.10095, "z": 0.049114}, 
# "orientation": {"qx": 0.454047, "qy": -0.492181, "qz": -0.532561, "qw": 0.517665}

# "x": 0.6999, "y": -0.0459, "z": 0.0775, "qx": 0.999, "qy": -0.039, "qz": -0.0194, "qw": -0.0066


import numpy as np
from scipy.spatial.transform import Rotation as R

# H_TB_LB = np.eye(4)
# H_TO_LB = np.eye(4)
# H_TCP_LB = np.eye(4)

# H_TB_LB[:3, :3] = R.from_quat([0.47596, -0.454539, -0.517307, 0.547038]).as_matrix()
# H_TO_LB[:3, :3] = R.from_quat([0.454047, -0.492181, -0.532561, 0.517665]).as_matrix()
# H_TCP_LB[:3, :3] = R.from_quat([0.999, -0.039, -0.0194, -0.0066]).as_matrix()

# H_TB_LB[:3, 3] = np.array([0.687474, 0.003273, 0.047775]).T
# H_TO_LB[:3, 3] = np.array([0.681913, -0.10095, 0.049114]).T
# H_TCP_LB[:3, 3] = np.array([0.6999, -0.0459, 0.0775]).T

# H_TCP_TB = np.linalg.inv(H_TB_LB) @ H_TCP_LB
# H_TCP_TO = np.linalg.inv(H_TO_LB) @ H_TCP_LB

H_WP_TB = np.array([[1, 0, 0, 0.05185],
           [0, 1, 0, 0.03682],
           [0, 0, 1, -0.0491],
           [0, 0, 0, 1]])

H_WP_TO = np.array([[1, 0, 0, -0.05185],
           [0, 1, 0, 0.03682],
           [0, 0, 1, -0.0491],
           [0, 0, 0, 1]])
H_TCP_WP = np.array([[0, 1, 0, -0.004],
            [0, 0, -1, -0.00485],
            [-1, 0, 0, 0.015],
            [0, 0, 0, 1]])
H_TCP_TB = H_WP_TB @ H_TCP_WP
H_TCP_TO = H_WP_TO @ H_TCP_WP

print("\nH_TCP_TB\n", H_TCP_TB)
print("\nH_TCP_TO\n", H_TCP_TO)

"""
H_WP_TB = [[1, 0, 0, 0.03682],
           [0, 1, 0, 0.05185],
           [0, 0, 1, -0.0491],
           [0, 0, 0, 1]]

H_WP_TO = [[1, 0, 0, 0.03682],
           [0, 1, 0, -0.05185],
           [0, 0, 1, -0.0491],
           [0, 0, 0, 1]]
H_TCP_WP = [[0, 0, -1, -0.00285],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]]

H_TCP_TB = 
H_TCP_TO = 
"""