import numpy as np
from scipy.spatial.transform import Rotation as R


def matrix_to_quat(mat):
    quat = R.from_matrix(mat).as_quat()
    # xyzw -> wxyz
    quat[1], quat[2], quat[3], quat[0] = quat[0], quat[1], quat[2], quat[3]
    return quat


def quat_to_matrix(quat):
    quat = quat.copy()
    # wxyz -> xyzw
    quat[0], quat[1], quat[2], quat[3] = quat[1], quat[2], quat[3], quat[0]
    return R.from_quat(quat).as_matrix()


def vec_to_skew_symmetric_mat(vec):
    v1, v2, v3 = vec
    mat = np.array([[0.0, -v3, v2],
                    [v3, 0.0, -v1],
                    [-v2, v1, 0.0]])
    return mat
