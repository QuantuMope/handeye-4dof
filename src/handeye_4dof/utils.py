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


def obtain_tf_from_rolled_arr(xi):
    # xi := ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz
    # For calibration, we do not care about tz as we cannot solve for it anyways
    tf = np.zeros(16)
    tf[:11] = xi
    tf[-1] = 1
    return tf.reshape((4, 4))


def rotation_matrix_constraints():
    # source: https://graphics.stanford.edu/courses/cs348a-17-winter/ReaderNotes/handout17.pdf
    def constraint1(xi):
        ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz = xi
        return ux**2 + uy**2 + uz**2 - 1

    def constraint2(xi):
        ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz = xi
        return vx**2 + vy**2 + vz**2 - 1

    def constraint3(xi):
        ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz = xi
        return wx**2 + wy**2 + wz**2 - 1

    def constraint4(xi):
        ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz = xi
        return ux*vx + uy*vy + uz*vz

    def constraint5(xi):
        ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz = xi
        return ux*wx + uy*wy + uz*wz

    def constraint6(xi):
        ux, vx, wx, tx, uy, vy, wy, ty, uz, vz, wz = xi
        return wx*vx + wy*vy + wz*vz

    constraints = [constraint1, constraint2, constraint3, constraint4, constraint5, constraint6]

    return [{'type': 'eq', 'fun': c} for c in constraints]
