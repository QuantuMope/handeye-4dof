import sympy as sy
import numpy as np
from scipy.optimize import minimize
from .dual_quaternions import DualQuaternion
from .utils import vec_to_skew_symmetric_mat, rotation_matrix_constraints, obtain_tf_from_rolled_arr


class Calibrator4DOF:
    def __init__(self, motions, sv_limit=0.5):
        self.motions = motions
        self.sv_limit = sv_limit

    @staticmethod
    def convert_to_dual_vector(A, B):
        dqs, dists = [], []
        for dq in [A, B]:
            L, d, M = dq.as_screw_params()[:-1]
            dists.append(d)
            dqs.append(DualQuaternion.from_dual_vector(L, M))

        # Fix screw sign ambiguity
        if dists[0] < 0 and dists[1] < 0:
            dqs[0] *= -1
            dqs[1] *= -1

        return dqs

    def calibrate(self, antiparallel_screw_axes=False):
        T = np.zeros((6*len(self.motions), 8))

        dq_rot1 = DualQuaternion.from_pose([0., 0., 0.], [0., 1., 0., 0.])
        dq_rot2 = dq_rot1.dq_conjugate1()

        for i, (A, B) in enumerate(self.motions):
            dq_a, dq_b = self.convert_to_dual_vector(A, B)

            if antiparallel_screw_axes:
                dq_a = dq_rot1 * dq_a * dq_rot2

            a_vec_r = dq_a.real.quat[1:]
            b_vec_r = dq_b.real.quat[1:]
            a_vec_d = dq_a.dual.quat[1:]
            b_vec_d = dq_b.dual.quat[1:]

            assert a_vec_r.dot(b_vec_r) > 0, "Error! Screw axes are anti-parallel. Matrix rank is being reduced."

            T[i*6:i*6+3, 0] = a_vec_r - b_vec_r
            T[i*6:i*6+3, 1:4] = vec_to_skew_symmetric_mat(a_vec_r + b_vec_r)
            T[i*6+3:i*6+6, 0] = a_vec_d - b_vec_d
            T[i*6+3:i*6+6, 1:4] = vec_to_skew_symmetric_mat(a_vec_d + b_vec_d)
            T[i*6+3:i*6+6, 4] = a_vec_r - b_vec_r
            T[i*6+3:i*6+6, 5:] = vec_to_skew_symmetric_mat(a_vec_r + b_vec_r)

        U, s, Vt = np.linalg.svd(T)

        # Check that singular values are as expected.
        for i, sv in enumerate(s):
            if i < 5:
                assert sv > self.sv_limit, "Singular value {} was {} < the limit {}.".format(i, sv, self.sv_limit)
            else:
                # The last 3 singular values should be reasonably close to zero.
                assert sv < self.sv_limit, "Singular value {} was {} > the limit {}.".format(i, sv, self.sv_limit)

        # Rows are same as V column vectors
        v6 = Vt[5]
        v7 = Vt[6]

        u1 = v6[:4]
        v1 = v6[4:]
        u2 = v7[:4]
        v2 = v7[4:]

        l1s, l2s = sy.symbols('l1, l2', real=True)

        # Dual quaternion unity constraints
        eq1 = l1s**2*u1.dot(u1) + 2*l1s*l2s*u1.dot(u2) + l2s**2*u2.dot(u2) - 1
        eq2 = l1s**2*u1.dot(v1) + l1s*l2s*(u1.dot(v2) + u2.dot(v1)) + l2s**2*u2.dot(v2)

        eqs = [sy.nsimplify(eq, rational=1) for eq in [eq1, eq2]]

        sols = list(sy.nonlinsolve(eqs, [l1s, l2s]))

        # Choose max s = l1/l2 value as solution.
        curr_l1, curr_l2 = 0, 0
        curr_max = 0
        for l1, l2 in sols:
            try:
                s = float(l1) / float(l2)
                val = s**2*u1.dot(u1) + 2*s*u1.dot(u2) + u2.dot(u2)
                if val > curr_max:
                    curr_max = val
                    curr_l1, curr_l2 = float(l1), float(l2)
            except TypeError:
                print("Detected Type Error")

        assert curr_l1 != 0 and curr_l2 != 0, "Couldn't find a solution."

        l1, l2 = curr_l1, curr_l2

        dq_x = l1 * v6 + l2 * v7
        dq_x = DualQuaternion(dq_x[:4], dq_x[4:])

        if antiparallel_screw_axes:
            dq_x = dq_rot2 * dq_x

        return dq_x

    @staticmethod
    def nonlinear_refinement(base_to_hand, camera_to_marker, calib_hand_to_camera):
        # We only vary the first 11 parameters of the transform matrix since we cannot solve for tz.
        W = np.eye(4)
        W[-1, -1] = 9

        def base_to_marker_error(xi):
            base_to_marker = obtain_tf_from_rolled_arr(xi)
            e = 0
            for bth, ctm in zip(base_to_hand, camera_to_marker):
                E = base_to_marker - bth.dot(calib_hand_to_camera).dot(ctm)
                e += np.trace(E.dot(W).dot(E.T))
            return e

        # We just use an arbitrary pose as our initial guess.
        x0 = (base_to_hand[0].dot(calib_hand_to_camera).dot(camera_to_marker[0])).ravel()[:11]

        nl_base_to_marker = obtain_tf_from_rolled_arr(minimize(base_to_marker_error, x0,
                                                               constraints=rotation_matrix_constraints()).x)

        # Now that we optimized to obtain base to marker transform, we perform the optimization
        # once more to regain the hand to camera transform (the one we care about).
        # This could possibly be replaced with some form of transform averaging.

        def hand_to_camera_error(xi):
            hand_to_camera = obtain_tf_from_rolled_arr(xi)
            e = 0
            for bth, ctm in zip(base_to_hand, camera_to_marker):
                E = hand_to_camera - np.linalg.inv(bth).dot(nl_base_to_marker).dot(np.linalg.inv(ctm))
                e += np.trace(E.dot(W).dot(E.T))
            return e

        x0 = (np.linalg.inv(base_to_hand[0]).dot(nl_base_to_marker).dot(np.linalg.inv(camera_to_marker[0]))).ravel()[:11]

        nl_hand_to_camera = obtain_tf_from_rolled_arr(minimize(hand_to_camera_error, x0,
                                                               constraints=rotation_matrix_constraints()).x)

        return nl_hand_to_camera
