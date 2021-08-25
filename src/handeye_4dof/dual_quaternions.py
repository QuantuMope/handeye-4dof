import numpy as np
from .quaternions import Quaternion
from .utils import matrix_to_quat, quat_to_matrix


class DualQuaternion:
    def __init__(self, real_quat, dual_quat):
        self.real = Quaternion(real_quat) if not isinstance(real_quat, Quaternion) else real_quat
        self.dual = Quaternion(dual_quat) if not isinstance(dual_quat, Quaternion) else dual_quat
        self.normalize()

    @classmethod
    def from_pose(cls, position, quat_rot):
        real_quat = Quaternion(quat_rot) if not isinstance(quat_rot, Quaternion) else quat_rot
        dual_quat = DualQuaternion.translation_to_dual(position, real_quat)
        return cls(real_quat, dual_quat)

    @classmethod
    def from_transform(cls, tf):
        real_quat = Quaternion(matrix_to_quat(tf[:3, :3]))
        dual_quat = DualQuaternion.translation_to_dual(tf[:3, -1], real_quat)
        return cls(real_quat, dual_quat)

    @classmethod
    def from_dual_vector(cls, line_axis, moment):
        return cls(np.hstack(([0], line_axis)), np.hstack(([0], moment)))

    def __add__(self, other):
        new_real = self.real + other.real
        new_dual = self.dual + other.dual
        return DualQuaternion(new_real, new_dual)

    def __neg__(self):
        return DualQuaternion(-self.real, -self.dual)

    def __mul__(self, other):
        if isinstance(other, DualQuaternion):
            new_real = self.real * other.real
            new_dual = self.real * other.dual + self.dual * other.real
            return DualQuaternion(new_real, new_dual)
        return DualQuaternion(other * self.real, other * self.dual)

    def __rmul__(self, other):
        return DualQuaternion(other * self.real, other * self.dual)

    def __str__(self):
        return str(list(self.real.quat) + list(self.dual.quat))

    def dq_conjugate1(self):
        """
        D* = D0* + eD1*
        """
        return DualQuaternion(self.real.conjugate(), self.dual.conjugate())

    def dq_conjugate2(self):
        """
        D* = D0 - eD1
        """
        return DualQuaternion(self.real, -self.dual)

    def dq_conjugate3(self):
        """
        D* = D0* - eD1*
        """
        return DualQuaternion(self.real.conjugate(), -self.dual.conjugate())

    def normalize(self):
        real_norm = np.linalg.norm(self.real.quat)
        self.real.quat /= real_norm
        self.dual.quat /= real_norm

    def get_translation(self):
        return (2 * (self.dual * self.real.conjugate())).quat[1:]

    def get_rotation(self):
        return quat_to_matrix(self.real.quat)

    @staticmethod
    def translation_to_dual(trans, real):
        vec = np.zeros(4, dtype=np.float64)
        vec[1:] = trans
        return 0.5 * Quaternion(vec) * real

    def as_screw_params(self):
        axis, theta = self.real.as_axis_angle()
        translation = self.get_translation()

        d = translation.dot(axis)
        moment = 0.5 * (np.cross(translation, axis) + (translation - d * axis) / np.tan(0.5 * theta))
        return axis, d, moment, theta

    def as_transform(self):
        tf = np.zeros((4, 4), dtype=np.float64)
        tf[:3, :3] = quat_to_matrix(self.real.quat)
        tf[:3, -1] = self.get_translation()
        tf[-1, -1] = 1
        return tf
