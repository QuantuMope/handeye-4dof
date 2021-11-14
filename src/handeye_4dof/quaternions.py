import numpy as np


class Quaternion:
    def __init__(self, quat):
        self.quat = np.array(quat, dtype=np.float64)

    def __add__(self, other):
        return Quaternion(self.quat + other.quat)

    def __neg__(self):
        return Quaternion(-self.quat)

    def __rmul__(self, other):
        return Quaternion(other * self.quat)

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            w1, x1, y1, z1 = self.quat
            w2, x2, y2, z2 = other.quat
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            y = w1*y2 + y1*w2 + z1*x2 - x1*z2
            z = w1*z2 + z1*w2 + x1*y2 - y1*x2
            return Quaternion([w, x, y, z])
        return Quaternion(other * self.quat)

    def __str__(self):
        return str(self.quat)

    def conjugate(self):
        return Quaternion(self.quat * [1, -1, -1, -1])

    def normalize(self):
        self.quat /= np.linalg.norm(self.quat)

    def as_axis_angle(self):
        w, xyz = self.quat[0], self.quat[1:]
        xyz_norm = np.linalg.norm(xyz)

        if xyz_norm < 1e-10:
            return np.array([1.0, 0.0, 0.0]), 0.0

        axis = xyz / np.linalg.norm(xyz)
        angle = -((np.pi - 2 * np.arccos(w)) % (2.0 * np.pi) - np.pi)

        if angle < 0.0:
            axis *= -1
            angle *= -1

        return axis, angle
