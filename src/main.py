import random
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R
from handeye_4dof import Calibrator4DOF


np.set_printoptions(suppress=True)


def main():
    with open("../example_data/paired_poses.pkl", "rb") as f:
        motions = pickle.load(f)

    motions = random.sample(motions, k=24)

    cb = Calibrator4DOF(motions)

    # Our camera and end effector z-axes are antiparallel so we must apply a 180deg x-axis rotation.
    dq_x = cb.calibrate(antiparallel_screw_axes=True)

    # Calibration Obtained Hand to Camera
    H = np.linalg.inv(dq_x.as_transform())
    translation = H[:3, -1]
    rot = np.rad2deg(R.from_matrix(H[:3, :3]).as_euler('xyz'))

    # Ground Truth Hand to Camera
    gt_translation = [-0.456, -0.037, -0.112]
    gt_rotation = [180, 0, 0]

    # Note that z-translation is meant to be inaccurate.
    print("Hand to Camera Transform Comparison")
    print("Obtained Translation: {} | Ground Truth Translation {}".format(translation, gt_translation))
    print("Obtained Rotation: {} | Ground Truth Rotation {}".format(rot, gt_rotation))


if __name__ == '__main__':
    main()