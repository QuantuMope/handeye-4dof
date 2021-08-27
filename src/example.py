import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R
from handeye_4dof import Calibrator4DOF


np.set_printoptions(suppress=True)


def main():
    with open("../example_data/pose_samples.pkl", "rb") as f:
        try:
            base_to_hand, camera_to_marker = pickle.load(f)
        except UnicodeDecodeError:
            # python 2 to python 3 pickle in case sampling was done in ROS
            base_to_hand, camera_to_marker = pickle.load(f, encoding='latin1')

    with open("../example_data/paired_poses.pkl", "rb") as f:
        motions = pickle.load(f)

    # Initialize calibrator with precomputed motions.
    cb = Calibrator4DOF(motions)

    # Our camera and end effector z-axes are antiparallel so we must apply a 180deg x-axis rotation.
    dq_x = cb.calibrate(antiparallel_screw_axes=True)

    # Calibration Obtained Hand to Camera
    ca_hand_to_camera = np.linalg.inv(dq_x.as_transform())

    # Calibration Obtained with post nonlinear refinement
    nl_hand_to_camera = cb.nonlinear_refinement(base_to_hand, camera_to_marker, ca_hand_to_camera)

    ca_rotation = np.rad2deg(R.from_matrix(ca_hand_to_camera[:3, :3]).as_euler('xyz'))
    nl_rotation = np.rad2deg(R.from_matrix(nl_hand_to_camera[:3, :3]).as_euler('xyz'))

    # Ground Truth Hand to Camera
    gt_translation = [-0.456, -0.037, -0.112]
    gt_rotation = [180, 0, 0]

    # NOTE: (1) Ground Truth itself may be inaccurate (manually measured).
    #       (2) z-translation is an invalid number.
    np.set_printoptions(precision=5)
    print("Hand to Camera Transform Comparisons")
    print("Translations: Calibration  {}".format(ca_hand_to_camera[:3, -1]))
    print("              Nonlinear    {}".format(nl_hand_to_camera[:3, -1]))
    print("              Ground Truth {}".format(gt_translation))
    print("Rotations:    Calibration  {}".format(ca_rotation))
    print("              Nonlinear    {}".format(nl_rotation))
    print("              Ground Truth {}".format(gt_rotation))


if __name__ == '__main__':
    main()
