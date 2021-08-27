import argparse
import random
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R
from handeye_4dof import Calibrator4DOF, robot_pose_selector


np.set_printoptions(suppress=True)


def main():
    parser = argparse.ArgumentParser(description="Handeye-4DOF Calibrator.")
    parser.add_argument("-p", "--posePath", dest="pose_path",
                        help="path to pose pairs")
    parser.add_argument("-m", "--motionPath", dest="motion_path",
                        help="path to store motions or to load precomputed motions")
    parser.add_argument("-c", "--computeMotions", action="store_true", dest="compute_motions",
                        help="determines whether to compute and store motions or load precomputed motions")
    parser.add_argument("-a", "--antiParallelAxes", dest="antiparallel_axes", action="store_true",
                        help="perform 180 x-axis rotation to avoid matrix rank loss due to anti-parallel screw axes")
    parser.add_argument("-n", "--nonlinear", action="store_true",
                        help="perform nonlinear optimization on calibration result using pose pairs")
    parser.add_argument("-s", "--sample", type=int, default=-1,
                        help="randomly sample n motions for calibration")

    args = parser.parse_args()

    if args.pose_path is not None:
        with open(args.pose_path, "rb") as f:
            try:
                base_to_hand, camera_to_marker = pickle.load(f)
            except UnicodeDecodeError:
                # python 2 to python 3 pickle in case sampling was done in ROS
                base_to_hand, camera_to_marker = pickle.load(f, encoding='latin1')

    if args.compute_motions:
        assert args.pose_path is not None, "Can't compute motions without poses. Please specify -p."
        motions = robot_pose_selector(camera_to_marker, base_to_hand)
        if args.motion_path is not None:
            with open(args.motion_path, "wb") as f:
                pickle.dump(motions, f)
    else:
        assert args.motion_path is not None, "Can't calibrate without computing or specifying precomputed motions. " \
                                             "Please specify -c or -m."
        with open(args.motion_path, "rb") as f:
            motions = pickle.load(f)

    if args.sample > 0:
        motions = random.sample(motions, k=args.sample)

    # Initialize calibrator with motions.
    cb = Calibrator4DOF(motions)

    # Our camera and end effector z-axes are antiparallel so we must apply a 180deg x-axis rotation.
    dq_x = cb.calibrate(antiparallel_screw_axes=args.antiparallel_axes)

    # Calibration Obtained Hand to Camera
    hand_to_camera = np.linalg.inv(dq_x.as_transform())

    # Calibration Obtained with post nonlinear refinement
    if args.nonlinear:
        assert args.pose_path is not None, "Can't perform nonlinear refinement without poses. Please specify -p."
        hand_to_camera = cb.nonlinear_refinement(base_to_hand, camera_to_marker, hand_to_camera)

    rotation = np.rad2deg(R.from_matrix(hand_to_camera[:3, :3]).as_euler('xyz'))

    np.set_printoptions(precision=5)
    print("Translation:  {}".format(hand_to_camera[:3, -1]))
    print("XYZ Euler:    {}".format(rotation))


if __name__ == '__main__':
    main()
