import pickle
import numpy as np
from handeye_4dof import DualQuaternion


def robot_pose_selector(camera_to_marker, base_to_hand):
    assert len(camera_to_marker) == len(base_to_hand), "Nonmatching number of transforms."
    motions = [None for _ in range(len(camera_to_marker))]
    for i, (Ai, Bi) in enumerate(zip(camera_to_marker, base_to_hand)):
        print("Obtaining motion {}...".format(i+1))
        curr_theta_max = 0
        for j, (Aj, Bj) in enumerate(zip(camera_to_marker, base_to_hand)):
            if i == j: continue
            A = DualQuaternion.from_transform(np.dot(Aj, np.linalg.inv(Ai)))
            B = DualQuaternion.from_transform(np.dot(np.linalg.inv(Bj), Bi))
            theta1 = A.as_screw_params()[-1]
            theta2 = B.as_screw_params()[-1]
            theta_sum = np.sum(np.abs([theta1, theta2]))
            if theta_sum > curr_theta_max:
                curr_theta_max = theta_sum
                motions[i] = (A, B)

    # TODO: add dual scalar minimization scoring selection
    print("Obtained all motions.")
    return motions


def main():
    with open("../example_data/pose_samples.pkl", "rb") as f:
        try:
            base_to_hand, camera_to_marker = pickle.load(f)
        except UnicodeDecodeError:
            # python 2 to python 3 pickle
            base_to_hand, camera_to_marker = pickle.load(f, encoding='latin1')

    motions = robot_pose_selector(camera_to_marker, base_to_hand)

    with open("../example_data/paired_poses.pkl", "wb") as f:
        pickle.dump(motions, f)


if __name__ == '__main__':
    main()
