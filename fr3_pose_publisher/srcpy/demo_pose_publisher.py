import time

import numpy as np
import read_fr3_state

from fr3_pose_publisher import FR3PosePublisher


def main():
    fr3_pose_publisher = FR3PosePublisher()

    for i in range(10000):
        joint_pos = read_fr3_state.read_state("10.42.0.4")
        q = np.concatenate([joint_pos, np.zeros(2)])
        fr3_pose_publisher.publish(q)
        time.sleep(0.01)

if __name__ == '__main__':
    main()