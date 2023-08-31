import numpy as np
import rospy
import tf
from FR3Env import getDataPath
from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial.transform import Rotation


class FR3PosePublisher:
    def __init__(self, node_name='fr3_pose_publisher', prefix="white"):
        # create FR3 pinocchio model
        fr3env_dir = getDataPath()
        robot_URDF = fr3env_dir + "/robots/fr3.urdf"
        self.robot = RobotWrapper.BuildFromURDF(robot_URDF, fr3env_dir)

        # get frame id
        self.hand_id = self.robot.model.getFrameId("fr3_hand")

        # create ROS node
        rospy.init_node(node_name)
        self.broadcaster = tf.TransformBroadcaster()
        self.prefix = prefix + "_"

        # sleep to ensure the node is ready
        rospy.sleep(0.1)   

    def publish(self, q):
        """
        Publish the hand frame position and orientation to /tf

        :param q: seven joint angles + two finger joint angles
        """
        t, quat = self.get_hand_pose(q, quat=True)

        self.broadcaster.sendTransform(t, quat, rospy.Time.now(), self.prefix+"fr3_hand", self.prefix+"fr3_link0")
    
    def _update_pinocchio(self, q):
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
    
    def get_hand_pose(self, q, quat=True):
        """
        Publish the hand frame position and orientation to /tf

        :param q: seven joint angles + two finger joint angles
        """
        self._update_pinocchio(q)

        # get hand frame position and orientation
        hand_frame = self.robot.data.oMf[self.hand_id]
        t = hand_frame.translation
        R = hand_frame.rotation

        if quat:
            quat = Rotation.from_matrix(R).as_quat()

            return t, quat
        else:
            return t, R
