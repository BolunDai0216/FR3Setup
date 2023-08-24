import pinocchio as pin
import rospy
import tf
from FR3Env import getDataPath
from pinocchio.robot_wrapper import RobotWrapper


class FR3PosePublisher:
    def __init__(self, node_name='fr3_pose_publisher'):
        # create FR3 pinocchio model
        fr3env_dir = getDataPath()
        robot_URDF = fr3env_dir + "/robots/fr3.urdf"
        self.robot = RobotWrapper.BuildFromURDF(robot_URDF, fr3env_dir)

        # get frame id
        self.hand_id = self.robot.model.getFrameId("fr3_hand")

        rospy.init_node(node_name)
        self.broadcaster = tf.TransformBroadcaster()

        # sleep to ensure the node is ready
        rospy.sleep(0.1)   

    def publish(self, q):
        """
        Publish the hand frame position and orientation to /tf

        :param q: seven joint angles + two finger joint angles
        """
        self._update_pinocchio(q)

        # get hand frame position and orientation
        hand_frame = self.robot.data.oMf[self.hand_id]
        t = hand_frame.translation
        R = hand_frame.rotation
        q = pin.Quaternion(R)  # [x, y, z, w]

        self.broadcaster.sendTransform(t, q, rospy.Time.now(), "hand_frame", "base_frame")
    
    def _update_pinocchio(self, q):
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)