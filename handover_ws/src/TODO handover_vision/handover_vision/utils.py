import numpy as np
import tf_transformations as tf_trans
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Converte geometry_msgs/Pose in matrice di trasformazione 4x4."""
    translation = [pose.position.x, pose.position.y, pose.position.z]
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    matrix = tf_trans.quaternion_matrix(rotation)
    matrix[0:3, 3] = translation
    return matrix

def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Converte matrice di trasformazione 4x4 in geometry_msgs/Pose."""
    pose = Pose()
    translation = matrix[0:3, 3]
    rotation = tf_trans.quaternion_from_matrix(matrix)
    pose.position.x, pose.position.y, pose.position.z = translation
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
    return pose

def create_transform_stamped(parent_frame: str, child_frame: str, pose: Pose, stamp) -> TransformStamped:
    """Crea un messaggio TransformStamped per tf2_ros."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    return t