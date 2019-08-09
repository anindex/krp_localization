import numpy as np
from geometry_msgs.msg import PoseArray, Pose
import tf

def mesh(x1,x2):
    """
    Formated mesh of vectors x1, x2
    Output is numpy array [p,2], congruent with 'X' matrix used for positions in localization
    functions and classes
    """
    x1v, x2v = np.meshgrid(x1,x2)
    x1v = np.reshape(x1v,(np.prod(x1v.shape),1))
    x2v = np.reshape(x2v,(np.prod(x2v.shape),1))
    XM  = np.concatenate((x1v,x2v),axis=1)
    return XM


def pose_from_array(array):
    pose = Pose()
    pose.position.x = array.flatten()[0]
    pose.position.y = array.flatten()[1]
    return pose

def pose_from_array_orientation(array):
    pose = Pose()
    pose.position.x = array[0]
    pose.position.y = array[1]
    quat = tf.transformations.quaternion_from_euler(0,0,array[2])
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def get_best_pose(pose_array, weights):
    if not pose_array or len(pose_array) != len(weights):
        return

    index = np.argmax(weights)
    return pose_array[index, :]
