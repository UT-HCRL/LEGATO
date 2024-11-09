from scipy.spatial.transform import Rotation as R
import numpy as np

from spatialmath import SO3
from spatialmath.base import trnorm


def integrate_pose_with_delta_pose(pose, delta_pose):
    position = pose['pos'] + delta_pose[:3]
    rotation = trnorm(euler_to_rot(delta_pose[3:]) @ quat_to_rot(pose['quat']))
    quaternion = rot_to_quat(rotation)
    return {'pos': position, 'quat': quaternion}


def get_pose_error_vector(pose_current, pose_goal):
    pose_delta = np.zeros(6)
    pose_delta[:3] = pose_goal['pos'] - pose_current['pos']
    pose_delta[3:] = SO3(
        quat_to_rot(pose_goal['quat']) @ quat_to_rot(pose_current['quat']).T
    ).log(twist=True)
    return pose_delta


def get_quat_error_vector(quat_current, quat_goal):
    ang_delta = np.zeros(3)
    ang_delta = SO3(
        quat_to_rot(quat_goal) @ quat_to_rot(quat_current).T
    ).log(twist=True)
    return ang_delta


def bound_angle(angle):
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2*np.pi
    return angle


def x_rot(angle):
    return R.from_euler('x', angle).as_matrix()

def y_rot(angle):
    return R.from_euler('y', angle).as_matrix()

def z_rot(angle):
    return R.from_euler('z', angle).as_matrix()

def euler_to_rot(angles):
    # Euler ZYX to Rot
    # Note that towr has (x, y, z) order
    return np.copy(R.from_euler('xyz', angles, degrees=False).as_matrix())


def euler_to_quat(angles):
    # Euler ZYX to Rot
    # Note that towr has (x, y, z) order
    return np.copy((R.from_euler('xyz', angles, degrees=False)).as_quat())


def euler_to_axis_angle(angles):
    # Euler ZYX to Rot
    # Note that towr has (x, y, z) order
    return np.copy((R.from_euler('xyz', angles, degrees=False)).as_rotvec())


def quat_to_rot(quat):
    """
    Parameters
    ----------
    quat (np.array): scalar last quaternion

    Returns
    -------
    ret (np.array): SO3

    """
    return np.copy((R.from_quat(quat)).as_matrix())


def rot_to_quat(rot):
    """
    Parameters
    ----------
    rot (np.array): SO3

    Returns
    -------
    quat (np.array): scalar last quaternion

    """
    return np.copy(R.from_matrix(rot).as_quat())


def rot_to_euler(rot):
    """
    Parameters
    ----------
    rot (np.array): SO3

    Returns
    -------
    quat (np.array): scalar last quaternion

    """
    return np.copy(R.from_matrix(rot).as_euler('xyz'))


def rot_to_axis_angle(rot):
    """
    Parameters
    ----------
    rot (np.array): SO3

    Returns
    -------
    axis_angle (np.array): axis angle representation

    """
    return np.copy(R.from_matrix(rot).as_rotvec())


def axis_angle_to_quat(rotvec):
    """
    Parameters
    ----------
    rotvec (np.array): axis angle representation

    Returns
    -------
    rot (np.array): SO3

    """
    return np.copy(R.from_rotvec(rotvec).as_quat())


def axis_angle_to_rot(axis_angle):
    """
    Parameters
    ----------
    axis_angle (np.array): axis angle representation

    Returns
    -------
    rot (np.array): SO3

    """
    return np.copy(R.from_rotvec(axis_angle).as_matrix())


def quat_to_axis_angle(quat):
    """
    Parameters
    ----------
    quat (np.array): scalar last quaternion

    Returns
    -------
    rotvec (np.array): axis angle representation

    """
    return np.copy(R.from_quat(quat).as_rotvec())


def quat_to_exp(quat):
    img_vec = np.array([quat[0], quat[1], quat[2]])
    w = quat[3]
    theta = 2.0 * np.arcsin(
        np.sqrt(img_vec[0] * img_vec[0] + img_vec[1] * img_vec[1] +
                img_vec[2] * img_vec[2]))

    if np.abs(theta) < 1e-4:
        return np.zeros(3)
    ret = img_vec / np.sin(theta / 2.0)

    return np.copy(ret * theta)


def quat_to_euler(quat):

    return np.copy((R.from_quat(quat)).as_euler('xyz'))


def exp_to_quat(exp):
    theta = np.sqrt(exp[0] * exp[0] + exp[1] * exp[1] + exp[2] * exp[2])
    ret = np.zeros(4)
    if theta > 1e-4:
        ret[0] = np.sin(theta / 2.0) * exp[0] / theta
        ret[1] = np.sin(theta / 2.0) * exp[1] / theta
        ret[2] = np.sin(theta / 2.0) * exp[2] / theta
        ret[3] = np.cos(theta / 2.0)
    else:
        ret[0] = 0.5 * exp[0]
        ret[1] = 0.5 * exp[1]
        ret[2] = 0.5 * exp[2]
        ret[3] = 1.0
    return np.copy(ret)


def get_sinusoid_trajectory(start_time, mid_point, amp, freq, eval_time):
    dim = amp.shape[0]
    p, v, a = np.zeros(dim), np.zeros(dim), np.zeros(dim)
    p = amp * np.sin(2 * np.pi * freq * (eval_time - start_time)) + mid_point
    v = amp * 2 * np.pi * freq * np.cos(2 * np.pi * freq *
                                        (eval_time - start_time))
    a = -amp * (2 * np.pi * freq)**2 * np.sin(2 * np.pi * freq *
                                              (eval_time - start_time))

    return p, v, a


def prevent_quat_jump(quat_des, quat_act):
    # print("quat_des:",quat_des)
    # print("quat_act:",quat_act)
    a = quat_des - quat_act
    b = quat_des + quat_act
    if np.linalg.norm(a) > np.linalg.norm(b):
        new_quat_act = -quat_act
    else:
        new_quat_act = quat_act

    return new_quat_act