import utils.geom as geom
import numpy as np


def post_process_obs(data):
    return data


def post_process_action(pose_vec_cur, pose_vec_prv, mode='quat'):

    pose_cur = np.eye(4)
    pose_cur[:3, :3] = geom.quat_to_rot(pose_vec_cur[3:])
    pose_cur[:3, 3] = pose_vec_cur[:3]

    pose_prv = np.eye(4)
    pose_prv[:3, :3] = geom.quat_to_rot(pose_vec_prv[3:])
    pose_prv[:3, 3] = pose_vec_prv[:3]

    # Convert to relative pose
    delta_pose = np.linalg.inv(pose_prv) @ pose_cur
    delta_pos = delta_pose[:3, 3]
    if mode == 'quat':
        delta_quat = geom.rot_to_quat(delta_pose[:3, :3])
    else:
        delta_quat = geom.rot_to_euler(delta_pose[:3, :3])

    act_data = np.concatenate([delta_pos, delta_quat])

    return act_data


def reconstruct_pose(delta_pose_vec, pose_vec_prv, mode='quat'):

    delta_pose = np.eye(4)
    if mode == 'quat':
        delta_pose[:3, :3] = geom.quat_to_rot(delta_pose_vec[3:])
    else:
        delta_pose[:3, :3] = geom.euler_to_rot(delta_pose_vec[3:])
    delta_pose[:3, 3] = delta_pose_vec[:3]

    pose_prv = np.eye(4)
    pose_prv[:3, :3] = geom.quat_to_rot(pose_vec_prv[3:])
    pose_prv[:3, 3] = pose_vec_prv[:3]

    # Convert to relative pose
    pose = pose_prv @ delta_pose
    pose_vec = np.zeros(7)
    pose_vec[:3] = pose[:3, 3]
    pose_vec[3:] = geom.rot_to_quat(pose[:3, :3])

    return pose_vec


def post_process_actions(pose_data, mode='quat'):

    # Convert to relative pose
    pose_rot = np.zeros((pose_data.shape[0], 4, 4))
    pose_rot[:, 0:3, 0:3] = geom.quat_to_rot(pose_data[:, 3:])
    pose_rot[:, 0:3, 3] = pose_data[:, :3]
    pose_rot[:, 3, 3] = 1

    sample_size = pose_rot.shape[0]

    delta_quat = []
    delta_rpy = []
    delta_pos = []

    for i in range(sample_size - 1):
        delta_transform = np.linalg.inv(pose_rot[i]) @ pose_rot[i+1]
        delta_pos.append(delta_transform[:3, 3])
        delta_quat.append(geom.rot_to_quat(delta_transform[:3, :3]))
        delta_rpy.append(geom.rot_to_euler(delta_transform[:3, :3]))

    delta_pos.append(np.zeros(3))
    delta_quat.append(np.array([0, 0, 0, 1]))
    delta_rpy.append(np.array([0, 0, 0]))

    if mode == 'quat':
        act_data = np.concatenate([np.array(delta_pos), np.array(delta_quat)], axis=1)
    else:
        act_data = np.concatenate([np.array(delta_pos), np.array(delta_rpy)], axis=1)
    return act_data


def reconstruct_poses(act_data, initial_pose):
    """
    Reconstruct pose from action data
    :param act_data: action data
    :param initial_pose: initial pose
    :return: pose data
    """
    # initial_pose = np.eye(4)    
    pose_vec = np.zeros((act_data.shape[0], 7))
    pose_rot = np.zeros((act_data.shape[0], 4, 4))
    pose_vec[0] = initial_pose

    pose_rot[0] = np.eye(4)
    pose_rot[0, :3, :3] = geom.quat_to_rot(initial_pose[3:])
    pose_rot[0, :3, 3] = initial_pose[:3]

    for i in range(act_data.shape[0] - 1):
        delta_transform = np.eye(4)
        delta_transform[:3, :3] = geom.quat_to_rot(act_data[i, 3:])
        delta_transform[:3, 3] = act_data[i, :3]
        pose_rot[i + 1] = pose_rot[i] @ delta_transform
        pose_vec[i + 1, :3] = pose_rot[i + 1, :3, 3]
        pose_vec[i + 1, 3:] = geom.rot_to_quat(pose_rot[i + 1, :3, :3])

    return pose_vec

