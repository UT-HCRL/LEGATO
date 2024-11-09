import torch
import pytorch3d.transforms as transforms

import os, sys

EPSILON = 1e-10


def axis_angle_to_rot(rot):
    return transforms.axis_angle_to_matrix(rot)

def rot_to_axis_angle(rot):
    return transforms.matrix_to_axis_angle(rot)

def quat_to_axis_angle(quat):
    return transforms.quaternion_to_axis_angle(quat)

def euler_to_axis_angle(euler_angles):
    rot = euler_to_rot(euler_angles)
    return rot_to_axis_angle(rot)

def quat_to_rot(quat):
    return transforms.quaternion_to_matrix(quat[..., [3, 0, 1, 2]])

def euler_to_rot(euler_angles):
    # Extract individual angles
    angles_x, angles_y, angles_z = euler_angles.unbind(-1)
    
    # Precompute cosines and sines of the angles
    cos_x, cos_y, cos_z = torch.cos(angles_x), torch.cos(angles_y), torch.cos(angles_z)
    sin_x, sin_y, sin_z = torch.sin(angles_x), torch.sin(angles_y), torch.sin(angles_z)
    
    # Construct the rotation matrix from Euler angles
    r00 = cos_y * cos_z
    r01 = sin_x * sin_y * cos_z - cos_x * sin_z
    r02 = cos_x * sin_y * cos_z + sin_x * sin_z
    r10 = cos_y * sin_z
    r11 = sin_x * sin_y * sin_z + cos_x * cos_z
    r12 = cos_x * sin_y * sin_z - sin_x * cos_z
    r20 = -sin_y
    r21 = sin_x * cos_y
    r22 = cos_x * cos_y

    # Stack the elements into the rotation matrix
    rotation_matrix = torch.stack([
        torch.stack([r00, r01, r02], dim=-1),
        torch.stack([r10, r11, r12], dim=-1),
        torch.stack([r20, r21, r22], dim=-1)
    ], dim=-2)

    return rotation_matrix

def quat_to_axis_angle(quat):
    return transforms.quaternion_to_axis_angle(quat[..., [3, 0, 1, 2]])

def compute_frame_axis_X(vector_u, default_x):
    norm_frame_x = torch.norm(vector_u, dim=-1, keepdim=True)
    frame_x = torch.where(norm_frame_x > EPSILON, vector_u / norm_frame_x, default_x)
    return frame_x

def compute_frame_axis_Y(frame_x1, frame_x2, default_y):
    frame_y = torch.cross(frame_x1, frame_x2, dim=-1)
    norm_frame_y = torch.norm(frame_y, dim=-1, keepdim=True)
    frame_y = torch.where(norm_frame_y > EPSILON, frame_y / norm_frame_y, default_y)
    return frame_y

def compute_invariants(vector_u, frame_x, frame_x2, frame_y, frame_y2):
    magnitude = (frame_x * vector_u).sum(dim=-1)
    cross_x_x2 = torch.cross(frame_x, frame_x2, dim=-1)
    cross_y_y2 = torch.cross(frame_y, frame_y2, dim=-1)
    angle1 = torch.atan2((cross_x_x2 * frame_y).sum(dim=-1), (frame_x * frame_x2).sum(dim=-1))
    angle2 = torch.atan2((cross_y_y2 * frame_x2).sum(dim=-1), (frame_y * frame_y2).sum(dim=-1))
    invariants = torch.stack([magnitude, angle1, angle2], dim=-1)
    return invariants

def compute_DHB_recur(cur_delta_position, cur_delta_rot, prv_delta_positions, prv_delta_rotations, mode):

    device = cur_delta_position.device

    assert prv_delta_positions.shape[-2] == 2 and prv_delta_rotations.shape[-2] == 2, "The number of samples should be 2"

    delta_positions = torch.cat([prv_delta_positions, cur_delta_position[...,None,:]], dim=-2)
    delta_rotations = torch.cat([prv_delta_rotations, cur_delta_rot[...,None,:]], dim=-2)

    delta_transforms = torch.zeros(delta_positions.shape[:-1]+(4, 4), dtype=torch.float32).to(device)
    if mode == 'rpy':
        delta_transforms[..., 0:3, 0:3] = euler_to_rot(delta_rotations)
    else:
        delta_transforms[..., 0:3, 0:3] = quat_to_rot(delta_rotations)
    delta_transforms[..., 0:3, 3] = delta_positions
    delta_transforms[..., 3, 3] = 1

    transform_matrices = torch.zeros(delta_positions.shape[:-2]+(delta_positions.shape[-2]+1, 4, 4), dtype=torch.float32).to(device)
    transform_matrices[..., 0,:,:] = torch.eye(4).to(device)

    for i in range(3):
        transform_matrices[..., i+1, :, :] = torch.matmul(transform_matrices[..., i, :, :], delta_transforms[..., i, :, :])

    positions = transform_matrices[..., 0:3, 3]
    rotations = transform_matrices[..., 0:3, 0:3]
    
    position_diff = positions[..., 1:,:] - positions[..., :-1,:]
    rotation_diff = torch.matmul(rotations[..., :-1, :,:].transpose(-1, -2), rotations[..., 1:, :,:])
    rotation_diff = rot_to_axis_angle(rotation_diff)

    default_x = torch.tensor([1, 0, 0], dtype=torch.float32).expand_as(position_diff[...,0,:]).to(device)
    default_y = torch.tensor([0, 1, 0], dtype=torch.float32).expand_as(position_diff[...,0,:]).to(device)

    linear_frame_x1 = compute_frame_axis_X(position_diff[...,0,:], default_x)
    linear_frame_x2 = compute_frame_axis_X(position_diff[...,1,:], linear_frame_x1)
    linear_frame_x3 = compute_frame_axis_X(position_diff[...,2,:], linear_frame_x2)

    # Compute a generic default_y based on linear_frame_x1 for demonstration
    default_vector_y = torch.cross(linear_frame_x1, linear_frame_x2, dim=-1)
    default_vector_y = default_vector_y / torch.norm(default_vector_y, dim=-1, keepdim=True)  # Normalize

    linear_frame_y1 = compute_frame_axis_Y(linear_frame_x1, linear_frame_x2, default_vector_y)
    linear_frame_y2 = compute_frame_axis_Y(linear_frame_x2, linear_frame_x3, linear_frame_y1)

    linear_motion_invariant_back = compute_invariants(
        position_diff[...,0,:],
        linear_frame_x1,
        linear_frame_x2,
        linear_frame_y1,
        linear_frame_y2,
    )

    angular_frame_x1 = compute_frame_axis_X(rotation_diff[...,0,:], default_x)
    angular_frame_x2 = compute_frame_axis_X(rotation_diff[...,1,:], angular_frame_x1)
    angular_frame_x3 = compute_frame_axis_X(rotation_diff[...,2,:], angular_frame_x2)
    angular_frame_y1 = compute_frame_axis_Y(angular_frame_x1, angular_frame_x2, default_y)
    angular_frame_y2 = compute_frame_axis_Y(angular_frame_x2, angular_frame_x3, angular_frame_y1)

    angular_motion_invariant_back = compute_invariants(
        rotation_diff[...,0,:],
        angular_frame_x1,
        angular_frame_x2,
        angular_frame_y1,
        angular_frame_y2,
    )

    # replace Nan with 0
    linear_motion_invariant_back[torch.isnan(linear_motion_invariant_back)] = 0
    angular_motion_invariant_back[torch.isnan(angular_motion_invariant_back)] = 0

    linear_motion_invariant_ext = torch.cat(
        (linear_motion_invariant_back[...,0], 
         torch.sin(linear_motion_invariant_back[...,1]),
         torch.sin(2*linear_motion_invariant_back[...,1]),
         torch.sin(linear_motion_invariant_back[...,2]),
         torch.sin(2*linear_motion_invariant_back[...,2])), dim=-1)
    angular_motion_invariant_ext = torch.cat(
        (angular_motion_invariant_back[...,0], 
         torch.sin(angular_motion_invariant_back[...,1]),
         torch.sin(2*angular_motion_invariant_back[...,1]),
         torch.sin(angular_motion_invariant_back[...,2]),
         torch.sin(2*angular_motion_invariant_back[...,2])), dim=-1)

    return linear_motion_invariant_ext, angular_motion_invariant_ext