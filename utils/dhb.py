import numpy as np
from . import geom

EPSILON = 1e-10

def apply_smoothening(data, window_size=3):

    #initial data offset
    data_offset = data[:,0]
    data_normalized = data - data_offset[:,None]
    smoothed_data = np.zeros(data_normalized.shape)
    for i in range(data_normalized.shape[0]):
        start = max(0, i - window_size)
        end = min(data_normalized.shape[0], i + window_size)
        smoothed_data[i] = np.mean(data_normalized[start:end], axis=0)
    return smoothed_data + data_offset[:,None]

def apply_low_pass_filtering(data, alpha=0.2):
    data_filtered = data
    for i in range(data_filtered.shape[0]-1):
        flag = np.zeros(data_filtered.shape[1])
        diff = data_filtered[i+1] - data_filtered[i]
        for j in range(data_filtered.shape[1]):
            if diff[j] > 0.5*np.pi:
                flag[j] = 1
                diff[j] -= np.pi
            elif diff[j] < -0.5*np.pi:
                flag[j] = -1
                diff[j] += np.pi
        data_filtered[i+1] = data_filtered[i] + (1-alpha) * diff
        data_filtered[i+1] += flag * np.pi
    return data_filtered


def apply_gaussian_noise(data, sigma=0.02):
    return data + np.random.normal(0, sigma, data.shape)

def bound_angle(angle):
    return 0.5*np.arctan2(np.sin(2*angle), np.cos(2*angle))

def pass_angle(angle):
    return angle

def compute_invariants(vector_u, frame_x, frame_x2, frame_y, frame_y2):
    magnitude = np.dot(frame_x, vector_u)
    angle1 = np.arctan2(
        np.dot(np.cross(frame_x, frame_x2), frame_y), np.dot(frame_x, frame_x2)
    )
    angle2 = np.arctan2(
        np.dot(np.cross(frame_y, frame_y2), frame_x2), np.dot(frame_y, frame_y2)
    )
    invariants = [magnitude, angle1, angle2]
    return invariants


def compute_frame_axis_X(vector_u, default_x):
    frame_x = vector_u
    norm_frame_x = np.linalg.norm(frame_x)
    if norm_frame_x > EPSILON:
        frame_x = frame_x / norm_frame_x
    else:
        frame_x = default_x
    return frame_x


def compute_frame_axis_Y(frame_x1, frame_x2, default_y):
    frame_y = np.cross(frame_x1, frame_x2)
    norm_frame_y = np.linalg.norm(frame_y)
    if norm_frame_y > EPSILON:
        frame_y = frame_y / norm_frame_y
    else:
        frame_y = default_y
    return frame_y


def compute_frame_axis_Z(frame_x, frame_y, default_z):
    frame_z = np.cross(frame_x, frame_y)
    norm_frame_z = np.linalg.norm(frame_z)
    if norm_frame_z > EPSILON:
        frame_z = frame_z / norm_frame_z
    else:
        frame_z = default_z
    return frame_z



def compute_initial_frames(positions, quaternions, method):

    assert positions.shape[0]==3 and quaternions.shape[0]==3

    initial_pose = np.eye(4)
    initial_pose[0:3, 0:3] = geom.quat_to_rot(quaternions[0])
    initial_pose[0:3, 3] = positions[0, :]

    position_diff = np.diff(positions, axis=0)
    rotation_diff = np.zeros((quaternions.shape[0] - 1, 3))
    for i in range(quaternions.shape[0] - 1):
        rotation_diff[i, :] = geom.rot_to_axis_angle(geom.quat_to_rot(quaternions[i]))

    linear_frame_x = compute_frame_axis_X(position_diff[0, :], np.array([1, 0, 0]))
    linear_frame_x2 = compute_frame_axis_X(position_diff[1, :], linear_frame_x)
    default_vector_y = np.array(
        [
            linear_frame_x[1] - linear_frame_x[2],
            linear_frame_x[2] - linear_frame_x[0],
            linear_frame_x[0] - linear_frame_x[1],
        ]
    )
    default_vector_y = default_vector_y / np.linalg.norm(default_vector_y)
    linear_frame_y = compute_frame_axis_Y(
        linear_frame_x, linear_frame_x2, default_vector_y
    )
    linear_frame_z = compute_frame_axis_Z(
        linear_frame_x, linear_frame_y, np.array([0, 0, 1])
    )

    angular_frame_x = compute_frame_axis_X(rotation_diff[0, :], np.array([1, 0, 0]))
    angular_frame_x2 = compute_frame_axis_X(rotation_diff[1, :], angular_frame_x)
    angular_frame_y = compute_frame_axis_Y(
        angular_frame_x, angular_frame_x2, np.array([0, 1, 0])
    )
    angular_frame_z = compute_frame_axis_Z(
        angular_frame_x, angular_frame_y, np.array([0, 0, 1])
    )

    linear_frame_initial = np.eye(4)
    linear_frame_initial[0:3, 0] = linear_frame_x
    linear_frame_initial[0:3, 1] = linear_frame_y
    linear_frame_initial[0:3, 2] = linear_frame_z
    if method == "pos":
        linear_frame_initial[0:3, 3] = initial_pose[0:3, 3]
    else:
        linear_frame_initial[0:3, 3] = position_diff

    angular_frame_initial = np.eye(4)
    angular_frame_initial[0:3, 0] = angular_frame_x
    angular_frame_initial[0:3, 1] = angular_frame_y
    angular_frame_initial[0:3, 2] = angular_frame_z

    return {'linear_frame_initial': linear_frame_initial, 'angular_frame_initial': angular_frame_initial, 'initial_pose': initial_pose}


def compute_DHBs(positions, quaternions):

    assert positions.shape[0] >2 and quaternions.shape[0] > 2, "The number of samples should be greater than 2"

    # Get the initial transform
    initial_pose = {"position": positions[0], "quaternion": quaternions[0]}

    position_diff = np.diff(positions, axis=0)
    rotation_diff = np.zeros((quaternions.shape[0] - 1, 3))
    for i in range(quaternions.shape[0] - 1):
        rot_prv = geom.quat_to_rot(quaternions[i])
        rot_cur = geom.quat_to_rot(quaternions[i+1])
        rotation_diff[i, :] = geom.rot_to_axis_angle(np.linalg.inv(rot_prv) @ rot_cur)

    num_samples = position_diff.shape[0]

    linear_frame_x = compute_frame_axis_X(position_diff[0, :], np.array([1, 0, 0]))
    linear_frame_x2 = compute_frame_axis_X(position_diff[1, :], linear_frame_x)
    default_vector_y = np.array(
        [
            linear_frame_x[1] - linear_frame_x[2],
            linear_frame_x[2] - linear_frame_x[0],
            linear_frame_x[0] - linear_frame_x[1],
        ]
    )
    default_vector_y = default_vector_y / np.linalg.norm(default_vector_y)
    linear_frame_y = compute_frame_axis_Y(
        linear_frame_x, linear_frame_x2, default_vector_y
    )
    linear_frame_z = compute_frame_axis_Z(
        linear_frame_x, linear_frame_y, np.array([0, 0, 1])
    )

    angular_frame_x = compute_frame_axis_X(rotation_diff[0, :], np.array([1, 0, 0]))
    angular_frame_x2 = compute_frame_axis_X(rotation_diff[1, :], angular_frame_x)
    angular_frame_y = compute_frame_axis_Y(
        angular_frame_x, angular_frame_x2, np.array([0, 1, 0])
    )
    angular_frame_z = compute_frame_axis_Z(
        angular_frame_x, angular_frame_y, np.array([0, 0, 1])
    )

    linear_frame_initial = np.eye(4)
    linear_frame_initial[0:3, 0] = linear_frame_x
    linear_frame_initial[0:3, 1] = linear_frame_y
    linear_frame_initial[0:3, 2] = linear_frame_z

    angular_frame_initial = np.eye(4)
    angular_frame_initial[0:3, 0] = angular_frame_x
    angular_frame_initial[0:3, 1] = angular_frame_y
    angular_frame_initial[0:3, 2] = angular_frame_z

    linear_motion_invariants = np.zeros((num_samples - 2, 3))
    angular_motion_invariants = np.zeros((num_samples - 2, 3))

    for i in range(num_samples - 2):
        linear_frame_x3 = compute_frame_axis_X(position_diff[i + 2], position_diff[i + 1])
        linear_frame_y2 = compute_frame_axis_Y(
            linear_frame_x2, linear_frame_x3, linear_frame_y
        )
        linear_motion_invariants[i, :] = compute_invariants(
            position_diff[i, :],
            linear_frame_x,
            linear_frame_x2,
            linear_frame_y,
            linear_frame_y2,
        )

        linear_frame_x = linear_frame_x2
        linear_frame_x2 = linear_frame_x3
        linear_frame_y = linear_frame_y2

        angular_frame_x3 = compute_frame_axis_X(rotation_diff[i + 2], rotation_diff[i + 1])
        angular_frame_y2 = compute_frame_axis_Y(
            angular_frame_x2, angular_frame_x3, angular_frame_y
        )
        angular_motion_invariants[i, :] = compute_invariants(
            rotation_diff[i, :],
            angular_frame_x,
            angular_frame_x2,
            angular_frame_y,
            angular_frame_y2,
        )

        angular_frame_x = angular_frame_x2
        angular_frame_x2 = angular_frame_x3
        angular_frame_y = angular_frame_y2

    return {
        "linear_motion_invariants": linear_motion_invariants,
        "angular_motion_invariants": angular_motion_invariants,
        "linear_frame_initial": linear_frame_initial,
        "angular_frame_initial": angular_frame_initial,
        "initial_pose": initial_pose,
    }


def compute_DHBs_ext(positions, quaternions):

    output = compute_DHBs(positions, quaternions)

    output["linear_motion_invariants"] = np.column_stack((output["linear_motion_invariants"][:,0],
                                                     np.sin(output["linear_motion_invariants"][:,1]),
                                                     np.sin(2*output["linear_motion_invariants"][:,1]),
                                                     np.sin(output["linear_motion_invariants"][:,2]),
                                                     np.sin(2*output["linear_motion_invariants"][:,2]),
                                                     ))
    output["angular_motion_invariants"] = np.column_stack((output["angular_motion_invariants"][:,0],
                                                        np.sin(output["angular_motion_invariants"][:,1]),
                                                        np.sin(2*output["angular_motion_invariants"][:,1]),
                                                        np.sin(output["angular_motion_invariants"][:,2]),
                                                        np.sin(2*output["angular_motion_invariants"][:,2]),
                                                        ))
    return output


def reconstruct_poses(
    linear_motion_invariants,
    angular_motion_invariants,
    linear_frame_initial,
    angular_frame_initial,
    initial_pose,
    method,
):
    linear_magnitude = linear_motion_invariants[:, 0]
    linear_angle_y = linear_motion_invariants[:, 1]
    linear_angle_x = linear_motion_invariants[:, 2]
    angular_magnitude = angular_motion_invariants[:, 0]
    angular_angle_y = angular_motion_invariants[:, 1]
    angular_angle_x = angular_motion_invariants[:, 2]

    num_samples = linear_angle_y.shape[0]

    trajectory_position = np.zeros((num_samples, 3))
    trajectory_quaternions = np.zeros((num_samples, 4))

    position_mode = method == "pos"

    linear_frame = np.copy(linear_frame_initial)
    linear_frame[0:3, 3] =  initial_pose["position"]
    linear_frame[3, 3] = float(position_mode)
    angular_frame = np.copy(angular_frame_initial)

    rotation_diff = np.zeros((num_samples, 3))
    trajectory_quaternions[0, :] = initial_pose["quaternion"]
    angular_rot_temp = geom.quat_to_rot(initial_pose['quaternion'])
    
    for i in range(num_samples):

        linear_frame_prv = np.copy(linear_frame)
        angular_frame_prv = np.copy(angular_frame)

        # if position_mode:
        trajectory_position[i, :] = linear_frame[0:3, 3]

        # Compute rotation matrices for the position or velocity
        rotation_matrix_position = np.dot(
            geom.y_rot(linear_angle_y[i]), geom.x_rot(linear_angle_x[i])
        )
        translation_vector = np.array([linear_magnitude[i], 0, 0])
        transformation_matrix = np.zeros((4, 4))
        transformation_matrix[0:3, 0:3] = rotation_matrix_position
        transformation_matrix[0:3, 3] = translation_vector
        transformation_matrix[3, 3] = float(position_mode)
        linear_frame = np.dot(linear_frame, transformation_matrix)

        if not position_mode:
            trajectory_position[i, :] = linear_frame[0:3, 3]

        # Apply the rotation to the angular frame and update the rotation trajectory
        rvec = np.dot(
            angular_frame[0:3, 0:3], np.array([angular_magnitude[i], 0, 0])
        )
        angular_rot_temp = angular_rot_temp @ geom.axis_angle_to_rot(rvec)
        if i < num_samples - 1:
            trajectory_quaternions[i+1, :] = geom.rot_to_quat(angular_rot_temp)
        rotation_diff[i, :] = rvec
        rotation_matrix_rotation = np.dot(
            geom.y_rot(angular_angle_y[i]), geom.x_rot(angular_angle_x[i])
        )
        angular_frame[0:3, 0:3] = np.dot(
            angular_frame[0:3, 0:3], rotation_matrix_rotation
        )

    # return trajectory_position, trajectory_quaternions, linear_frame, angular_frame

    return {
        "positions": trajectory_position,
        "quaternions": trajectory_quaternions,
        "linear_frame": linear_frame,
        "angular_frame": angular_frame,
        "linear_frame_prv": linear_frame_prv,
        "angular_frame_prv": angular_frame_prv,
    }

def reconstruct_poses_ext(    
        linear_motion_invariants,
        angular_motion_invariants,
        linear_frame_initial,
        angular_frame_initial,
        initial_pose,
        method="pos",
):

    new_linear_motion_invariants = np.column_stack((
                                        linear_motion_invariants[:,0],
                                        np.arctan2(linear_motion_invariants[:,1], 
                                                   0.5 * (linear_motion_invariants[:,2]/(linear_motion_invariants[:,1]+1e-12))),
                                        np.arctan2(linear_motion_invariants[:,3], 
                                                   0.5 * (linear_motion_invariants[:,4]/(linear_motion_invariants[:,3]+1e-12))),
                                        ))
    new_angular_motion_invariants = np.column_stack((
                                        angular_motion_invariants[:,0],
                                        np.arctan2(angular_motion_invariants[:,1], 
                                                   0.5 * (angular_motion_invariants[:,2]/(angular_motion_invariants[:,1]+1e-12))),
                                        np.arctan2(angular_motion_invariants[:,3], 
                                                   0.5 * (angular_motion_invariants[:,4]/(angular_motion_invariants[:,3]+1e-12))),
                                        ))
    
    return reconstruct_poses(
            new_linear_motion_invariants,
            new_angular_motion_invariants,
            linear_frame_initial,
            angular_frame_initial,
            initial_pose,
            method,
            )


def compute_DHB_recur(delta_positions, delta_rotations):

    assert delta_positions.shape[0] == 3 and delta_rotations.shape[0] == 3, "The number of samples should be 2"

    transform_matrix = np.zeros((4, 4, 4))
    delta_transform = np.zeros((3, 4, 4))
    delta_transform[:, 0:3, 0:3] = geom.euler_to_rot(delta_rotations)
    delta_transform[:, 0:3, 3] = delta_positions
    delta_transform[:, 3, 3] = 1

    transform_matrix[0] = np.eye(4)
    for i in range(3):
        transform_matrix[i+1] = transform_matrix[i] @ delta_transform[i]

    positions = transform_matrix[:, :3, 3]
    quaternions = geom.rot_to_quat(transform_matrix[:, :3, :3])

    position_diff = np.diff(positions, axis=0)
    rotation_diff = np.zeros((quaternions.shape[0] - 1, 3))
    for i in range(quaternions.shape[0] - 1):
        rot_prv = geom.quat_to_rot(quaternions[i])
        rot_cur = geom.quat_to_rot(quaternions[i+1])
        rotation_diff[i, :] = geom.rot_to_axis_angle(np.linalg.inv(rot_prv) @ rot_cur)

    linear_frame_x1 = compute_frame_axis_X(position_diff[0], np.array([1, 0, 0]))
    linear_frame_x2 = compute_frame_axis_X(position_diff[1], linear_frame_x1)
    linear_frame_x3 = compute_frame_axis_X(position_diff[2], linear_frame_x2)
    default_vector_y = np.array(
        [
            linear_frame_x1[1] - linear_frame_x1[2],
            linear_frame_x1[2] - linear_frame_x1[0],
            linear_frame_x1[0] - linear_frame_x1[1],
        ]
    )
    default_vector_y = default_vector_y / np.linalg.norm(default_vector_y)
    linear_frame_y1 = compute_frame_axis_Y(linear_frame_x1, linear_frame_x2, default_vector_y)
    linear_frame_y2 = compute_frame_axis_Y(linear_frame_x2, linear_frame_x3, linear_frame_y1)
    linear_motion_invariant_back = compute_invariants(
        position_diff[0],
        linear_frame_x1,
        linear_frame_x2,
        linear_frame_y1,
        linear_frame_y2,
    )

    angular_frame_x1 = compute_frame_axis_X(rotation_diff[0], np.array([1, 0, 0]))
    angular_frame_x2 = compute_frame_axis_X(rotation_diff[1], angular_frame_x1)
    angular_frame_x3 = compute_frame_axis_X(rotation_diff[2], angular_frame_x2)
    angular_frame_y1 = compute_frame_axis_Y(angular_frame_x1, angular_frame_x2, np.array([0, 1, 0]))
    angular_frame_y2 = compute_frame_axis_Y(angular_frame_x2, angular_frame_x3, angular_frame_y1)
    angular_motion_invariant_back = compute_invariants(
        rotation_diff[0],
        angular_frame_x1,
        angular_frame_x2,
        angular_frame_y1,
        angular_frame_y2,
    )

    linear_motion_invariant_back = np.array((linear_motion_invariant_back[0],
                                            np.sin(linear_motion_invariant_back[1]),
                                            np.sin(2*linear_motion_invariant_back[1]),
                                            np.sin(linear_motion_invariant_back[2]),
                                            np.sin(2*linear_motion_invariant_back[2]),
                                            ))
    angular_motion_invariant_back = np.array((angular_motion_invariant_back[0],
                                            np.sin(angular_motion_invariant_back[1]),
                                            np.sin(2*angular_motion_invariant_back[1]),
                                            np.sin(angular_motion_invariant_back[2]),
                                            np.sin(2*angular_motion_invariant_back[2]),
                                            ))

    return linear_motion_invariant_back, angular_motion_invariant_back
