import os
import sys
import argparse
import numpy as np
cwd = os.getcwd()
sys.path.append(cwd)

import utils.geom as geom
import csv

import cv2

from multiprocessing import Process
import h5py
import utils.demo as demo
import utils.dhb as dhb


def gray_obs_process(path):

    def process_img(img, resize=(128, 128)):
        # crop the image with the shortest side from the center
        img_shape = img.shape[:2]
        cropped_shape = min(img_shape)
        cropped_img = img[(img_shape[0] - cropped_shape) // 2: (img_shape[0] + cropped_shape) // 2,
                            (img_shape[1] - cropped_shape) // 2: (img_shape[1] + cropped_shape) // 2]
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        proc_img = cv2.resize(gray_img, resize)
        return np.expand_dims(proc_img, axis=-1)

    demo_file = h5py.File(os.path.join(path, 'demo.hdf5'), 'r')
    time_stamps = np.array(demo_file['time'])
    demo_file.close()

    paths = {}
    paths['left_gray'] = os.path.join(path, 'left_img')
    paths['right_gray'] = os.path.join(path, 'right_img')

    samples = []
    obs_data = {}
    for key, img_dir in paths.items():
        # print('\tworking on {}...'.format(path))
        count_gray = 0
        gray_data = []

        # firt image
        img_files = {int(file_name.split('.')[0]): file_name for file_name in os.listdir(img_dir) if file_name.endswith('.png')}
        first_img_idx = sorted(list(img_files.keys()))[0]
        first_img_file = img_files[first_img_idx]
        proc_img = process_img(cv2.imread(os.path.join(img_dir, first_img_file)))

        for time_stamp in time_stamps:
            img_path = os.path.join(img_dir, '{}.png'.format(int(time_stamp*100)))
            # img_path = os.path.join(img_dir, img_files[int(time_stamp*100)])
            if not os.path.isfile(img_path):
                print("No image found for the time stamp {}... Using the previous step...".format(time_stamp))
            else:
                img = cv2.imread(os.path.join(img_dir, '{}.png'.format(int(time_stamp*100))))
                proc_img = process_img(img)
            gray_data.append(proc_img)
            count_gray += 1

        obs_data[key] = np.array(gray_data, dtype='uint8')

        samples.append(count_gray)

    obs_file = h5py.File(os.path.join(path, 'gray_obs.hdf5'), 'a')
    obs_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
    obs_group = obs_file.create_group("obs")
    for label, buffer in obs_data.items():
        obs_group.create_dataset(
            label, data=np.array(buffer), compression="gzip", chunks=True, dtype="uint8"
        )
    obs_file.close()


def demo_process(path, demo_mode='robot'):

    low_dim_file = open(os.path.join(path, 'low_dim.csv'), 'r')
    low_dim_reader = csv.reader(low_dim_file)

    # first row is the header
    # create a dict with the header names as keys and save values of the lines to corresponding keys
    data = {
        "time": [],
        "trk_pos": [],
        "trk_rot": [],
        "robot_pos": [],
        "robot_rot": [],
        "grasp": [],
    }

    labels = {'pos': 'trk_pos', 'rot': 'trk_rot', 'grasp': 'grasp'}

    next(low_dim_reader)
    for row in low_dim_reader:
        data['time'] += [float(row[0])]
        data['trk_pos'] += [[float(row[1]), float(row[2]), float(row[3])]]
        data['trk_rot'] += [[float(row[4]), float(row[5]), float(row[6]), float(row[7])]]
        data['robot_pos'] += [[float(row[8]), float(row[9]), float(row[10])]]
        data['robot_rot'] += [[float(row[11]), float(row[12]), float(row[13]), float(row[14])]]
        # data['grasp'] += [[float("True" in row[15])]]
        data['grasp'] += [[float(row[15])]]

    demo_file = h5py.File(os.path.join(path, 'demo.hdf5'), 'w')
    demo_file.create_dataset('time', data=np.array(data['time']), compression="gzip", chunks=True, dtype="f")

    if demo_mode == 'robot':

        mat_tool_offset = np.eye(4)
        mat_tool_offset[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        mat_tool_offset[:3, 3] = np.zeros(3)

        positions = np.array(data['robot_pos'])
        quaternions = np.array(data['robot_rot'])
        cam_pose_vectors = []
        for i in range(len(positions)):
            robot_mat = np.eye(4)
            robot_mat[:3, :3] = geom.quat_to_rot(quaternions[i])
            robot_mat[:3, 3] = positions[i]
            cam_mat = robot_mat @ np.linalg.inv(mat_tool_offset)
            cam_pose_vector = np.concatenate(
                [cam_mat[:3, 3], geom.rot_to_quat(cam_mat[:3, :3])]
            )
            cam_pose_vectors.append(cam_pose_vector)
        actions = np.concatenate([np.array(cam_pose_vectors), np.array(data['grasp'])], axis=1)

    else:
        actions = np.concatenate([np.array(data[labels[key]]) for key in ['pos', 'rot', 'grasp']], axis=1)

    demo_file.create_dataset(
        "action", data=actions, compression="gzip", chunks=True, dtype="f"
    )
    demo_file.close()


def delta_act_process(path, angle_mode='quat'):

    demo_file = h5py.File(os.path.join(path, 'demo.hdf5'), 'r')
    demo_data = np.array(demo_file['action'])
    time_stamps = np.array(demo_file['time'])
    demo_file.close()

    action_data = {
        'pose_ee_vector': demo_data[:,:7],
        'grasp': demo_data[:,7:]
    }
    action_data['delta_pose_vector'] = demo.post_process_actions(action_data['pose_ee_vector'], mode=angle_mode)
    act_concat = np.concatenate(
        (action_data["delta_pose_vector"], action_data["grasp"]), axis=1
    )

    act_file = h5py.File(os.path.join(path, 'delta_{}_act.hdf5'.format(angle_mode)), 'a')
    act_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
    act_file.create_dataset(
        "action", data=act_concat, compression="gzip", chunks=True, dtype="f"
    )
    act_file.close()


def dhb_act_process(path):

    demo_file = h5py.File(os.path.join(path, 'demo.hdf5'), 'r')
    demo_data = np.array(demo_file['action'])
    time_stamps = np.array(demo_file['time'])
    demo_file.close()

    action_data = {
        'positions': demo_data[:,:3],
        'quaternions': demo_data[:,3:7],
        'grasp': demo_data[:,7:]>0.5,
        'linear_motion_invariants': [],
        'angular_motion_invariants': [],
    }
    obs_data = {
        'linear_frame_initial': [],
        'angular_frame_initial': []
    }

    pre_step = 3
    post_step = 3

    pre_pos = [action_data['positions'][0,:]]
    pre_quat = [action_data['quaternions'][0,:]]
    for i in range(pre_step):
        pre_pos = [pre_pos[-1] + np.random.normal(0, 0.001, 3)] + pre_pos
        euler_tmp = geom.quat_to_euler(pre_quat[-1])
        euler_tmp += np.random.normal(0, 0.005, 3)
        pre_quat = [geom.euler_to_quat(euler_tmp)] + pre_quat

    pre_pos = np.array(pre_pos)
    pre_quat = np.array(pre_quat)

    post_pos = np.array([action_data['positions'][-1,:]]*post_step)
    post_quat = np.array([action_data['quaternions'][-1,:]]*post_step)

    ext_positions = np.concatenate((
        pre_pos,
        action_data['positions'],
        post_pos,
        ), axis=0)
    ext_quaternions = np.concatenate((
        pre_quat,
        action_data['quaternions'],
        post_quat,
        ), axis=0)

    for i in range(len(action_data['positions'])):
        out_dict = dhb.compute_DHBs_ext(ext_positions[i:i+4], ext_quaternions[i:i+4], 'vel')
        linear_frame_initial = out_dict['linear_frame_initial']
        angular_frame_initial = out_dict['angular_frame_initial']
        linear_motion_invariants = out_dict['linear_motion_invariants']
        angular_motion_invariants = out_dict['angular_motion_invariants']

        assert linear_motion_invariants.shape[0] == 1
        assert angular_motion_invariants.shape[0] == 1

        obs_data['linear_frame_initial'].append(np.concatenate((linear_frame_initial[0:3,3], geom.rot_to_quat(linear_frame_initial[:3,:3]))))
        obs_data['angular_frame_initial'].append(np.concatenate((angular_frame_initial[0:3,3], geom.rot_to_quat(angular_frame_initial[:3,:3]))))
        action_data['linear_motion_invariants'].append(linear_motion_invariants[-1])
        action_data['angular_motion_invariants'].append(angular_motion_invariants[-1])

    act_concat = np.concatenate((
                                action_data['linear_motion_invariants'],
                                action_data['angular_motion_invariants'],
                                action_data['grasp']>0.5),
                                axis=1)

    obs_file = h5py.File(os.path.join(path, 'dhb_vel_obs.hdf5'), 'a')
    obs_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
    obs_group = obs_file.create_group("obs")
    obs_group.create_dataset('linear_frame_initial',
                            data=np.array(obs_data['linear_frame_initial']),
                            compression="gzip", chunks=True, dtype='f')
    obs_group.create_dataset('angular_frame_initial',
                            data=np.array(obs_data['angular_frame_initial']),
                            compression="gzip", chunks=True, dtype='f')
    obs_file.close()

    act_file = h5py.File(os.path.join(path, 'dhb_vel_act.hdf5'), 'a')
    act_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
    act_file.create_dataset(
        "action", data=act_concat, compression="gzip", chunks=True, dtype="f"
    )
    act_file.close()


def history_obs_process(path):

    demo_file = h5py.File(os.path.join(path, 'demo.hdf5'), 'r')
    demo_data = np.array(demo_file['action'])
    time_stamps = np.array(demo_file['time'])
    demo_file.close()

    action_data = {
        'positions': demo_data[:,:3],
        'quaternions': demo_data[:,3:7],
    }
    position_diffs = np.diff(action_data['positions'], axis=0)
    position_diffs = np.concatenate((np.random.normal(0, 0.0001, (3,3)), position_diffs), axis=0)
    quaternions = np.concatenate(([action_data['quaternions'][0]]*2, action_data['quaternions']), axis=0)

    position_diff_buffer = []
    quaternion_buffer = []
    for idx in range(demo_data.shape[0]):
        position_diff_buffer.append(position_diffs[idx:idx+2].flatten())
        quaternion_buffer.append(quaternions[idx:idx+2].flatten())

    obs_file = h5py.File(os.path.join(path, 'history_obs.hdf5'), 'a')
    obs_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")

    obs_group = obs_file.create_group("obs")
    obs_group.create_dataset(
        "position_diffs",
        data=np.array(position_diff_buffer),
        compression="gzip",
        chunks=True,
        dtype="f",
    )
    obs_group.create_dataset(
        "quaternions",
        data=np.array(quaternion_buffer),
        compression="gzip",
        chunks=True,
        dtype="f",
    )

    # extention for locomotion
    obs_file.close()


def history_act_process(path):

    history_length = 2

    demo_file = h5py.File(os.path.join(path, 'demo.hdf5'), 'r')
    time_stamps = np.array(demo_file['time'])

    demo_data = np.array(demo_file['action'])
    demo_data = demo_data[[0] * history_length + list(range(demo_data.shape[0])),:]

    demo_file.close()

    action_data = {
        'pose_ee_vector': demo_data[:,:7],
        'grasp': demo_data[:,7:] > 0.5
    }
    action_data['delta_pose_vector_rpy'] = demo.post_process_actions(action_data['pose_ee_vector'], mode='rpy')
    action_data['delta_pose_vector_quat'] = demo.post_process_actions(action_data['pose_ee_vector'], mode='quat')

    obs_file = h5py.File(os.path.join(path, 'history_act.hdf5'), 'a')
    obs_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")

    delta_pos_buffer = []
    delta_euler_buffer = []
    delta_quat_buffer = []

    for idx in range(demo_data.shape[0]-history_length):
        delta_pos_buffer.append(action_data['delta_pose_vector_rpy'][idx:idx+history_length,:3].flatten())
        delta_euler_buffer.append(action_data['delta_pose_vector_rpy'][idx:idx+history_length,3:6].flatten())
        delta_quat_buffer.append(action_data['delta_pose_vector_quat'][idx:idx+history_length,3:7].flatten())

    obs_group = obs_file.create_group("obs")
    obs_group.create_dataset(
        "delta_positions",
        data=np.array(delta_pos_buffer),
        compression="gzip",
        chunks=True,
        dtype="f",
    )
    obs_group.create_dataset(
        "delta_eulers",
        data=np.array(delta_euler_buffer),
        compression="gzip",
        chunks=True,
        dtype="f",
    )
    obs_group.create_dataset(
        "delta_quaternions",
        data=np.array(delta_quat_buffer),
        compression="gzip",
        chunks=True,
        dtype="f",
    )

    demo_file.close()
    obs_file.close()


def multiproc_post_process(demo_paths, post_process_fcn, num_workers=12, **kwargs):

    def check_fn_wrapper(fcn):
        def wrappeed_fn(*args, **kwargs):
            try:
                fcn(*args, **kwargs)
            except Exception as e:
                # print("Error in processing the demo data {}".format(kwargs['path']))
                print(e)
        return wrappeed_fn

    jobs = []
    count_demos = 0
    num_demos = len(demo_paths)

    # Process data with a limited worker
    for demo_path in demo_paths:
        while len(jobs) >= num_workers:
            for job in jobs:
                if not job.is_alive():
                    job.join()
                    jobs.remove(job)
                    break
        job = Process(target=post_process_fcn, args=(demo_path,), kwargs=kwargs)
        jobs.append(job)
        job.start()
        count_demos += 1
        print("[{}/{}] Post-processing for the demo \"{}\"...".format(count_demos, num_demos, demo_path))

    for job in jobs:
        job.join()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, default='./data/test')
    parser.add_argument('--mode', type=str, default='record')
    parser.add_argument('--demo_mode', type=str, default='robot')
    parser.add_argument('--angle_mode', type=str, default='rpy')
    args = parser.parse_args()
    path = args.path
    mode = args.mode
    demo_mode = args.demo_mode
    angle_mode = args.angle_mode

    demo_paths = [os.path.join(path, demo_dir) for demo_dir in os.listdir(path) if os.path.isdir(os.path.join(path, demo_dir))]
    if mode == 'obs':
        multiproc_post_process(demo_paths, post_process_fcn=gray_obs_process)
    elif mode == 'history_obs':
        multiproc_post_process(demo_paths, post_process_fcn=history_obs_process)
    elif mode == 'history_act':
        multiproc_post_process(demo_paths, post_process_fcn=history_act_process)
    elif mode == 'demo':
        multiproc_post_process(demo_paths, post_process_fcn=demo_process, demo_mode=demo_mode)
    elif mode == 'delta':
        multiproc_post_process(demo_paths, post_process_fcn=delta_act_process, angle_mode=angle_mode)
    elif mode == 'dhb':
        multiproc_post_process(demo_paths, post_process_fcn=dhb_act_process)