import os
import sys
import argparse
import numpy as np
cwd = os.getcwd()
sys.path.append(cwd)

# from devices.t265 import T265
import utils.geom as geom
import csv

import copy
import io
import time
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from utils.comm import ZMQServer

from multiprocessing import Process
import h5py
import utils.demo as demo
import utils.dhb as dhb

# Define colors for each axis
AXIS_COLORS = ['r', 'g', 'b']  # Red for x, Green for y, Blue for z

TARGET_IP = "*"
PUB_PORT = 5000
SUB_PORT = 6000

FPS = 20


def record(path, ros_socket=None):

    t265 = T265()
    mat_trans = np.eye(4)
    mat_trans[:3, :3] = geom.quat_to_rot([0.5, -0.5, -0.5, 0.5]) #T265
    trk_init = True

    t265.start()

    done = False
    read_time = 0

    # state = {key: [] for key in ['time', 'left_img', 'right_img', 'trk_pos', 'trk_rot', 'robot_pos', 'robot_rot', 'robot_grasp']}

    dir_name = "{}".format(int(time.time()))
    dir_path = os.path.join(path, dir_name)
    left_img_path = os.path.join(dir_path, 'left_img')
    right_img_path = os.path.join(dir_path, 'right_img')

    os.makedirs(dir_path, exist_ok=True)
    os.makedirs(left_img_path, exist_ok=True)
    os.makedirs(right_img_path, exist_ok=True)

    low_dim_file = open(os.path.join(dir_path, 'low_dim.csv'), 'w')
    low_dim_file.write("time, \
                       pos_x, pos_y, pos_z, \
                       quat_x, quat_y, quat_z, quat_w, \
                       robot_pos_x, robot_pos_y, robot_pos_z, \
                       robot_quat_x, robot_quat_y, robot_quat_z, robot_quat_w, \
                       robot_grasp\
                       \n")
    # 0 3 7 10 14 15

    pos = np.zeros(3)
    quat = np.array([0, 0, 0, 0])

    while not done:

        if t265.time > read_time + 0.1:
            trk_pos = t265.pos
            trk_rot = t265.rot
            left_img = np.copy(t265.left)
            right_img = np.copy(t265.right)
            read_time += 0.1
            # print("[{:.3f}], pos: {}, quat: {}".format(t265.time, pos.round(3), quat.round(3)))

            obs = copy.deepcopy(ros_socket.obs)
            done = obs["done"]

            mat_se3 = np.eye(4)
            mat_se3[:3, :3] = geom.quat_to_rot(trk_rot)
            mat_se3[:3, 3] = trk_pos

            if trk_init:
                mat_se3_base = np.eye(4)
                mat_se3_base = np.linalg.inv(mat_trans @ mat_se3) @ mat_se3_base
                trk_init = False
                print("Tracking initialized... Start now")

            trk_mat_se3 = mat_trans @ mat_se3 @ mat_se3_base
            pos = trk_mat_se3[:3, 3]
            quat = geom.rot_to_quat(trk_mat_se3[:3, :3])
            robot_pos = obs["pose_ee_vector"][0:3]
            robot_quat = obs["pose_ee_vector"][3:7]
            robot_grasp = obs["grasp"]

            # state['time'] += [t265.time]
            # state['left_img'] += [[left_img]]
            # state['right_img'] += [[right_img]]
            # state['trk_pos'] += [pos]
            # state['trk_rot'] += [quat]
            # state['robot_pos'] += [robot_pos]
            # state['robot_rot'] += [robot_quat]
            # state['grasp'] += [robot_grasp]

            if not trk_init:

                low_dim_file.write(
                    "{}, \
                                    {}, {}, {},\
                                    {}, {}, {}, {},\
                                    {}, {}, {},\
                                    {}, {}, {}, {},\
                                    {}\
                                \n".format(
                        t265.time,
                        pos[0],
                        pos[1],
                        pos[2],
                        quat[0],
                        quat[1],
                        quat[2],
                        quat[3],
                        robot_pos[0],
                        robot_pos[1],
                        robot_pos[2],
                        robot_quat[0],
                        robot_quat[1],
                        robot_quat[2],
                        robot_quat[3],
                        obs["grasp"],
                    )
                )
                cv2.imwrite(
                    os.path.join(left_img_path, "{}.png".format(int(t265.time * 100))),
                    left_img,
                )
                cv2.imwrite(
                    os.path.join(right_img_path, "{}.png".format(int(t265.time * 100))),
                    right_img,
                )

    t265.stop()

    if input("Do you want to keep the data? (y/n): ") == "n":
        os.system("rm -rf {}".format(dir_path))

def visualize(data_path):

    low_dim_file = open(os.path.join(data_path, 'low_dim.csv'), 'r')
    low_dim_reader = csv.reader(low_dim_file)
    left_img_dir = os.path.join(data_path, 'left_img')
    right_img_dir = os.path.join(data_path, 'right_img')

    # first row is the header
    # create a dict with the header names as keys and save values of the lines to corresponding keys
    data = {
        "time": [],
        "trk_pos": [],
        "trk_rot": [],
        "robot_pos": [],
        "robot_rot": [],
    }

    next(low_dim_reader)
    for row in low_dim_reader:
        data['time'] += [float(row[0])]
        data['trk_pos'] += [[float(row[1]), float(row[2]), float(row[3])]]
        data['trk_rot'] += [[float(row[4]), float(row[5]), float(row[6]), float(row[7])]]
        data["robot_pos"] += [[float(row[8]), float(row[9]), float(row[10])]]
        data["robot_rot"] += [
            [float(row[11]), float(row[12]), float(row[13]), float(row[14])]
        ]

    for key in data.keys():
        data[key] = np.array(data[key])

    mat_tool_offset = np.eye(4)
    mat_tool_offset[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    mat_tool_offset[:3, 3] = np.zeros(3)

    data['left_img'] = []
    data['right_img'] = []
    left_img = np.zeros((800, 848, 3), dtype="uint8")
    right_img = np.zeros((800, 848, 3), dtype="uint8")
    for time in data['time']:
        left_img_read = cv2.imread(
            os.path.join(left_img_dir, "{}.png".format(int(time * 100)))
        )
        right_img_read = cv2.imread(
            os.path.join(right_img_dir, "{}.png".format(int(time * 100)))
        )
        data['left_img'] += [left_img]
        data['right_img'] += [right_img]
        # print(right_img)
        if left_img_read is not None:
            left_img = left_img_read
        if right_img_read is not None:
            right_img = right_img_read
        # print(left_img.shape, right_img.shape)

    time = np.array(data['time'][:])
    left_img = np.array(data['left_img'][:])
    right_img = np.array(data['right_img'][:])
    trk_pos = np.array(data["trk_pos"][:])
    trk_rot = np.array(data["trk_rot"][:])
    # ipdb.set_trace()

    robot_pos = np.array(data["robot_pos"][:])
    robot_rot = np.array(data["robot_rot"][:])

    cmd_pos =[]
    cmd_rot = []
    for i in range(len(robot_pos)):
        robot_mat = np.eye(4)
        robot_mat[:3, :3] = geom.quat_to_rot(robot_rot[i])
        robot_mat[:3, 3] = robot_pos[i]
        cam_mat = robot_mat @ np.linalg.inv(mat_tool_offset)
        cmd_pos.append(cam_mat[:3, 3])
        cmd_rot.append(geom.rot_to_quat(cam_mat[:3, :3]))
    cmd_pos = np.array(cmd_pos)
    cmd_rot = np.array(cmd_rot)

    initial_mat = np.eye(4)
    initial_mat[:3, :3] = geom.quat_to_rot(robot_rot[0])
    initial_mat[:3, 3] = robot_pos[0]

    trk_action = []
    for i in range(trk_pos.shape[0]-1):
        pose_vec_cur = np.concatenate([trk_pos[i+1], trk_rot[i+1]])
        pose_vec_prv = np.concatenate([trk_pos[i], trk_rot[i]])
        action = demo.reconstruct_pose(pose_vec_cur, pose_vec_prv, mode='quat')
        trk_action.append(action)
    trk_action = np.array(trk_action)

    cmd_action = []
    for i in range(cmd_pos.shape[0]-1):
        pose_vec_cur = np.concatenate([cmd_pos[i+1], cmd_rot[i+1]])
        pose_vec_prv = np.concatenate([cmd_pos[i], cmd_rot[i]])
        action = demo.reconstruct_pose(pose_vec_cur, pose_vec_prv, mode='quat')
        cmd_action.append(action)
    cmd_action = np.array(cmd_action)

    estimated_pos = []
    estimated_rot = []
    pose_ee_vector_prv = np.concatenate([mat_tool_offset[:3, 3], geom.rot_to_quat(mat_tool_offset[:3, :3])])
    for i in range(trk_action.shape[0]-1):
        delta_pose_vector = trk_action[i]
        pose_ee_vector_cur = demo.reconstruct_pose(
            delta_pose_vector, pose_ee_vector_prv, mode="quat"
        )
        cam_pose_mat = np.eye(4)
        cam_pose_mat[:3, :3] = geom.quat_to_rot(pose_ee_vector_cur[3:])
        cam_pose_mat[:3, 3] = pose_ee_vector_cur[:3]
        robot_pose_mat = cam_pose_mat @ mat_tool_offset
        estimated_pos.append(robot_pose_mat[:3, 3])
        estimated_rot.append(geom.rot_to_quat(robot_pose_mat[:3, :3]))
    estimated_pos = np.array(estimated_pos)
    estimated_rot = np.array(estimated_rot)

    reconstructed_pos = []
    reconstructed_rot = []
    pose_ee_vector_prv = np.concatenate([mat_tool_offset[:3, 3], geom.rot_to_quat(mat_tool_offset[:3, :3])])
    for i in range(cmd_action.shape[0]-1):
        delta_pose_vector = cmd_action[i]
        pose_ee_vector_cur = demo.reconstruct_pose(
            delta_pose_vector, pose_ee_vector_prv, mode="quat"
        )
        cam_pose_mat = np.eye(4)
        cam_pose_mat[:3, :3] = geom.quat_to_rot(pose_ee_vector_cur[3:])
        cam_pose_mat[:3, 3] = pose_ee_vector_cur[:3]
        robot_pose_mat = cam_pose_mat @ mat_tool_offset
        reconstructed_pos.append(robot_pose_mat[:3, 3])
        reconstructed_rot.append(geom.rot_to_quat(robot_pose_mat[:3, :3]))
    reconstructed_pos = np.array(reconstructed_pos)
    reconstructed_rot = np.array(reconstructed_rot)

    fig = plt.figure()
    ax1 = fig.add_subplot(221)
    ax2 = fig.add_subplot(222)
    ax3 = fig.add_subplot(223, projection='3d')
    ax4 = fig.add_subplot(224, projection='3d')

    length = time.shape[0]
    img_buffer = []

    def get_img_from_fig(fig, dpi=360):
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=dpi)
        buf.seek(0)
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        img = cv2.imdecode(img_arr, 1)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2gray)
        return img

    for num in range(length):

        # Prepare the 2D plot for visual observation
        pos_data_3 = estimated_pos[:num+1, :]
        rot_data_3 = estimated_rot[:num+1, :]

        pos_data_4 = reconstructed_pos[:num+1, :]
        rot_data_4 = reconstructed_rot[:num+1, :]

        left_data = left_img[num]
        right_data = right_img[num]

        fig.suptitle('Time: %.2f' % time[num])
        fig.set_size_inches(5, 2.5)
        if num == 0:
            # fig.set_size_inches(15, 7)
            left_image = ax1.imshow(left_img[0])
            ax1.set_title('Left image', fontdict={'fontsize': 8})
            ax1.tick_params(left = False, right = False , labelleft = False ,
                            labelbottom = False, bottom = False)

            right_image = ax2.imshow(right_img[0])
            ax2.set_title('Right image', fontdict={'fontsize': 8})
            ax2.tick_params(left = False, right = False , labelleft = False ,
                            labelbottom = False, bottom = False)

            line_3, = ax3.plot(pos_data_3[0,0], pos_data_3[0,1], pos_data_3[0,2], color='blue')
            # point_3, = ax3.plot([], [], [], 'go')
            axis_3 = []
            for j in range(3):
                axis_j, = ax3.plot([], [], [], color=AXIS_COLORS[j])
                axis_3.append(axis_j)
            ax3.set_xlim(pos_data_3[0,0]-0.6, pos_data_3[0,0]+0.6)
            ax3.set_ylim(pos_data_3[0,1]-0.6, pos_data_3[0,1]+0.6)
            ax3.set_zlim(pos_data_3[0,2]-0.6, pos_data_3[0,2]+0.6)
            ax3.xaxis.set_tick_params(labelsize=4, pad=0)
            ax3.yaxis.set_tick_params(labelsize=4, pad=0)
            ax3.zaxis.set_tick_params(labelsize=4, pad=0)
            ax3.set_title('Tracker trajectory', fontdict={'fontsize': 8})
            ax3.grid(True)

            line_4, = ax4.plot(pos_data_4[0,0], pos_data_4[0,1], pos_data_4[0,2], color='blue')
            # point_4, = ax4.plot([], [], [], 'go')
            axis_4 = []
            for j in range(3):
                axis_j, = ax4.plot([], [], [], color=AXIS_COLORS[j])
                axis_4.append(axis_j)
            ax4.set_xlim(pos_data_4[0,0]-0.6, pos_data_4[0,0]+0.6)
            ax4.set_ylim(pos_data_4[0,1]-0.6, pos_data_4[0,1]+0.6)
            ax4.set_zlim(pos_data_4[0,2]-0.6, pos_data_4[0,2]+0.6)
            ax4.xaxis.set_tick_params(labelsize=4, pad=0)
            ax4.yaxis.set_tick_params(labelsize=4, pad=0)
            ax4.zaxis.set_tick_params(labelsize=4, pad=0)
            ax4.set_title('Command trajectory', fontdict={'fontsize': 8})
            ax4.grid(True)

        left_image.set_array(left_data)
        right_image.set_array(right_data)

        line_3.set_data(pos_data_3[:,0], pos_data_3[:,1])
        line_3.set_3d_properties(pos_data_3[:,2])
        # Create a basis in the orientation of the quaternion
        rot_3 = R.from_quat(rot_data_3[-1])
        basis_3 = rot_3.apply(np.eye(3))
        # Plot each basis vector
        for j in range(3):
            start_point = pos_data_3[-1]
            end_point = pos_data_3[-1] + 0.1 * basis_3[j]
            axis_3[j].set_data([start_point[0], end_point[0]], [start_point[1], end_point[1]])
            axis_3[j].set_3d_properties([start_point[2], end_point[2]])
        # point_3.set_data(pos_data_3[-1,0], pos_data_3[-1,1])
        # point_3.set_3d_properties(pos_data_3[-1,2])

        line_4.set_data(pos_data_4[:,0], pos_data_4[:,1])
        line_4.set_3d_properties(pos_data_4[:,2])
        # Create a basis in the orientation of the quaternion
        rot_4 = R.from_quat(rot_data_4[-1])
        basis_4 = rot_4.apply(np.eye(3))
        # Plot each basis vector
        for j in range(3):
            start_point = pos_data_4[-1]
            end_point = pos_data_4[-1] + 0.1 * basis_4[j]
            axis_4[j].set_data([start_point[0], end_point[0]], [start_point[1], end_point[1]])
            axis_4[j].set_3d_properties([start_point[2], end_point[2]])
        # point_4.set_data(pos_data_4[-1,0], pos_data_4[-1,1])
        # point_4.set_3d_properties(pos_data_4[-1,2])

        img = get_img_from_fig(fig)
        img_buffer.append(img)

    save_path = os.path.join(data_path, "visualize_trk.mp4")
    video_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), 30,  img_buffer[0].shape[1::-1])
    for img in img_buffer:
        video_writer.write(img)
    video_writer.release()


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
        # img_files = {int(file_name.split('.')[0]): file_name for file_name in os.listdir(path) if file_name.endswith('.png')}

        for time_stamp in time_stamps:
            img_path = os.path.join(img_dir, '{}.png'.format(int(time_stamp*100)))
            # img_path = os.path.join(img_dir, img_files[int(time_stamp*100)])
            if not os.path.isfile(img_path):
                print("No image found for the time stamp {}... Using the previous step...".format(time_stamp))
                gray_data.append(proc_img)
                count_gray += 1
                continue
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
        data['grasp'] += [[int(row[15])]]

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
        p = Process(target=post_process_fcn, args=(demo_path,), kwargs=kwargs)
        p.start()
        count_demos += 1
        print("[{}/{}] Post-processing for the demo \"{}\"...".format(count_demos, num_demos, demo_path))

    for proc in jobs:
        proc.join()


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

    if mode == 'record':
        server = ZMQServer(ip=TARGET_IP, pub_port=PUB_PORT, sub_port=SUB_PORT)
        server.start()

        demo_cnt = 1

        while True:
            while not server.obs or server.obs["done"]:
                pass
            print("Recording {}th demo...".format(demo_cnt))
            record(path, ros_socket=server)
            print("Pause, waiting for next demo...")
            demo_cnt += 1

        server.stop()
    if mode == 'visualize':
        visualize(path)
    else:
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
