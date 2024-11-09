import os
import sys
import argparse
import collections

import numpy as np
import h5py

cwd = os.getcwd()
sys.path.append(cwd)

import utils.demo as demo
from simulator import *
from simulator.render import CV2Renderer
from simulator.recorder import HDF5Player


def action_process_fcn(player, mode="step"):

    if mode == "reset":
        pass
    if mode == "step":
        pass
    if mode == "close":
        demo_file = h5py.File(os.path.join(player._path, 'demo.hdf5'), 'r')
        act_file = h5py.File(os.path.join(player._path, 'action.hdf5'), 'a')

        demo_data = np.array(demo_file['action'])

        action_data = {
            'pose_ee_vector': demo_data[:,:7],
            'grasp': demo_data[:,7:] > 0.5
        }
        action_data['delta_pose_vector'] = demo.post_process_actions(action_data['pose_ee_vector'], mode='rpy')

        time_stamps = np.array(demo_file['time'])
        act_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
        act_concat = np.concatenate(
            (action_data["delta_pose_vector"], action_data["grasp"] > 0.5), axis=1
        )
        act_file.create_dataset(
            "action", data=act_concat, compression="gzip", chunks=True, dtype="f"
        )
        demo_file.close()
        act_file.close()


def history_obs_process_fcn(player, mode="step"):

    if mode == "reset":
        pass
    if mode == "step":
        pass
    if mode == "close":
        demo_file = h5py.File(os.path.join(player._path, 'demo.hdf5'), 'r')
        obs_file = h5py.File(os.path.join(player._path, 'history_obs.hdf5'), 'a')

        time_stamps = np.array(demo_file['time'])
        obs_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")

        history_length = 2
        demo_data = np.array(demo_file['action'])
        demo_data = demo_data[[0] * history_length + list(range(demo_data.shape[0])),:]

        action_data = {
            'pose_ee_vector': demo_data[:,:7],
            'grasp': demo_data[:,7:] > 0.5
        }
        action_data['delta_pose_vector_rpy'] = demo.post_process_actions(action_data['pose_ee_vector'], mode='rpy')
        action_data['delta_pose_vector_quat'] = demo.post_process_actions(action_data['pose_ee_vector'], mode='quat')

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


def demo_sync_process_fcn(player, mode="step"):

    if mode == "reset":
        player._timestamps = []

    if mode == "step":
        if player._demo_count < player._demo_length:
            player._timestamps.append(player._cur_sim_time)

    if mode == "close":
        demo_file = h5py.File(os.path.join(player._path, 'demo_raw.hdf5'), 'r')
        act_file = h5py.File(os.path.join(player._path, 'demo.hdf5'), 'a')

        demo_data = np.array(demo_file['action'])
        demo_time = np.array(demo_file['time'])
        idx_buffer = []

        for time_stamp in player._timestamps:
            max_idx = 0
            for idx in range(len(demo_time)):
                if demo_time[idx] <= time_stamp:
                    max_idx = idx
            idx_buffer.append(max_idx)

        demo_data = demo_data[idx_buffer]
        time_stamps = np.array(demo_file['time'])

        act_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
        act_file.create_dataset("action", data=demo_data, compression="gzip", chunks=True, dtype="f")

        demo_file.close()
        act_file.close()


def obs_process_fcn(player, mode="step"):

    if mode == "reset":
        player._obs_buffer = {'right_gray': collections.deque(),
                               'left_gray': collections.deque(),
                               }

    if mode == "step":
        if player._demo_count < player._demo_length:
            imgs = player._get_stereo()
            player._obs_buffer['right_gray'].append(np.copy(imgs['right']))
            player._obs_buffer['left_gray'].append(np.copy(imgs['left']))

    elif mode == "close":
        demo_file = h5py.File(os.path.join(player._path, 'demo.hdf5'), 'r')
        obs_file = h5py.File(os.path.join(player._path, 'gray_obs.hdf5'), 'a')

        time_stamps = np.array(demo_file['time'])
        
        obs_file.create_dataset('time', data=np.array(time_stamps), compression="gzip", chunks=True, dtype="f")
        obs_group = obs_file.create_group("obs")
        
        for label, buffer in player._obs_buffer.items():
            obs_group.create_dataset(
                label,
                data=np.array(buffer),
                compression="gzip",
                chunks=True,
                dtype="uint8",
            )
        
        demo_file.close()
        obs_file.close()


def post_process(load_path, task, gripper_tool, robot_name, post_process_fcn, step_player):

    env_class = ENVS[task]
    setup = SETUPS[robot_name]
    robot_type = setup['robot_type']
    env_config = setup['env_config']

    if gripper_tool:
        env_config = tool_env_config_wrapper(env_config, gripper_name="right_gripper")
        setup['hand'] = setup['hand'].replace("eef", "tool")
    
    env_config.TELEOP_TIME = 0.1
    env = env_class(env_config=env_config)

    if gripper_tool:
        renderer_right = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots["right_gripper"].naming_prefix+'stereo_r_right_hand', 
                                    rgb=False,
                                    gray_3ch=True,
                                    gui=False)
        renderer_left = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots["right_gripper"].naming_prefix+'stereo_l_right_hand', 
                                    rgb=False,
                                    gray_3ch=True,
                                    gui=False)
    else:
        renderer_right = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots[robot_type].naming_prefix+'stereo_r_right_hand', 
                                    rgb=False,
                                    gui=False)
        renderer_left = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots[robot_type].naming_prefix+'stereo_l_right_hand', 
                                    rgb=False,
                                    gui=False)
    
    player = HDF5Player(env, mode='replay', post_fcn=post_process_fcn, file_path=load_path)
    player.set_stereo_renderer(renderer_right=renderer_right, renderer_left=renderer_left)

    player.reset()
    if step_player:
        while not player.done:
            player.step()
    player.close()

    renderer_right.close()
    renderer_left.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, default='./data/sim')
    parser.add_argument('--task', type=str, default='lid')
    parser.add_argument('--mode', type=str, default='sync')
    parser.add_argument('--angle_mode', type=str, default='rpy')
    parser.add_argument('--gripper_tool', type=int, default=1)
    parser.add_argument('--robot', type=str, default='abstract')
    args = parser.parse_args()

    path = args.path
    mode = args.mode
    task = args.task
    angle_mode = args.angle_mode
    gripper_tool = args.gripper_tool
    robot_name = args.robot

    demo_paths = [os.path.join(path, demo_dir) for demo_dir in os.listdir(path) if os.path.isdir(os.path.join(path, demo_dir))]

    for cnt, demo_path in enumerate(demo_paths):
        print("[{}/{}] Post-processing for the demo \"{}\"...".format(cnt+1, len(demo_paths), demo_path))
        if mode == 'obs':
            post_process(demo_path, task=task, gripper_tool=gripper_tool, robot_name=robot_name, post_process_fcn=obs_process_fcn, step_player=True)
        if mode == 'history_obs':
            post_process(demo_path, task=task, gripper_tool=gripper_tool, robot_name=robot_name, post_process_fcn=history_obs_process_fcn, step_player=False)
        if mode == 'action':
            post_process(demo_path, task=task, gripper_tool=gripper_tool, robot_name=robot_name, post_process_fcn=action_process_fcn, step_player=False)
        if mode == 'sync':
            post_process(demo_path, task=task, gripper_tool=gripper_tool, robot_name=robot_name, post_process_fcn=demo_sync_process_fcn, step_player=True)
