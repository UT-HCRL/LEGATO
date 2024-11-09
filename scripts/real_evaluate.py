import os
import sys
import argparse
import numpy as np
cwd = os.getcwd()
sys.path.append(cwd)

import utils.geom as geom

import copy
import cv2
from scipy.spatial.transform import Rotation as R
from utils.comm import ZMQServer

import torch

from robomimic.utils.file_utils import policy_from_checkpoint
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.obs_utils as ObsUtils

import utils.demo as demo
import collections
import json
from mimic import TrainConfig, policy_from_checkpoint

from devices.t265 import T265
from pynput import keyboard
import csv
import h5py

TARGET_IP = "*"
PUB_PORT = 5000
SUB_PORT = 6000

FPS = 20

PANDA_TOOL_OFFSET = np.array([0.0, 0.0, 0.0, 0.707, 0.0, 0.707, 0.0])

GRASP_THRESHOLD = 0.5
INFERENCE_PERIOD = 0.1

data_path = "data/pick_place/1709248594/low_dim.csv"
with open(data_path, "r") as f:
    reader = csv.reader(f)
    data = list(reader)
    data = np.array(data[1:], dtype=np.float32)
pose_vec_data = np.array(data[:, 8:15], dtype=np.float32)

def process_img(img, resize=(128, 128)):
    # crop the image with the shortest side from the center
    img_shape = img.shape[:2]
    cropped_shape = min(img_shape)
    cropped_img = img[(img_shape[0] - cropped_shape) // 2: (img_shape[0] + cropped_shape) // 2,
                        (img_shape[1] - cropped_shape) // 2: (img_shape[1] + cropped_shape) // 2]
    # gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    proc_img = cv2.resize(cropped_img, resize)
    return np.expand_dims(proc_img, axis=-1)


def unwrap_delta_action(action, command_buffer, angle_mode="rpy"):
    delta_pose_vector = action[:-1]
    pose_ee_vector_prv = command_buffer["pose_ee_vector"][-1]
    pose_ee_vector_cur = demo.reconstruct_pose(
        delta_pose_vector,
        pose_ee_vector_prv,
        mode=angle_mode,
        # delta_pose_vector,
        # pose_ee_vector_prv,
        # mode="quat",
    )
    grasp = bool(action[-1] > GRASP_THRESHOLD)
    return {
        "pose_ee_vector": pose_ee_vector_cur,
        "grasp": grasp,
    }


def unwrap_delta_obs(obs, command_buffer):

    position_diffs = np.array(command_buffer["delta_pose_ee_vector"])[:, :3].flatten()
    # print(position_diffs.round(3))
    quaternions = np.array(command_buffer["pose_ee_vector"])[:, 3:].flatten()
    obs.update(
        {
            "position_diffs": position_diffs,
            "quaternions": quaternions,
        }
    )

    return obs


def evaluate(
    config,
    checkpt_path,
    device="cpu",
    seed=0,
    inference_period=INFERENCE_PERIOD,
    ros_socket=None,
    angle_mode="rpy",
    tool_offset=PANDA_TOOL_OFFSET,
):

    # Configuration
    np.random.seed(seed)
    torch.manual_seed(seed)

    # read config to set up metadata for observation modalities (e.g. detecting rgb observations)
    ObsUtils.initialize_obs_utils_with_config(config)
    eval_policy, ckpt_dict = policy_from_checkpoint(ckpt_path=checkpt_path, device=device)

    mat_tool_offset = np.eye(4)
    # mat_tool_offset[:3, :3] = geom.quat_to_rot(tool_offset[3:])
    mat_tool_offset[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    mat_tool_offset[:3, 3] = tool_offset[:3]

    t265 = T265()
    t265.start()

    done = False
    read_time = 0
    robot_init = True

    command_buffer = {
        "pose_ee_vector": collections.deque(maxlen=2),
        "grasp": collections.deque(maxlen=2),
        "delta_pose_ee_vector": collections.deque(maxlen=2),
    }

    # pred_action = np.zeros(8)
    pred_action = np.zeros(7)

    cnt = 0
    while not done:

        if t265.time > read_time + inference_period:
            left_img = np.copy(t265.left)
            right_img = np.copy(t265.right)
            proc_left_img = process_img(left_img)
            proc_right_img = process_img(right_img)
            read_time += inference_period

            if not robot_init:
                # print(t265.time)
                robot_mat = np.eye(4)
                robot_mat[:3, :3] = geom.quat_to_rot(ros_cmd["pose_ee_vector"][3:])
                robot_mat[:3, 3] = ros_cmd["pose_ee_vector"][:3]
                cam_mat = robot_mat @ np.linalg.inv(mat_tool_offset)
                cam_pose_vector = np.concatenate(
                    [cam_mat[:3, 3], geom.rot_to_quat(cam_mat[:3, :3])]
                )
                command_buffer["delta_pose_ee_vector"].append(pred_action[:-1])
                command_buffer["pose_ee_vector"].append(cam_pose_vector)
                command_buffer["grasp"].append(ros_cmd["grasp"])

                obs = {
                    "left_gray": proc_left_img,
                    "right_gray": proc_right_img,
                }
                obs = unwrap_delta_obs(obs, command_buffer)
                obs = ObsUtils.process_obs_dict(obs)
                pred_action = eval_policy(obs)
                pred_action[:6] = pred_action[:6]
                # pred_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
                # pred_action = replay_actions[cnt]
                ros_cmd = unwrap_delta_action(pred_action, command_buffer, angle_mode)
                cam_new_mat = np.eye(4)
                cam_new_mat[:3, :3] = geom.quat_to_rot(ros_cmd["pose_ee_vector"][3:])
                cam_new_mat[:3, 3] = ros_cmd["pose_ee_vector"][:3]
                robot_new_mat = cam_new_mat @ mat_tool_offset
                ros_cmd["pose_ee_vector"] = np.concatenate(
                    [robot_new_mat[:3, 3], geom.rot_to_quat(robot_new_mat[:3, :3])]
                )
                # ros_cmd["pose_ee_vector"] = np.array(
                #     pose_vec_data[cnt], dtype=np.float32
                # )
                print(ros_cmd)
                ros_socket.update(ros_cmd)
                cnt += 1

        raw_obs = copy.deepcopy(ros_socket.obs)
        # robot_pos = raw_obs["pose_ee_vector"][0:3]
        # robot_quat = raw_obs["pose_ee_vector"][3:7]

        # done = raw_obs['done']

        # print(pos.round(3))

        if robot_init:

            robot_mat = np.eye(4)
            robot_mat[:3, :3] = geom.quat_to_rot(raw_obs["pose_ee_vector"][3:7])
            robot_mat[:3, 3] = raw_obs["pose_ee_vector"][0:3]

            for i in range(2):
                # print(raw_obs)
                command_buffer["grasp"].append(0)
                cam_mat = np.linalg.inv(mat_tool_offset) @ robot_mat
                cam_pose_vector = np.concatenate(
                    [cam_mat[:3, 3], geom.rot_to_quat(cam_mat[:3, :3])]
                )
                command_buffer["pose_ee_vector"].append(cam_pose_vector)
                command_buffer["delta_pose_ee_vector"].append(np.zeros(6))
                # command_buffer["delta_pose_ee_vector"].append(
                #     np.array([0, 0, 0, 0, 0, 0, 1])
                # )

            ros_cmd = {
                "pose_ee_vector": raw_obs["pose_ee_vector"],
                "grasp": False,
            }

            robot_init = False
        # robot_grasp = raw_obs['grasp']

    t265.stop()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--exp", type=str, default="default", help="experiment name")
    parser.add_argument("--label", type=str, default="pouring", help="label name")
    parser.add_argument("--run", type=str, default=None, help="run name")
    parser.add_argument("--epoch", type=int, default=None, help="epoch")
    parser.add_argument(
        "--infer_period", type=float, default=None, help="inference period"
    )
    parser.add_argument("--best", type=int, default=0, help="epoch")
    parser.add_argument("--seed", type=int, default=0, help="random seed")
    parser.add_argument("--angle_mode", type=str, default="rpy", help="angle mode")
    parser.add_argument(
        "--device", type=str, default="cuda", help="device to use: cuda or cpu"
    )
    args = parser.parse_args()

    exp_name = "{}_{}".format(args.exp, args.label)
    load_dir = os.path.join("save", exp_name, args.run)
    seed = args.seed
    inference_period = args.infer_period

    config = TrainConfig()
    with config.values_unlocked():
        config.update(
            json.load(open(os.path.join(load_dir, "logs", "config.json"), "r"))
        )

    if args.device is not None:
        device = args.device
    else:
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    if args.epoch is not None and args.best == 0:
        # checkpt_path = os.path.join(load_dir, 'models', 'model_best_training_at_epoch_{}.pth'.format(args.epoch))
        checkpt_path = os.path.join(
            load_dir, "models", "epoch_{}.pth".format(args.epoch)
        )
    else:
        max_idx = 0
        model_dir = os.path.join(load_dir, "models")
        for file in os.listdir(model_dir):
            if file.startswith("model_best_training_at_epoch_") and file.endswith(
                ".pth"
            ):
                idx = file.split("_")[-1].split(".")[0]
                if int(idx) > max_idx:
                    max_idx = int(idx)
        if args.epoch is not None and args.best == 1:
            checkpt_path = os.path.join(
                model_dir, "model_best_training_at_epoch_{}.pth".format(args.epoch)
            )
        else:
            checkpt_path = os.path.join(
                model_dir, "model_best_training_at_epoch_{}.pth".format(max_idx)
            )

    server = ZMQServer(ip=TARGET_IP, pub_port=PUB_PORT, sub_port=SUB_PORT)
    server.start()

    demo_cnt = 1
    evaluate(
        config,
        checkpt_path,
        inference_period=inference_period,
        device=device,
        seed=seed,
        ros_socket=server,
    )
    server.stop()
