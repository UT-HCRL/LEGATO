import os
import sys

import argparse
import numpy as np
import torch
import json

import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.obs_utils as ObsUtils

cwd = os.getcwd()
sys.path.append(cwd)

from simulator import *
from simulator.wrapper import SimRobotEnvWrapper, SimEnvWrapper
from simulator.wrapper import unwrap_delta_action, unwrap_delta_obs
from simulator.render import CV2Renderer
from mimic import TrainConfig, policy_from_checkpoint


def evaluate(config, ckpt_path,
            gripper_tool, task, robot_name,
            gui, cam_name,
            seed=0, time_max=40.0, device='cpu'):

    # random seed
    np.random.seed(seed)
    torch.manual_seed(seed)

    # read config to set up metadata for observation modalities
    ObsUtils.initialize_obs_utils_with_config(config)
    eval_policy, ckpt_dict = policy_from_checkpoint(ckpt_path=ckpt_path, device=device)

    # build environment
    env_class = ENVS[task]
    setup = SETUPS[robot_name]
    robot_type = setup['robot_type']
    env_config = setup['env_config']

    if gripper_tool:
        env_config = tool_env_config_wrapper(env_config, gripper_name="right_gripper")
        setup['hand'] = setup['hand'].replace("eef", "tool")

    env = env_class(env_config=env_config)
    renderer = CV2Renderer(device_id=-1, 
                           sim=env.sim, gui=gui, width=400, height=300,
                           cam_name=cam_name
                           )

    if gripper_tool:
        renderer_right = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots["right_gripper"].naming_prefix+'stereo_r_right_hand', 
                                    rgb=False, gui=False)
        renderer_left = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots["right_gripper"].naming_prefix+'stereo_l_right_hand', 
                                    rgb=False, gui=False)
    else:
        renderer_right = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots[robot_type].naming_prefix+'stereo_r_right_hand', 
                                    rgb=False, gui=False)
        renderer_left = CV2Renderer(device_id=-1, sim=env.sim, width=128, height=128, 
                                    cam_name=env.robots[robot_type].naming_prefix+'stereo_l_right_hand', 
                                    rgb=False, gui=False)

    if robot_name == 'abstract':
        env_wrapper = SimEnvWrapper
    else:
        env_wrapper = SimRobotEnvWrapper

    env = env_wrapper(env, setup, angle_mode='rpy', img_mode='gray',
                        unwrap_action_fn=unwrap_delta_action,
                        unwrap_obs_fn=unwrap_delta_obs)

    env.set_renderer(renderer)
    env.set_stereo_renderer(renderer_right=renderer_right, renderer_left=renderer_left)

    raw_ob = env.reset()
    done = False

    while not done:

        ob = ObsUtils.process_obs_dict(raw_ob)
        action = eval_policy(ob)
        raw_ob, _, done, _ = env.step(action)

        if env.cur_time > time_max:
            done = True

    env.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", type=int, default=1, help="gui enabled or not")
    parser.add_argument("--exp", type=str, default='default', help="experiment name")
    parser.add_argument("--task", type=str, default='pouring', help="task name")
    parser.add_argument("--gripper_tool", type=int, default=1, help="using the LEGATO Gripper")
    parser.add_argument("--robot", type=str, default="spot", help="deployment robot")
    parser.add_argument("--device", type=str, default=None, help="device to use: cuda or cpu")
    parser.add_argument("--seed", type=int, default=0, help="random seed")
    parser.add_argument('--ckpt_path', type=str, default='./test_eval.pth')
    parser.add_argument('--cfg_path', type=str, default='./configs/sim.json')
    parser.add_argument('--time_max', type=float, default=40.0)
    parser.add_argument("--cam", type=str, default='frontview', help="")
    args = parser.parse_args()

    task=args.task
    gui = args.gui
    cam_name = args.cam
    robot_name = args.robot
    time_max = args.time_max
    gripper_tool = args.gripper_tool
    seed = args.seed
    ckpt_path = args.ckpt_path
    cfg_path=args.cfg_path

    config = TrainConfig()
    with config.values_unlocked():
        config.update(json.load(open(cfg_path, 'r')))

    if args.device is not None:
        device = args.device
    else:
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    evaluate(config, ckpt_path, gripper_tool=gripper_tool, task=task, robot_name=robot_name,
            seed=seed, gui=gui, cam_name=cam_name,
            time_max=time_max, device=device)