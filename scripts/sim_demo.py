import os
import sys
import time
import argparse

import numpy as np
from pynput import keyboard

cwd = os.getcwd()
sys.path.append(cwd)

import utils.geom as geom
from simulator import *
from simulator.wrapper import SimRobotEnvWrapper, SimEnvWrapper
from simulator.wrapper import unwrap_delta_action, unwrap_delta_obs
from simulator.render import CV2Renderer
from simulator.recorder import HDF5Recorder
from devices.t265 import T265


## Define the thread receiving keyboard for debugging
class Keyboard():

    def __init__(self):

        self.single_click_and_hold = False

        self._reset_state = 0
        self._enabled = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1

        self._succeed = False

        # launch a new listener thread to listen to keyboard
        self.thread = keyboard.Listener(on_press=self._on_press,
                                        on_release=self._on_release)
        self.thread.start()


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset grasp
        self.single_click_and_hold = False

        self._flag_init = False
        self._t_last_click = - 1
        self._t_click = - 1


    def _on_press(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char == 'e':
            self._t_last_click = -1
            self._t_click = time.time()
            elapsed_time = self._t_click - self._t_last_click
            self._t_last_click = self._t_click
            self.single_click_and_hold = True
        elif key_char == 's':
            self._succeed = True
            print('Recording successful')


    def _on_release(self, key):

        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char == 'e':
            self.single_click_and_hold = False

        elif key == keyboard.Key.esc or key_char == 'q':
            self._reset_state = 1
            self._enabled = False
            self._reset_internal_state()

        elif key_char == 'r':
            self._reset_state = 1
            self._enabled = True


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        # Reset grasp
        self.single_click_and_hold = False


    @property
    def click(self):
        """
        Maps internal states into gripper commands.
        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0


    @property
    def enable(self):
        return self._enabled


    @property
    def succeed(self):
        return self._succeed


def sim_demo(gui, save_path, task, robot_name="spot", cam_name="upview", gripper_tool=0):

    dir_name = "{}".format(int(time.time()))
    dir_path = os.path.join(save_path, dir_name)

    t265 = T265(img_stream=False)
    keyboard = Keyboard()
    mat_trans = np.eye(4)
    mat_trans[:3, :3] = geom.quat_to_rot([0.5, -0.5, -0.5, 0.5]) #T265
    trk_init = True

    env_class = ENVS[task]
    setup = SETUPS[robot_name]
    env_config = setup['env_config']
    if gripper_tool:
        env_config = tool_env_config_wrapper(env_config, gripper_name="right_gripper")
        setup['hand'] = setup['hand'].replace("eef", "tool")

    env = env_class(env_config=env_config)
    recorder = HDF5Recorder(config=env.config, sim=env.sim, file_path=dir_path)
    renderer = CV2Renderer(device_id=-1, sim=env.sim, height=900, width=1200, cam_name=cam_name, gui=gui)

    if robot_name == 'abstract':
        env_wrapper = SimEnvWrapper
    else:
        env_wrapper = SimRobotEnvWrapper

    env = env_wrapper(env, setup, angle_mode='rpy',
                        img_mode='gray',
                        unwrap_action_fn=unwrap_delta_action,
                        unwrap_obs_fn=unwrap_delta_obs)

    env.set_renderer(renderer)
    env.set_recorder(recorder)

    t265.start()
    raw_ob = env.reset()

    done = False
    while not keyboard.enable:
        pass

    read_time = 0
    prv_se3 = np.eye(4)
    prv_se3[:3, 3] = np.array(env._default_hand_pose['pos'])

    while not done:

        if t265.time > read_time + 0.1:
            trk_pos = t265.pos
            trk_rot = t265.rot
            read_time += 0.1

        # T265 POSE
        mat_se3 = np.eye(4)
        mat_se3[:3, :3] = geom.quat_to_rot(trk_rot)
        mat_se3[:3, 3] = trk_pos

        if trk_init:
            mat_se3_base = np.eye(4)
            mat_se3_base = np.linalg.inv(mat_trans @ mat_se3) @ mat_se3_base
            trk_init = False

        trk_mat_se3 =  mat_trans @ mat_se3 @ mat_se3_base
        pos = trk_mat_se3[:3, 3]
        quat = geom.rot_to_quat(trk_mat_se3[:3, :3])

        cur_se3 = np.eye(4)
        cur_se3[:3, :3] = geom.quat_to_rot(quat)
        cur_se3[:3, 3] = 5*pos + np.array(env._default_hand_pose['pos'])
        delta_se3 = cur_se3 @ np.linalg.inv(prv_se3)
        prv_se3 = cur_se3

        act_pos = delta_se3[:3, 3]
        act_rpy = geom.rot_to_euler(delta_se3[:3, :3])
        act_gripper = 1.0 * keyboard.click

        action = np.concatenate((act_pos, act_rpy, [act_gripper]))

        _ = env.step(action)

        done = not keyboard.enable

    t265.stop()
    renderer.close()
    if keyboard._succeed:
        print("Recording successful")
        recorder.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", type=int, default=1, help="")
    parser.add_argument("--robot", type=str, default='abstract', help="")
    parser.add_argument('--gripper_tool', type=int, default=1)
    parser.add_argument('--task', type=str, default='ladle')
    parser.add_argument('--path', type=str, default='./data/lid')
    parser.add_argument('--hand_link', type=str, default='eef_point')
    parser.add_argument('--gripper_joint', type=str, default='joint_0')
    parser.add_argument(
        "--cam",
        type=str,
        default='demoview',
        help="",
    )
    args = parser.parse_args()

    gui = args.gui
    cam_name = args.cam
    gripper_tool = args.gripper_tool
    path = args.path 
    task = args.task
    robot_name = args.robot

    while True:
        sim_demo(save_path=path, gui=gui, cam_name=cam_name, robot_name=robot_name, task=task, gripper_tool=gripper_tool)
