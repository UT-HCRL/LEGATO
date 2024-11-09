import copy
import collections
import numpy as np

import gym

import utils.geom as geom
import utils.demo as demo


def wrap_delta_action(command_cur, command_buffer):
    pose_ee_vector_prv = command_buffer["pose_ee_vector"][-1]
    pose_ee_vector_cur = demo.post_process_action(
        pose_ee_vector_cur, pose_ee_vector_prv
    )
    gripper = command_cur["gripper"]
    return np.concatenate([pose_ee_vector_cur, gripper])


def unwrap_delta_action(action, command_buffer, mode="quat"):
    delta_pose_vector = action[:-1]
    pose_ee_vector_prv = command_buffer["pose_ee_vector"][-1]
    pose_ee_vector_cur = demo.reconstruct_pose(
        delta_pose_vector, pose_ee_vector_prv, mode=mode
    )
    gripper = 0.8 * float(action[-1] > 0.5)
    return {
        "pose_ee_vector": pose_ee_vector_cur,
        "gripper": gripper,
    }


def unwrap_delta_obs(obs, command_buffer):

    position_diffs = np.array(command_buffer["delta_pose_ee_vector"])[:, :3].flatten()
    quaternions = np.array(command_buffer["pose_ee_vector"])[:, 3:].flatten()
    delta_positions = np.array(command_buffer["delta_positions"]).flatten()
    delta_eulers = np.array(command_buffer["delta_eulers"]).flatten()
    out_obs ={
                "position_diffs": position_diffs,
                "quaternions": quaternions,
                "delta_positions": delta_positions,
                "delta_eulers": delta_eulers,
            }

    return out_obs


class SimEnvWrapper(gym.Env):
    def __init__(
        self,
        env,
        setup, 
        angle_mode="quat",
        img_mode="rgb",
        unwrap_action_fn=unwrap_delta_action,
        unwrap_obs_fn=unwrap_delta_obs,
    ) -> None:
        self._env = env
        self.robots = env.robots
        self._renderer_right = None
        self._renderer_left = None
        self.sim = self._env.sim
        self._unwrap_action = unwrap_action_fn
        self._unwrap_obs = unwrap_obs_fn
        self._policy_period = env.config.POLICY_TIME
        self._recorder = None
        self._angle_mode = angle_mode
        self._img_mode = img_mode

        self._gripper_tools = {}
        for robot_name, robot_config in self._env.config.ROBOT_CONFIG.items():
            if robot_config["type"] == "LegatoGripper":
                self._gripper_tools[robot_name] = copy.deepcopy(self._env.config.CONTROL_CONFIG[robot_name]['init_state'])
        self._default_hand_pose = setup["ik_config"].DEFAULT_HAND_POSE


    def reset(self, sim_reset=True, **kwargs):
        if sim_reset:
            obs = self._env.reset(**kwargs)
        else:
            obs = self._env._get_obs()
        self._cur_obs = {}
        self._cur_action = copy.deepcopy(self._env._cur_action)
        self._command_buffer = {
            "pose_ee_vector": collections.deque(maxlen=2),
            "gripper": collections.deque(maxlen=2),
            "delta_pose_ee_vector": collections.deque(maxlen=2),
            "delta_positions": collections.deque(maxlen=2),
            "delta_eulers": collections.deque(maxlen=2),
        }
        pose_ee_vector_prv = np.concatenate(
            [
                self._cur_action["demo_link"]["joint_right_hand"]["pos"],
                self._cur_action["demo_link"]["joint_right_hand"]["quat"],
            ]
        )
        gripper_prv = self._cur_action["right_gripper"]["joint_0"]

        for i in range(2):
            self._command_buffer["gripper"].append(gripper_prv)
            self._command_buffer["pose_ee_vector"].append(pose_ee_vector_prv)
            if self._angle_mode == "quat":
                self._command_buffer["delta_pose_ee_vector"].append(
                    np.array([0, 0, 0, 0, 0, 0, 1])
                )
            else:
                self._command_buffer["delta_pose_ee_vector"].append(np.zeros(6))

            self._command_buffer["delta_positions"].append(
                np.array([0, 0, 0])
            )
            self._command_buffer["delta_eulers"].append(
                np.array([0, 0, 0])
            )

        self._policy_time = self.cur_time

        self._update_obs(obs)
        return self._get_obs()

    def step(self, action=None):
        self._update_action(action)
        while not self.cur_time - self._policy_time > self._policy_period:
            obs = self._env.step(self._get_action())
        self._policy_time += self._policy_period
        self._update_obs(obs)

        return self._get_obs(), 0, self.done, {}

    def _update_action(self, action):

        pose_ee_vector_prv = np.concatenate(
            [
                self._cur_action["demo_link"]["joint_right_hand"]["pos"],
                self._cur_action["demo_link"]["joint_right_hand"]["quat"],
            ]
        )
        gripper_prv = self._cur_action["right_gripper"]["joint_0"]

        self._command_buffer["delta_positions"].append(action[:3])
        if self._angle_mode == "quat":
            self._command_buffer["delta_eulers"].append(geom.quat_to_euler(action[3:-1]))
        else:
            self._command_buffer["delta_eulers"].append(action[3:-1])
        self._command_buffer["delta_pose_ee_vector"].append(action[:-1])
        self._command_buffer["pose_ee_vector"].append(pose_ee_vector_prv)
        self._command_buffer["gripper"].append(gripper_prv)

        command_cur = self._unwrap_action(
            action, self._command_buffer, mode=self._angle_mode
        )

        self._cur_action.update(
            {
                "demo_link": {
                    "joint_right_hand": {
                        "pos": command_cur["pose_ee_vector"][:3],
                        "quat": command_cur["pose_ee_vector"][3:],
                    }
                },
                "right_gripper": {"joint_0": command_cur["gripper"]},
            }
        )

        if self._recorder is not None:
            command = {
                "pos": command_cur["pose_ee_vector"][:3],
                "quat": command_cur["pose_ee_vector"][3:],
                "gripper": command_cur["gripper"],
            }
            self._recorder._update_commands(command)


    def _get_obs(self):
        return copy.deepcopy(self._cur_obs)

    def _get_action(self):
        action = copy.deepcopy(self._cur_action)
        for gripper_name, tool_action in self._gripper_tools.items():
            gripper_action = action[gripper_name]
            grasping = float(gripper_action["joint_0"] > 0.5)
            direction = -1 if grasping else 1
            left_action = np.clip(tool_action["joint_l"] + direction * self._env.config.CONTROL_CONFIG[gripper_name]['vel_max'] * self._env.config.CONTROL_TIME, 
                                    self._env.config.CONTROL_CONFIG[gripper_name]['pos_min'], 
                                    self._env.config.CONTROL_CONFIG[gripper_name]['pos_max'])
            right_action = np.clip(tool_action["joint_r"] - direction * self._env.config.CONTROL_CONFIG[gripper_name]['vel_max'] * self._env.config.CONTROL_TIME, 
                                    -self._env.config.CONTROL_CONFIG[gripper_name]['pos_max'], 
                                    self._env.config.CONTROL_CONFIG[gripper_name]['pos_min'])
            tool_action.update({
                'joint_l': left_action,
                'joint_r': right_action,
            })
            action[gripper_name] = tool_action

        return action
    
    def _update_obs(self, obs):
        processed_obs = self._unwrap_obs(obs, self._command_buffer)
        self._cur_obs.update(processed_obs)
        obs_imgs = self._get_stereo()
        if obs_imgs is not None:
            self._cur_obs.update(
                {
                    "right_{}".format(self._img_mode): np.array(obs_imgs["right"], dtype=np.uint8),
                    "left_{}".format(self._img_mode): np.array(obs_imgs["left"], dtype=np.uint8),
                }
            )

    def set_stereo_renderer(self, renderer_right, renderer_left):
        self._renderer_right = renderer_right
        self._renderer_left = renderer_left

    def set_renderer(self, renderer):
        self._env.set_renderer(renderer)

    def set_recorder(self, recorder):
        self._env.set_recorder(recorder)
        self._recorder = recorder

    def close(self):
        if self._env.renderer == None:
            self._env.renderer.close()
        if self._renderer_left == None:
            self._renderer_left.close()
        if self._renderer_right == None:
            self._renderer_right.close()

    def _render(self):
        if self._env.renderer == None:
            return
        else:
            return self._env.render()

    def _get_stereo(self):
        if self._renderer_right == None or self._renderer_left == None:
            return
        else:
            return {
                "right": self._renderer_right.render(),
                "left": self._renderer_left.render(),
            }

    @property
    def cur_time(self):
        return self._env.cur_time

    @property
    def done(self):
        return self._env.done

    @property
    def success(self):
        return self._env.success


class SimRobotEnvWrapper(SimEnvWrapper):
    def __init__(self, env, setup, angle_mode, img_mode, unwrap_action_fn, unwrap_obs_fn, **kwargs) -> None:
        super().__init__(env, setup, angle_mode, img_mode, unwrap_action_fn, unwrap_obs_fn, **kwargs)
        self.robots = env.robots

        self._link_name = setup["hand"]
        self._body_name = setup["body"]
        self._head_name = setup["head"]
        robot_type = setup["robot_type"]
        env_config = setup["env_config"]

        mj_link_name = env.robots[robot_type].naming_prefix + self._link_name
        mj_body_name = env.robots[robot_type].naming_prefix + self._body_name
        mj_head_name = env.robots[robot_type].naming_prefix + self._head_name
        mj_joint_names = [env.robots[robot_type]._key_map['joint'][key] for key in setup["ik_config"].CONTROLLABLE_JOINTS]

        self.ik_solver = setup["ik_solver"](
            config=setup["ik_config"],
            sim=env.sim,
            link_name=mj_link_name,
            body_name=mj_body_name,
            joint_names=mj_joint_names,
            head_name=mj_head_name,
        )
        self._default_body_pose = setup["ik_config"].DEFAULT_BODY_POSE
        self._joint_obs_list = setup["ik_config"].JOINT_OBS_LIST


    def reset(self, sim_reset=True, **kwargs):

        if sim_reset:
            obs = self._env.reset(**kwargs)
        else:
            obs = self._env._get_obs()
        self._cur_obs = {}
        self._cur_action = copy.deepcopy(self._env._cur_action)

        self._command_buffer = {
            "pose_ee_vector": collections.deque(maxlen=2),
            "gripper": collections.deque(maxlen=2),
            "delta_pose_ee_vector": collections.deque(maxlen=2),
            "delta_positions": collections.deque(maxlen=2),
            "delta_eulers": collections.deque(maxlen=2),
        }
        pose_ee_vector_prv = np.concatenate(
            [
                obs["hand_link"]["link_pose"][self._link_name]["pos"],
                obs["hand_link"]["link_pose"][self._link_name]["quat"],
            ]
        )

        if self._recorder is not None:
            command = {
                "pos": obs["hand_link"]["link_pose"][self._link_name]["pos"],
                "quat": obs["hand_link"]["link_pose"][self._link_name]["quat"],
                "gripper": 0.0,
            }
            self._recorder._update_commands(command)

        gripper_prv = self._cur_action["right_gripper"]["joint_0"]
        self._command_buffer["gripper"].append(gripper_prv)
        self._command_buffer["pose_ee_vector"].append(pose_ee_vector_prv)

        for i in range(2):
            self._command_buffer["pose_ee_vector"].append(pose_ee_vector_prv)
            self._command_buffer["gripper"].append(gripper_prv)
            if self._angle_mode == "quat":
                self._command_buffer["delta_pose_ee_vector"].append(
                    np.array([0, 0, 0, 0, 0, 0, 1])
                )
            else:
                self._command_buffer["delta_pose_ee_vector"].append(np.zeros(6))
            self._command_buffer["delta_positions"].append(
                np.array([0, 0, 0])
            )
            self._command_buffer["delta_eulers"].append(
                np.array([0, 0, 0])
            )

        if "body_link" in obs.keys():
            self._init_body_pose = copy.deepcopy(obs["body_link"]["body_pose"])
        else:
            self._init_body_pose = None

        self._init_joint_pos = {}
        self._cur_joint_pos = {}
        for joint_obs_key in self._joint_obs_list:
            self._init_joint_pos.update(obs[joint_obs_key]["joint_pos"])

        self._init_hand_pose = copy.deepcopy(obs["hand_link"]["link_pose"][self._link_name]) if self._link_name else None
        self._init_head_pose = copy.deepcopy(obs["hand_link"]["link_pose"][self._head_name]) if self._head_name else None

        self._goal_body_pose = copy.deepcopy(self._init_body_pose)
        self._goal_head_pose = copy.deepcopy(self._init_head_pose)
        self._goal_joint_pos = copy.deepcopy(self._init_joint_pos)
        self._goal_hand_pose = copy.deepcopy(self._init_hand_pose)

        self._des_joint_pos = np.copy(self._init_joint_pos)
        self._des_body_pose = copy.deepcopy(self._goal_body_pose)

        init_time = self.cur_time
        self._policy_time = self.cur_time

        # while not self.cur_time - init_time > 3.0:
        while not self.cur_time - init_time > 5.0:

            if self.cur_time - self._policy_time > self._policy_period:
                self._policy_time += self._policy_period
                angle = (self.cur_time - init_time) * np.pi / 3.0
                smoother_value = 0.5 * (1 - np.cos(min(angle, np.pi)))

                self._goal_hand_pose["pos"] = (1 - smoother_value) * self._init_hand_pose["pos"] \
                    + smoother_value * self._default_hand_pose["pos"]
                self._goal_hand_pose["quat"] = self._default_hand_pose["quat"]

                if self._body_name:
                    self._goal_body_pose["pos"] = (1 - smoother_value) * self._init_body_pose["pos"]\
                        + smoother_value * self._default_body_pose["pos"]
                    self._goal_body_pose["quat"] = self._init_body_pose["quat"]

                if self._recorder is not None:
                    command = {
                        "pos": self._goal_hand_pose["pos"],
                        "quat": self._goal_hand_pose["quat"],
                        "gripper": 0.0,
                    }
                    self._recorder._update_commands(command)

                self._command_buffer["pose_ee_vector"].append(
                    np.concatenate((self._goal_hand_pose["pos"], self._goal_hand_pose["quat"]))
                )
                self._command_buffer["gripper"].append(gripper_prv)

            obs = self._env.step(self._get_action())

        self._policy_time = self.cur_time
        self._update_obs(obs)

        return self._get_obs()


    def _update_action(self, action, **kwargs):

        pose_ee_vector_prv = np.concatenate(
            [self._goal_hand_pose["pos"], self._goal_hand_pose["quat"]]
        )
        gripper_prv = self._cur_action["right_gripper"]["joint_0"]

        self._command_buffer["delta_pose_ee_vector"].append(action[:-1])
        self._command_buffer["pose_ee_vector"].append(pose_ee_vector_prv)
        self._command_buffer["gripper"].append(gripper_prv)
        self._command_buffer["delta_positions"].append(action[:3])
        if self._angle_mode == "quat":
            self._command_buffer["delta_eulers"].append(geom.quat_to_euler(action[3:-1]))
        else:
            self._command_buffer["delta_eulers"].append(action[3:-1])

        command_cur = self._unwrap_action(
            action, self._command_buffer, mode=self._angle_mode
            )

        self._goal_hand_pose["pos"] = command_cur["pose_ee_vector"][:3]
        self._goal_hand_pose["quat"] = command_cur["pose_ee_vector"][3:]
        self._cur_action.update({"right_gripper": {"joint_0": command_cur["gripper"]}})

        if self._recorder is not None:
            command = {
                "pos": self._goal_hand_pose["pos"],
                "quat": self._goal_hand_pose["quat"],
                "gripper": command_cur["gripper"],
            }
            self._recorder._update_commands(command)


    def _get_action(self):

        obs = self._env._get_obs()

        self._cur_body_pose = obs["body_link"]["body_pose"] if self._body_name else None
        self._cur_hand_pose = obs["hand_link"]["link_pose"][self._link_name] if self._link_name else None
        self._cur_head_pose = obs["hand_link"]["link_pose"][self._head_name] if self._head_name else None
        for joint_obs_key in self._joint_obs_list:
            self._cur_joint_pos.update(obs[joint_obs_key]["joint_pos"])

        action = self.ik_solver.solve(
            goal_joint_pos=self._goal_joint_pos,
            goal_body_pose=self._goal_body_pose,
            goal_hand_pose=self._goal_hand_pose,
            goal_head_pose=self._goal_head_pose,
            cur_joint_pos=self._cur_joint_pos,
            cur_body_pose=self._cur_body_pose,
            cur_hand_pose=self._cur_hand_pose,
            cur_head_pose=self._cur_head_pose,
            des_joint_pos=self._des_joint_pos,
            des_body_pose=self._des_body_pose,
        )

        if "mobile_joint" in action.keys():
            self._des_body_pose = {
                "pos": action["mobile_joint"]["joint_body"]["pos"],
                "quat": action["mobile_joint"]["joint_body"]["quat"],
            }
        if "quad_joint" in action.keys():
            self._des_body_pose = {
                "pos": action["quad_joint"]["body_pos"],
                "quat": action["quad_joint"]["body_quat"],
            }

        self._cur_action.update(action)

        return super()._get_action()

    def _get_obs(self):
        return copy.deepcopy(self._cur_obs)
