import copy
import numpy as np

from robosuite.models.base import MujocoXML
from robosuite.utils.binding_utils import MjSim

from simulator.controllers import CompositeController
from simulator.observer import CompositeObserver
from simulator.robots import *
from simulator.grippers import *


def set_joint_key_map(robot, key_map):
    for key in key_map['joint'].keys():
        robot._key_map['joint'].update({key: robot.naming_prefix+key_map['joint'][key]})
        if key in key_map['actuator'].keys():
            robot._key_map['actuator'].update({key: robot.naming_prefix+key_map['actuator'][key]})

def get_mujoco_joint_names(env_config, robot_name, joint_names):
    return [env_config.JOINT_MAP[env_config.ROBOT_CONFIG[robot_name]["type"]]["joint"][joint_name] for joint_name in joint_names]


class BaseEnvConfig:

    INIT_TIME = 1.0
    SIM_TIME = 0.002
    CONTROL_TIME = 0.002
    TELEOP_TIME = 0.01
    POLICY_TIME = 0.1
    RENDER_TIME = 0.03
    JOINT_MAP = None
    PATH_TO_WORLD_XML = "./models/base.xml"
    PATH_TO_ROBOT_MODEL = None
    INIT_ACTION = None
    CONTROL_CONFIG = None
    OBSERVER_CONFIG = None
    ROBOT_CONFIG = None

    @classmethod
    def to_dict(cls):
        return {attr: getattr(cls, attr) for attr in cls.__dict__ if not callable(getattr(cls, attr)) and not attr.startswith("__")}

    @classmethod
    def from_dict(cls, dict_data):
        for key, value in dict_data.items():
            setattr(cls, key, value)


class BaseEnv:
    def __init__(self, env_config=BaseEnvConfig) -> None:
        self._env_config = env_config

        self._load_model()
        for key, robot in self.robots.items():
            robot_type = self._env_config.ROBOT_CONFIG[key]['type']
            joint_map = self._env_config.JOINT_MAP[robot_type]
            set_joint_key_map(robot, joint_map)

        model = self.world.get_model(mode="mujoco")

        self.sim = MjSim(model)

        # Renderer
        self.renderer = None
        self.recorder = None
        self.set_controller(CompositeController(self.sim, self.robots, self._env_config.CONTROL_CONFIG))
        self.set_observer(CompositeObserver(self.sim, self.robots, self._env_config.OBSERVER_CONFIG))


    def reset(self, initial_pos=None, initial_qpos=None, **kwargs):
        self.sim.reset()
        self._reset_robot(initial_pos)
        self._reset_objects()
        self._reset_recorder()
        if initial_qpos is not None:
            self._reset_initial_qpos(initial_qpos)

        self.sim.forward()

        self._cur_sim_time = 0.0
        self._cur_control_time = 0.0
        self._cur_render_time = 0.0
        self._cur_teleop_time = 0.0

        self._cur_action = copy.deepcopy(self._env_config.INIT_ACTION)
        self._cur_obs = {}

        while self._cur_sim_time < self._env_config.INIT_TIME:
            self._apply_control()
            self.sim.step()
            self._cur_sim_time += self._env_config.SIM_TIME
            if self._cur_sim_time - self._cur_render_time >= self._env_config.RENDER_TIME:
                self._render()
                self._cur_render_time += self._env_config.RENDER_TIME

        self._update_obs()

        return self._get_obs()


    def step(self, action):

        for key in action.keys():
            self._cur_action[key].update(action[key])

        prv_obs = self._get_obs()

        cur_cmd = self._get_cmd()
        cur_action = self._get_action()
        self._record(
            time_stamp=self._cur_teleop_time,
            label="demo",
            data={"action": cur_action, "observation": prv_obs},
        )

        self.controller.update_target(cur_cmd)

        while self._cur_sim_time - self._cur_teleop_time < self._env_config.TELEOP_TIME:
            self._apply_control()
            self.sim.step()
            self._record(time_stamp=self._cur_sim_time)

            if self._cur_sim_time - self._cur_render_time >= self._env_config.RENDER_TIME:
                self._render()
                self._cur_render_time += self._env_config.RENDER_TIME

            self._cur_sim_time += self._env_config.SIM_TIME

        self._cur_teleop_time += self._env_config.TELEOP_TIME
        self._update_obs()

        return self._get_obs()

    @property
    def cur_time(self):
        return self._cur_sim_time

    @property
    def done(self):
        return False

    def render(self):
        return self._render()

    def set_renderer(self, renderer):
        self.renderer = renderer

    def set_recorder(self, recorder):
        self.recorder = recorder

    def set_controller(self, controller):
        self.controller = controller

    def set_col_checker(self, col_checker):
        self.col_checker = col_checker

    def set_observer(self, observer):
        self.observer = observer

    def _update_obs(self):
        self._cur_obs.update(self.observer.get_state())

    def _get_obs(self):
        return self._cur_obs

    def _get_action(self):
        return self._cur_action

    def _get_cmd(self):
        return copy.deepcopy(self._cur_action)

    def _load_model(self):
        self.world = MujocoXML(self._env_config.PATH_TO_WORLD_XML)
        self.robots = {}
        for key in self._env_config.ROBOT_CONFIG.keys():
            robot_type = self._env_config.ROBOT_CONFIG[key]['type']
            self.robots[key] = eval(robot_type)(idn='_'+key)
        for key in self.robots.keys():
            if self._env_config.ROBOT_CONFIG[key]['root'] is not None:
                root_robot = self.robots[self._env_config.ROBOT_CONFIG[key]['root']]
                child_robot = self.robots[key]
                if type(root_robot._eef_name) is str:
                    eef_link = root_robot._eef_name
                else:
                    eef_link = root_robot._eef_name[self._env_config.ROBOT_CONFIG[key]['eef_name']]
                root_robot.add_gripper(
                    child_robot,
                    arm_name=root_robot.naming_prefix + eef_link,
                )
        for key in self.robots.keys():
            if self._env_config.ROBOT_CONFIG[key]['root'] is None:
                self.world.merge(self.robots[key])
                self.world.merge_assets(self.robots[key])    

    def _reset_robot(self, init_state=None):
        if init_state is not None:
            self.controller.update_init_state(init_state)
        self.controller.reset()
        self.observer.reset()
        self.controller.apply_init_state()

    def _reset_objects(self):
        pass

    def _render(self):
        if self.renderer == None:
            return
        else:
            return self.renderer.render()

    def _record(self, **kwargs):
        if self.recorder == None:
            return
        else:
            return self.recorder.record(**kwargs)

    def _reset_recorder(self, **kwargs):
        if self.recorder == None:
            return
        else:
            return self.recorder.reset(**kwargs)

    def _reset_initial_qpos(self, initial_qpos):
        self.sim.data.qpos[:] = np.copy(initial_qpos)

    def _apply_control(self):

        if self._cur_sim_time - self._cur_control_time >= self._env_config.CONTROL_TIME:
            self.controller.apply_control()
            self._cur_control_time += self._env_config.CONTROL_TIME

    def _check_success(self):
        raise NotImplementedError

    @property
    def body_pos(self):
        return self._get_body_pos()

    @property
    def body_quat(self):
        return self._get_body_quat()

    @property
    def hand_pos(self):
        return self._get_hand_pos()

    @property
    def hand_quat(self):
        return self._get_hand_quat()

    @property
    def success(self):
        self._check_success()
        return self._success

    @property
    def config(self):
        return self._env_config
