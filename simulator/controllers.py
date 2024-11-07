import copy
import numpy as np
from collections import OrderedDict

from utils import geom
import simulator.sim_util as sim_util


class JointController(object):

    def __init__(self, sim, robot, init_state, robot_name, gains, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._control = {
            "joint_pos": OrderedDict(),
            "joint_vel": OrderedDict(),
            "joint_trq": OrderedDict(),
        }
        self._target = {}
        self._init_state = {
            "joint_pos": OrderedDict(init_state),
            "joint_vel": OrderedDict({key: 0.0 for key in init_state.keys()}),
            "joint_trq": OrderedDict({key: 0.0 for key in init_state.keys()}),
        }
        self._robot_name = robot_name
        self._gains = gains
        self.update_target(init_state)

    def reset(self):
        self._target = {}
        self._control.update(self._init_state)
        # self.update_target(self._init_state)

    def standby(self):
        pass

    def update_init_state(self, init_state):
        self._init_state = copy.deepcopy(init_state)

    def update_target(self, target):
        self._target.update(target)
        self._control['joint_pos'].update(self._target)

    def get_control(self):
        return copy.deepcopy(self._control)

    def get_target(self):
        return copy.deepcopy(self._target)

    def get_init_state(self):
        return copy.deepcopy(self._init_state)

    def apply_control(self):
        sim_util.set_motor_impedance(self._sim, self._robot, 
                                    self._control,
                                    self._gains)
        
    def apply_init_state(self):
        sim_util.set_motor_pos_vel(self._sim, self._robot, 
                                self._init_state)


class BodyInitializer(JointController):

    def __init__(self, sim, robot, init_state, robot_name, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._init_state = {
            "body_pose": OrderedDict({"pos": np.array(init_state["body_pos"]), "quat": np.array(init_state["body_quat"])}),
            "body_twist": OrderedDict({"pos": np.zeros(3), "rpy": np.zeros(3)}),
        }
        self._control = {}
        self._target = {}
        self._robot_name = robot_name
        self.reset()

    def reset(self):
        self._target = {}
        self._control = {}

    def update_target(self, target):
        self._target.update(target)

    def apply_control(self):
        pass

    def apply_init_state(self):
        sim_util.set_body_pos_vel(self._sim, self._robot, 
                                self._init_state)


class PlanarController(JointController):

    def __init__(self, sim, robot, init_state, robot_name, gains, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._init_state = {
            "linear": {
                "joint_pos": OrderedDict({key: np.array(val['pos']) for key, val in init_state.items()}),
                "joint_vel": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
                "joint_trq": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
            },
            "angular": {
                "joint_pos": OrderedDict({key: np.array(val['quat']) for key, val in init_state.items()}),
                "joint_vel": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
                "joint_trq": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
            },
        }
        self._target = {}
        self._control = {
            "linear": {
                "joint_pos": OrderedDict(),
                "joint_vel": OrderedDict(),
                "joint_trq": OrderedDict(),
            },
            "angular": {
                "joint_pos": OrderedDict(),
                "joint_vel": OrderedDict(),
                "joint_trq": OrderedDict(),
            },
        }
        self._robot_name = robot_name
        self._gains ={
            'linear': {key: val['pos'] for key, val in gains.items()},
            'angular': {key: val['quat'] for key, val in gains.items()}
        }
        self.reset()
        self.update_target(init_state)

    def update_target(self, target):
        for key, val in target.items():
            if key in self._target.keys():
                self._target[key].update(val)
            else:
                self._target[key] = val
        for key, val in target.items():
            self._control['linear']['joint_pos'][key] = val['pos']
            self._control['angular']['joint_pos'][key] = val['quat']

        # print(self._control['angular']['joint_pos'])
    def apply_control(self):
        sim_util.set_linear_impedance(self._sim, self._robot, 
                                    self._control['linear'],
                                    self._gains['linear'])
        sim_util.set_angular_impedance(self._sim, self._robot, 
                                    self._control['angular'],
                                    self._gains['angular'])
        
    def apply_init_state(self):
        sim_util.set_linear_pos_vel(self._sim, self._robot, 
                                    self._init_state['linear'])
        sim_util.set_angular_pos_vel(self._sim, self._robot,
                                    self._init_state['angular'])



class LinkController(JointController):

    def __init__(self, sim, robot, init_state, robot_name, gains, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._init_state = {
            "linear": {
                "joint_pos": OrderedDict({key: np.array(val['pos']) for key, val in init_state.items()}),
                "joint_vel": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
                "joint_trq": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
            },
            "angular": {
                "joint_pos": OrderedDict({key: np.array(val['quat']) for key, val in init_state.items()}),
                "joint_vel": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
                "joint_trq": OrderedDict({key: np.zeros(3) for key in init_state.keys()}),
            },
        }
        self._target = {}
        self._control = {
            "linear": {
                "joint_pos": OrderedDict(),
                "joint_vel": OrderedDict(),
                "joint_trq": OrderedDict(),
            },
            "angular": {
                "joint_pos": OrderedDict(),
                "joint_vel": OrderedDict(),
                "joint_trq": OrderedDict(),
            },
        }
        self._robot_name = robot_name
        self._gains ={
            'linear': {key: val['pos'] for key, val in gains.items()},
            'angular': {key: val['quat'] for key, val in gains.items()}
        }
        self.reset()
        self.update_target(init_state)

    def update_target(self, target):
        for key, val in target.items():
            if key in self._target.keys():
                self._target[key].update(val)
            else:
                self._target[key] = val
        for key, val in target.items():
            self._control['linear']['joint_pos'][key] = val['pos']
            self._control['angular']['joint_pos'][key] = val['quat']

        # print(self._control['angular']['joint_pos'])
    def apply_control(self):
        sim_util.set_linear_impedance(self._sim, self._robot, 
                                    self._control['linear'],
                                    self._gains['linear'])
        sim_util.set_angular_impedance(self._sim, self._robot, 
                                    self._control['angular'],
                                    self._gains['angular'])
        
    def apply_init_state(self):
        sim_util.set_linear_pos_vel(self._sim, self._robot, 
                                    self._init_state['linear'])
        sim_util.set_angular_pos_vel(self._sim, self._robot,
                                    self._init_state['angular'])


class LegController(JointController):

    def __init__(self, sim, robot, init_state, robot_name, dimensions, foot_positions, init_target, gains, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._control = {
            "joint_pos": OrderedDict(),
            "joint_vel": OrderedDict(),
            "joint_trq": OrderedDict(),
        }
        self._target = {
            "body_pos": OrderedDict(),
            "body_quat": OrderedDict(),
        }
        self._init_state = {
            "joint_pos": OrderedDict(init_state),
            "joint_vel": OrderedDict({key: 0.0 for key in init_state.keys()}),
            "joint_trq": OrderedDict({key: 0.0 for key in init_state.keys()}),
        }
        self._init_target = copy.deepcopy(init_target)
        self._foot_positions = copy.deepcopy(foot_positions)
        self._dimensions = copy.deepcopy(dimensions)
        self._robot_name = robot_name
        self._gains = gains
        self.reset()

    def reset(self):
        self._control.update(self._init_state)
        self.update_target(self._init_target)

    def update_target(self, target):
        self._target.update(target)
        body_pos = self._target["body_pos"]
        body_quat = self._target["body_quat"]

        leg_keys = ['fl', 'fr', 'rl', 'rr']

        foot_global_pos = np.array([self._foot_positions[key] for key in leg_keys])
        foot_pos = (foot_global_pos - np.repeat(np.array(body_pos).reshape(1,3), 4, axis=0)) @ geom.quat_to_rot(body_quat)
        shoulder_pos = np.array([self._dimensions[key]["geom_base_link_0"] for key in leg_keys])

        geom_l0_l1 = np.array([self._dimensions[key]["geom_link_0_link_1"] for key in leg_keys])
        geom_l1_l2 = np.array([self._dimensions[key]["geom_link_1_link_2"] for key in leg_keys])
        geom_l2_l3 = np.array([self._dimensions[key]["geom_link_2_link_3"] for key in leg_keys])
        
        geom_shoulder_foot = foot_pos - shoulder_pos
        val_tmp1 = geom_l0_l1[:,1] + geom_l1_l2[:,1] + geom_l2_l3[:,1]
        val_tmp2 = np.sqrt(geom_shoulder_foot[:,1]**2 + geom_shoulder_foot[:,2]**2 - val_tmp1**2)
        val_joint1 = -np.arctan(geom_shoulder_foot[:,1]/geom_shoulder_foot[:,2]) - np.arctan2(val_tmp1,val_tmp2)

        geom_thigh_foot = geom_shoulder_foot
        for i in range(4):
            geom_thigh_foot[i] -= geom.euler_to_rot(np.array([val_joint1[i], 0, 0])) @ geom_l0_l1[i].T        

        val_tmp3 = np.linalg.norm(geom_l1_l2[:,1:], axis=1)
        val_tmp4 = np.linalg.norm(geom_l2_l3[:,1:], axis=1)
        val_tmp5 = np.linalg.norm(geom_thigh_foot[:,1:], axis=1)

        val_tmp6 = - np.arctan2(geom_l1_l2[:,2]/np.cos(val_joint1) , geom_l1_l2[:,0]) #alpha
        val_tmp7 = - np.arctan2(geom_l2_l3[:,2]/np.cos(val_joint1) , geom_l2_l3[:,0]) #beta
        val_tmp8 = - np.arctan2(geom_thigh_foot[:,2]/ np.cos(val_joint1) , geom_thigh_foot[:,0]) #gamma

        val_tmp9 = (val_tmp3**2 + val_tmp5**2 - val_tmp4**2)/(2.0 * val_tmp3 * val_tmp5)
        val_tmp10 = (val_tmp3**2 + val_tmp4**2 - val_tmp5**2)/(2.0 * val_tmp3 * val_tmp4)

        val_tmp11 = np.arccos(val_tmp9)
        val_tmp12 = np.arccos(val_tmp10)

        val_joint2 = - val_tmp6 + val_tmp8 + val_tmp11 
        val_joint3 = - np.pi + val_tmp6 - val_tmp7 + val_tmp12

        joint_control = np.array([val_joint1, val_joint2, val_joint3]).T

        for i, key in enumerate(leg_keys):
            for j in range(3):
                self._control["joint_pos"]["joint_quad_{}_{}".format(
                    key, j)] = joint_control[i][j]


class CompositeController(object):
    def __init__(self, sim, robots, ctrl_configs) -> None:
        self._controllers = {key: 
                             eval(ctrl_config['type'])(sim=sim, 
                                                       robot=robots[ctrl_config['robot_name']], 
                                                       **ctrl_config) 
                             for key, ctrl_config in ctrl_configs.items()}
        self.reset()

    def reset(self):
        for _, controller in self._controllers.items():
            controller.reset()

    def standby(self):
        for _, controller in self._controllers.items():
            controller.standby()

    def update_init_state(self, init_state):
        for key, val in init_state.items():
            self._controllers[key].update_init_state(val)

    def update_target(self, target):
        for key, val in target.items():
            self._controllers[key].update_target(val)

    def get_control(self):
        control = OrderedDict()
        for key, controller in self._controllers.items():
            control[key] = controller.get_control()
        return control

    def get_target(self):
        target = OrderedDict()
        for key, controller in self._controllers.items():
            target[key] = controller.get_target()
        return target
    
    def get_init_state(self):
        init_state = OrderedDict()
        for key, controller in self._controllers.items():
            init_state[key] = controller.get_init_state()
        return init_state
    
    def apply_control(self):
        for _, controller in self._controllers.items():
            controller.apply_control()

    def apply_init_state(self):
        for _, controller in self._controllers.items():
            controller.apply_init_state()