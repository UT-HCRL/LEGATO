import copy
from collections import OrderedDict
import simulator.sim_util as sim_util

class JointObserver(object):

    def __init__(self, sim, robot, joint_names, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._joint_names = joint_names
        self.reset()

    def reset(self):
        pass    

    def standby(self):
        pass

    def get_state(self):
        state = sim_util.get_joint_state(self._sim, self._robot, self._joint_names)
        return copy.deepcopy(state)


class LinkObserver(object):

    def __init__(self, sim, robot, link_names, frame_link, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self._link_names = link_names
        self._frame_link = frame_link
        self.reset()

    def reset(self):
        pass

    def standby(self):
        pass

    def get_state(self):
        state = sim_util.get_link_state(self._sim, self._robot, self._link_names, self._frame_link)
        return copy.deepcopy(state)


class OdometryObserver(object):

    def __init__(self, sim, robot, **kwargs) -> None:
        self._sim = sim
        self._robot = robot
        self.reset()

    def reset(self):
        pass

    def standby(self):
        pass

    def get_state(self):
        state = sim_util.get_body_state(self._sim, self._robot)
        return copy.deepcopy(state)


class CompositeObserver(object):
    def __init__(self, sim, robots, obs_configs) -> None:
        self._observers = {key: 
                             eval(obs_config['type'])(sim=sim, 
                                                    robot=robots[obs_config['robot_name']], 
                                                    **obs_config) 
                             for key, obs_config in obs_configs.items()}
        self.reset()

    def reset(self):
        for _, observer in self._observers.items():
            observer.reset()

    def standby(self):
        for _, observer in self._observers.items():
            observer.standby()
    
    def get_state(self):
        state = OrderedDict()
        for key, observer in self._observers.items():
            state[key] = observer.get_state()
        return state