import numpy as np
from ...simulator.base import BaseEnvConfig
from ...simulator.mp_solver import BaseMPConfig

class PandaEnvConfig(BaseEnvConfig):

    ROBOT_CONFIG = {
        "arm": {
            "type": "Panda",
            "root": None,
        },
    }
    JOINT_MAP = {
        "Panda": {
            "joint": {
                "joint_0": "joint1",
                "joint_1": "joint2",
                "joint_2": "joint3",
                "joint_3": "joint4",
                "joint_4": "joint5",
                "joint_5": "joint6",
                "joint_6": "joint7",
            },
            "actuator": {
                "joint_0": "torq_j1",
                "joint_1": "torq_j2",
                "joint_2": "torq_j3",
                "joint_3": "torq_j4",
                "joint_4": "torq_j5",
                "joint_5": "torq_j6",
                "joint_6": "torq_j7",
            },
        },
    }

    INIT_ACTION = {
        "arm_joint": {
            "joint_0": 0,
            "joint_1": -np.pi / 4.0,
            "joint_2": 0.00,
            "joint_3": -np.pi + np.pi / 8,
            "joint_4": 0.0*np.pi / 4.0,
            "joint_5": np.pi - np.pi / 3.0,
            "joint_6": - np.pi / 4.0 + np.pi/2,
        },
    }

    CONTROL_CONFIG = {
        "arm_joint": {
            "robot_name": "arm",
            "type": "JointController",
            "init_state": {
                "joint_0": 0,
                "joint_1": -np.pi / 4.0,
                "joint_2": 0.00,
                "joint_3": -np.pi + np.pi / 8,
                "joint_4": 0.0*np.pi / 4.0,
                "joint_5": np.pi - np.pi / 3.0,
                "joint_6": - np.pi / 4.0,
            },
            "gains": {
                "joint_0": (5000, 50),
                "joint_1": (5000, 50),
                "joint_2": (2000, 50),
                "joint_3": (2000, 20),
                "joint_4": (1000, 20),
                "joint_5": (1000, 10),
                "joint_6": (1000, 10),
            },
        },
    }
    'JointController'

    OBSERVER_CONFIG = {
        "arm_joint": {
            "robot_name": "arm",
            "type": "JointObserver",
            "joint_names": [
                "joint_0",
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
        },
        "hand_link": {
            "robot_name": "arm",
            "type": "LinkObserver",
            "frame_link": None,
            "link_names": ["eef_point"],
        },
    }

class PandaMPConfig(BaseMPConfig):
    
    TIME_STEP = PandaEnvConfig.TELEOP_TIME

    ALGORITHM = "rrt_connect"

    NUM_RETRIES = 5
    MAX_RUNTIME = 500.0
    SMOOTHING = True

    ROBOT_NAME = "arm"
    ALL_JOINT_NAMES = PandaEnvConfig.JOINT_MAP[PandaEnvConfig.ROBOT_CONFIG[ROBOT_NAME]["type"]]["joint"]
    CONTROLLABLE_JOINTS = list(PandaEnvConfig.CONTROL_CONFIG["arm_joint"]["init_state"].keys())
