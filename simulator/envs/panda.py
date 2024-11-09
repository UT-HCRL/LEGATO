import numpy as np
from .base import BaseEnvConfig
from ..ik_solver import BaseIKConfig
from ..mp_solver import BaseMPConfig

class PandaEnvConfig(BaseEnvConfig):

    ROBOT_CONFIG = {
        "arm": {
            "type": "Panda",
            "root": None,
        },
        'right_gripper': {'type': 'Robotiq2F85',
                    'root': 'arm',
                    'eef_name': 'right-eef'
        }
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
        "Robotiq2F85": {
            "joint": {
                "joint_0": "joint_right_driver",
            },
            "actuator": {
                "joint_0": "torque_drive",
            },
        },
    }


    INIT_ACTION = {
        "arm_joint": {
            "joint_0": -0.125*np.pi,
            "joint_1": +0.200*np.pi,
            "joint_2": +0.125*np.pi,
            "joint_3": -0.700*np.pi,
            "joint_4": +0.750*np.pi,
            "joint_5": +0.500*np.pi,
            "joint_6": -0.666*np.pi,
        },
        "right_gripper": {
            "joint_0": 0,
        },
    }

    CONTROL_CONFIG = {
        "arm_joint": {
            "robot_name": "arm",
            "type": "JointController",
            "init_state": {
                "joint_0": -0.125*np.pi,
                "joint_1": +0.200*np.pi,
                "joint_2": -0.125*np.pi,
                "joint_3": -0.700*np.pi,
                "joint_4": +0.750*np.pi,
                "joint_5": +0.500*np.pi,
                "joint_6": -0.666*np.pi,
            },
            "gains": {
                "joint_0": (9000, 225),
                "joint_1": (9000, 225),
                "joint_2": (6000, 150),
                "joint_3": (6000, 150),
                "joint_4": (4000, 100),
                "joint_5": (4000, 100),
                "joint_6": (4000, 100),
            },
        },
        "right_gripper": {
            "robot_name": "right_gripper",
            "type": "JointController",
            "init_state": {
                "joint_0": 0,
            },
            "gains": (50, 10),
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
            "link_names": [
                "eef_point",
                "tool_point"],
            "right_gripper": {
                "robot_name": "right_gripper",
                "type": "JointObserver",
                "joint_names": ["joint_right_driver"],
            },
        },
    }


class PandaMPConfig(BaseMPConfig):
    
    ROBOT_NAME = "arm"
    CONTROLLABLE_JOINTS = list(PandaEnvConfig.CONTROL_CONFIG["arm_joint"]["init_state"].keys())

    TIME_STEP = PandaEnvConfig.TELEOP_TIME

    ALGORITHM = "rrt_connect"

    NUM_RETRIES = 5
    MAX_RUNTIME = 8.0
    SMOOTHING = True


class PandaIKConfig(BaseIKConfig):

    ROBOT_NAME = "arm"
    CONTROLLABLE_JOINTS = list(PandaEnvConfig.CONTROL_CONFIG["arm_joint"]["init_state"].keys())

    TIME_STEP = PandaEnvConfig.TELEOP_TIME

    ARM_DOFS_JOINT = 7

    POS_LIMIT_MARGIN = 1e-8

    ACC_LIMIT_SCALING_FACTOR = 1.0  # Scaling factor for acceleration limits
    VEL_LIMIT_SCALING_FACTOR = 1.0  # Scaling factor for velocity limits
    JERK_LIMIT_SCALING_FACTOR = 0.2  # Scaling factor for jerk limits

    POS_TRACKING_GAIN = 2.4e1  # Gain applied for position tracking with the goal pose
    ROT_TRACKING_GAIN = 2.4e1  # Gain applied for rotational tracking with the goal pose
    
    NULL_SPACE_JOINT_INDICES = [0, 1, 2, 3]
    ARM_NULLSPACE_GAIN = 1e3  # Gain applied for nullspace bias in the joints

    MAX_Q_ACC = 100.0  # Rad/sec^2 -- used only for smoothing joint saturation behavior
    MAX_Q_VEL = 2.5  # Rad/sec
    
    # MAX_BASE_VEL = 0.1  # Maximum base velocity
    # MAX_BASE_ACC = 0.1  # Maximum base acceleration

    DEFAULT_BODY_POSE = None

    JOINT_OBS_LIST = ["arm_joint"]