import numpy as np
from .base import BaseEnvConfig
from ..ik_solver import BaseIKConfig

class GoogleEnvConfig(BaseEnvConfig):

    ROBOT_CONFIG = {
        'mobile-arm': {'type': 'Google',
                'root': None,
        },
        'right_gripper': {'type': 'Robotiq2F85',
                    'root': 'mobile-arm',
                    'eef_name': 'right-eef'
        }
    }
    JOINT_MAP = {
        "Google": {
            "joint": {
                "joint_body":  "root",
                "joint_arm_0": "joint_torso",
                "joint_arm_1": "joint_shoulder",
                "joint_arm_2": "joint_bicep",
                "joint_arm_3": "joint_elbow",
                "joint_arm_4": "joint_forearm",
                "joint_arm_5": "joint_wrist",
                "joint_arm_6": "joint_gripper",
            },
            "actuator": {
                "joint_body":  "torque_body",
                "joint_arm_0": "torque_arm_0",
                "joint_arm_1": "torque_arm_1",
                "joint_arm_2": "torque_arm_2",
                "joint_arm_3": "torque_arm_3",
                "joint_arm_4": "torque_arm_4",
                "joint_arm_5": "torque_arm_5",
                "joint_arm_6": "torque_arm_6",
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
    angle = -0.25*np.pi

    INIT_ACTION = {
        "mobile_body": {},
        "arm_joint": {
            "joint_arm_0": 0.00 * np.pi,
            "joint_arm_1": 0.125 * np.pi,
            "joint_arm_2": 0.00 * np.pi,
            "joint_arm_3": 0.625 * np.pi,
            "joint_arm_4": 0.00 * np.pi,
            "joint_arm_5": -0.25 * np.pi,
            "joint_arm_6": 0.50 * np.pi,
        },
        "mobile_joint": {
            'joint_body': {
                "pos": (-0.3, 0.1, 0.0),
                "quat": (0.0, 0.0, 0.0, 1),
            }
        },
        "right_gripper": {
            "joint_0": 0,
        },
    }

    CONTROL_CONFIG = {
        "mobile_body": {
            "robot_name": "mobile-arm",
            "type": "BodyInitializer",
            "init_state": {
                "body_pos": (-0.3, 0.1, 0.0),
                "body_quat": (0.0, 0.0, 0.0, 1.0),
            },
        },
        "mobile_joint": {
            "robot_name": "mobile-arm",
            "type": "LinkController",
            "init_state": {
                'joint_body': {
                    "pos": (-0.3, 0.1, 0.0),
                    "quat": (0.0, 0.0, 0.0, 1.0),
                }
            },
            "gains": {
                "joint_body": {
                    'pos': (1.0e3, 400),
                    'quat': (1.0e3, 400),
                },
            },
        },
        "arm_joint": {
            "robot_name": "mobile-arm",
            "type": "JointController",
            "init_state": {
                "joint_arm_0": 0.00 * np.pi,
                "joint_arm_1": 0.125 * np.pi,
                "joint_arm_2": 0.00 * np.pi,
                "joint_arm_3": 0.625 * np.pi,
                "joint_arm_4": 0.00 * np.pi,
                "joint_arm_5": -0.25 * np.pi,
                "joint_arm_6": 0.50 * np.pi,
            },
            "gains": {
                "joint_arm_0": (1000, 50),
                "joint_arm_1": (2000, 50),
                "joint_arm_2": (2000, 50),
                "joint_arm_3": (2000, 20),
                "joint_arm_4": (1000, 20),
                "joint_arm_5": (1000, 20),
                "joint_arm_6": ( 500, 20),
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

    OBSERVER_CONFIG = {
        'arm_joint': {
            'robot_name': 'mobile-arm',
            'type': 'JointObserver',
            'joint_names': [
                'joint_arm_0',
                'joint_arm_1',
                'joint_arm_2',
                'joint_arm_3',
                'joint_arm_4',
                'joint_arm_5',
                'joint_arm_6',
            ],
        },
        'body_link': {
            'robot_name': 'mobile-arm',
            'type': 'OdometryObserver',
            'frame': None,
        },
        'hand_link': {
            'robot_name': 'mobile-arm',
            'type': 'LinkObserver',
            'frame_link': None,
            'link_names': [
                'eef_point',
                'tool_point',
                'base'
            ],
        },
    }


class GoogleIKConfig(BaseIKConfig):

    ROBOT_NAME = "mobile-arm"
    CONTROLLABLE_JOINTS = list(GoogleEnvConfig.CONTROL_CONFIG["arm_joint"]["init_state"].keys())

    TIME_STEP = GoogleEnvConfig.TELEOP_TIME

    ARM_DOFS_JOINT = 7
    BASE_DOFS_VEL = 6

    POS_LIMIT_MARGIN = 1e-8

    ACC_LIMIT_SCALING_FACTOR = 1.0  # Scaling factor for acceleration limits
    VEL_LIMIT_SCALING_FACTOR = 0.2  # Scaling factor for velocity limits
    JERK_LIMIT_SCALING_FACTOR = 0.2  # Scaling factor for jerk limits

    POS_TRACKING_GAIN = 1.0e2  # Gain applied for position tracking with the goal pose
    ROT_TRACKING_GAIN = 1.0e2  # Gain applied for rotational tracking with the goal pose

    NULL_SPACE_JOINT_INDICES = [0, 1, 2, 3]
    BASE_NULLSPACE_GAIN = 5e0  # Gain applied for nullspace bias in the base
    ARM_NULLSPACE_GAIN = 0e-3  # Gain applied for nullspace bias in the joints

    MAX_Q_VEL = 200.0  # Rad/sec^2 -- used only for smoothing joint saturation behavior

    MAX_Q_ACC = 100.0  # Rad/sec^2 -- used only for smoothing joint saturation behavior

    MAX_BASE_VEL = 0.1 # Maximum base velocity
    MAX_BASE_ACC = 0.1  # Maximum base acceleration

    TORSO_POSE_HALF_RANGE = [0.3, 0.3, 1.0e-6, 1.0e-6, 1.0e-6, np.pi]

    DEFAULT_BODY_POSE = {
            "pos": np.array([-0.3, 0.1, 0.0]), # Google
            "quat": np.array([0.0, 0.0, 0.0, 1.0]),
        }

    JOINT_OBS_LIST = ["arm_joint"]
