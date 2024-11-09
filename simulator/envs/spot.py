import numpy as np
from .base import BaseEnvConfig
from ..ik_solver import BaseIKConfig

DEFAULT_POS = (-0.2, 0.0, 0.0)
DEFAULT_HEIGHT = 0.4

class SpotEnvConfig(BaseEnvConfig):

    ROBOT_CONFIG = {
        'legged-arm': {'type': 'Spot',
                'root': None,
        },
        'right_gripper': {'type': 'Robotiq2F85',
                    'root': 'legged-arm',
                    'eef_name': 'right-eef'
        }
    }
    JOINT_MAP = {
        "Spot": {
            "joint": {
                "joint_body":  "root",
                "joint_arm_0": "joint_arm_0",
                "joint_arm_1": "joint_arm_1",
                "joint_arm_2": "joint_arm_2",
                "joint_arm_3": "joint_arm_3",
                "joint_arm_4": "joint_arm_4",
                "joint_arm_5": "joint_arm_5",
                "joint_quad_fl_0": "joint_quad_fl_0",
                "joint_quad_fl_1": "joint_quad_fl_1",
                "joint_quad_fl_2": "joint_quad_fl_2",
                "joint_quad_fr_0": "joint_quad_fr_0",
                "joint_quad_fr_1": "joint_quad_fr_1",
                "joint_quad_fr_2": "joint_quad_fr_2",
                "joint_quad_rl_0": "joint_quad_rl_0",
                "joint_quad_rl_1": "joint_quad_rl_1",
                "joint_quad_rl_2": "joint_quad_rl_2",
                "joint_quad_rr_0": "joint_quad_rr_0",
                "joint_quad_rr_1": "joint_quad_rr_1",
                "joint_quad_rr_2": "joint_quad_rr_2",
            },
            "actuator": {
                "joint_body":  "torque_body",
                "joint_arm_0": "torque_arm_0",
                "joint_arm_1": "torque_arm_1",
                "joint_arm_2": "torque_arm_2",
                "joint_arm_3": "torque_arm_3",
                "joint_arm_4": "torque_arm_4",
                "joint_arm_5": "torque_arm_5",
                "joint_quad_fl_0": "torque_quad_fl_0",
                "joint_quad_fl_1": "torque_quad_fl_1",
                "joint_quad_fl_2": "torque_quad_fl_2",
                "joint_quad_fr_0": "torque_quad_fr_0",
                "joint_quad_fr_1": "torque_quad_fr_1",
                "joint_quad_fr_2": "torque_quad_fr_2",
                "joint_quad_rl_0": "torque_quad_rl_0",
                "joint_quad_rl_1": "torque_quad_rl_1",
                "joint_quad_rl_2": "torque_quad_rl_2",
                "joint_quad_rr_0": "torque_quad_rr_0",
                "joint_quad_rr_1": "torque_quad_rr_1",
                "joint_quad_rr_2": "torque_quad_rr_2",
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
        "mobile_joint": {
            'joint_body': {
                "pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                "quat": (0.0, 0.0, 0.0, 1),
            }
        },
        "arm_joint": {
            "joint_arm_0": 0.00 * np.pi,
            "joint_arm_1": -0.66 * np.pi,
            "joint_arm_2": 0.66 * np.pi,
            "joint_arm_3": 0.00 * np.pi,
            "joint_arm_4": 0.0 * np.pi,
            "joint_arm_5": 0.00 * np.pi,
        },
        "quad_joint": {
            "body_pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
            "body_quat": (0.0, 0.0, 0.0, 1),
        },
        "right_gripper": {
            "joint_0": 0,
        },
    }

    CONTROL_CONFIG = {
        "mobile_body": {
            "robot_name": "legged-arm",
            "type": "BodyInitializer",
            "init_state": {
                "body_pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                "body_quat": (0.0, 0.0, 0.0, 1.0),
            },
        },
        "arm_joint": {
            "robot_name": "legged-arm",
            "type": "JointController",
            "init_state": {
                "joint_arm_0": 0.00 * np.pi,
                "joint_arm_1": -0.66 * np.pi,
                "joint_arm_2": 0.66 * np.pi,
                "joint_arm_3": 0.00 * np.pi,
                "joint_arm_4": 0.0 * np.pi,
                "joint_arm_5": 0.00 * np.pi,
            },
            "gains": {
                "joint_arm_0": (3000, 150),
                "joint_arm_1": (4000, 200),
                "joint_arm_2": (4000, 200),
                "joint_arm_3": (2000, 100),
                "joint_arm_4": (2000, 100),
                "joint_arm_5": (1000,  50),
            },
        },
        "mobile_joint": {
            "robot_name": "legged-arm",
            "type": "LinkController",
            "init_state": {
                'joint_body': {
                    "pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                    "quat": (0.0, 0.0, 0.0, 1.0),
                }
            },
            "gains": {
                "joint_body": {
                    'pos': (6.0e3, 1.2e3),
                    'quat': (4.0e3, 0.8e3),
                    # 'pos': (0, 0),
                    # 'quat': (0, 0),
                },
            },
        },
        "quad_joint": {
            "robot_name": "legged-arm",
            "type": "LegController",
            "foot_positions": {
                "fl": (DEFAULT_POS[0]+0.29785, DEFAULT_POS[1]+0.165945, DEFAULT_POS[2]+0.05),
                "fr": (DEFAULT_POS[0]+0.29785, DEFAULT_POS[1]-0.165945, DEFAULT_POS[2]+0.05),
                "rl": (DEFAULT_POS[0]-0.29785, DEFAULT_POS[1]+0.165945, DEFAULT_POS[2]+0.05),
                "rr": (DEFAULT_POS[0]-0.29785, DEFAULT_POS[1]-0.165945, DEFAULT_POS[2]+0.05),
            },
            "dimensions": {
                "fl": {
                    "geom_base_link_0": (0.29785, 0.055, 0.0),
                    "geom_link_0_link_1": (0.0, 0.110945, 0.0),
                    "geom_link_1_link_2": (0.025, 0.0, -0.3205),
                    "geom_link_2_link_3": (0.0, 0.0, -0.33),
                },
                "fr": {
                    "geom_base_link_0": (0.29785, -0.055, 0.0),
                    "geom_link_0_link_1": (0.0, -0.110945, 0.0),
                    "geom_link_1_link_2": (0.025, 0.0, -0.3205),
                    "geom_link_2_link_3": (0.0, 0.0, -0.33),
                },
                "rl": {
                    "geom_base_link_0": (-0.29785, 0.055, 0.0),
                    "geom_link_0_link_1": (0.0, 0.110945, 0.0),
                    "geom_link_1_link_2": (0.025, 0.0, -0.3205),
                    "geom_link_2_link_3": (0.0, 0.0, -0.33),
                },
                "rr": {
                    "geom_base_link_0": (-0.29785, -0.055, 0.0),
                    "geom_link_0_link_1": (0.0, -0.110945, 0.0),
                    "geom_link_1_link_2": (0.025, 0.0, -0.3205),
                    "geom_link_2_link_3": (0.0, 0.0, -0.33),
                },
            },
            "init_target": {
                "body_pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                "body_quat": (0.0, 0.0, 0.0, 1),
            },
            "init_state": {
                "joint_quad_fl_0": 0.00 * np.pi,
                "joint_quad_fl_1": -angle - 0.02 * np.pi,
                "joint_quad_fl_2": 2 * angle,
                "joint_quad_fr_0": 0.00 * np.pi,
                "joint_quad_fr_1": -angle - 0.02 * np.pi,
                "joint_quad_fr_2": 2 * angle,
                "joint_quad_rl_0": 0.00 * np.pi,
                "joint_quad_rl_1": -angle - 0.02 * np.pi,
                "joint_quad_rl_2": 2 * angle,
                "joint_quad_rr_0": 0.00 * np.pi,
                "joint_quad_rr_1": -angle - 0.02 * np.pi,
                "joint_quad_rr_2": 2 * angle,
            },
            "gains": {
                "joint_quad_fl_0": (200 * 1, 25 * 5),
                "joint_quad_fl_1": (200 * 2, 25 * 10),
                "joint_quad_fl_2": (200 * 2, 25 * 10),
                "joint_quad_fr_0": (200 * 1, 25 * 5),
                "joint_quad_fr_1": (200 * 2, 25 * 10),
                "joint_quad_fr_2": (200 * 2, 25 * 10),
                "joint_quad_rl_0": (200 * 1, 25 * 5),
                "joint_quad_rl_1": (200 * 2, 25 * 10),
                "joint_quad_rl_2": (200 * 2, 25 * 10),
                "joint_quad_rr_0": (200 * 1, 25 * 5),
                "joint_quad_rr_1": (200 * 2, 25 * 10),
                "joint_quad_rr_2": (200 * 2, 25 * 10),
            },
        },
        "right_gripper": {
            "robot_name": "right_gripper",
            "type": "JointController",
            "init_state": {
                "joint_0": 0,
            },
            'gains': (100, 10)
        },
    }

    OBSERVER_CONFIG = {
        'arm_joint': {
            'robot_name': 'legged-arm',
            'type': 'JointObserver',
            'joint_names': [
                'joint_arm_0',
                'joint_arm_1',
                'joint_arm_2',
                'joint_arm_3',
                'joint_arm_4',
                'joint_arm_5',
            ],
        },
        'body_link': {
            'robot_name': 'legged-arm',
            'type': 'OdometryObserver',
            'frame': None,
        },
        'hand_link': {
            'robot_name': 'legged-arm',
            'type': 'LinkObserver',
            'frame_link': None,
            'link_names': [
                'eef_point',
                'tool_point',
                'base'
            ],
        },
        'right_gripper': {
            'robot_name': 'right_gripper',
            'type': 'JointObserver',
            'joint_names': [
                'joint_0'
                ]
        }
    }


class SpotIKConfig(BaseIKConfig):

    ROBOT_NAME = "legged-arm"
    CONTROLLABLE_JOINTS = list(SpotEnvConfig.CONTROL_CONFIG["arm_joint"]["init_state"].keys())

    TIME_STEP = SpotEnvConfig.TELEOP_TIME

    ARM_DOFS_JOINT = 6
    BASE_DOFS_VEL = 6

    POS_LIMIT_MARGIN = 1e-8

    ACC_LIMIT_SCALING_FACTOR = 1.0  # Scaling factor for acceleration limits
    VEL_LIMIT_SCALING_FACTOR = 0.2  # Scaling factor for velocity limits
    JERK_LIMIT_SCALING_FACTOR = 0.2  # Scaling factor for jerk limits

    POS_TRACKING_GAIN = 1.0e2  # Gain applied for position tracking with the goal pose
    ROT_TRACKING_GAIN = 1.0e2  # Gain applied for rotational tracking with the goal pose

    NULL_SPACE_JOINT_INDICES = []
    BASE_NULLSPACE_GAIN = 5e0  # Gain applied for nullspace bias in the base
    ARM_NULLSPACE_GAIN = 0  # Gain applied for nullspace bias in the joints

    MAX_Q_VEL = 100.0  # Rad/sec^2 -- used only for smoothing joint saturation behavior
    MAX_Q_ACC = 100.0  # Rad/sec^2 -- used only for smoothing joint saturation behavior

    MAX_BASE_VEL = 1.0  # Maximum base velocity
    MAX_BASE_ACC = 1.  # Maximum base acceleration

    TORSO_POSE_HALF_RANGE = [0.1, 0.1, 0.1, np.pi / 16, np.pi / 16, np.pi / 16]

    DEFAULT_BODY_POSE = {
            "pos": np.array([-0.2, 0.0, 0.4]), 
            "quat": np.array([0.0, 0.0, 0.0, 1.0]),
        }

    JOINT_OBS_LIST = ["arm_joint"]
