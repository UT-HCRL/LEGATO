import numpy as np
from .base import BaseEnvConfig
from ..ik_solver import BaseIKConfig


DEFAULT_POS = (-0.1, 0.0, 0.0)
DEFAULT_HEIGHT = 0.88

class GR1EnvConfig(BaseEnvConfig):

    ROBOT_CONFIG = {
        'humanoid': {'type': 'GR1',
                'root': None,
        },
        'right_gripper': {'type': 'Robotiq2F85',
                    'root': 'humanoid',
                    'eef_name': 'right-eef',
        },
        'left_gripper': {'type': 'Robotiq2F85',
                    'root': 'humanoid',
                    'eef_name': 'left-eef',
        }    }
    JOINT_MAP = {
        "GR1": {
            "joint": {
                "joint_body":  "root",
                "l_hip_roll": "l_hip_roll",
                "l_hip_yaw": "l_hip_yaw",
                "l_hip_pitch": "l_hip_pitch",
                "l_knee_pitch": "l_knee_pitch",
                "l_ankle_pitch": "l_ankle_pitch",
                "l_ankle_roll": "l_ankle_roll",
                "r_hip_roll": "r_hip_roll",
                "r_hip_yaw": "r_hip_yaw",
                "r_hip_pitch": "r_hip_pitch",
                "r_knee_pitch": "r_knee_pitch",
                "r_ankle_pitch": "r_ankle_pitch",
                "r_ankle_roll": "r_ankle_roll",
                "waist_yaw": "waist_yaw",
                "waist_pitch": "waist_pitch",
                "waist_roll": "waist_roll",
                "l_shoulder_pitch": "l_shoulder_pitch",
                "l_shoulder_roll": "l_shoulder_roll",
                "l_shoulder_yaw": "l_shoulder_yaw",
                "l_elbow_pitch": "l_elbow_pitch",
                "l_wrist_yaw": "l_wrist_yaw",
                "l_wrist_roll": "l_wrist_roll",
                "l_wrist_pitch": "l_wrist_pitch",
                "r_shoulder_pitch": "r_shoulder_pitch",
                "r_shoulder_roll": "r_shoulder_roll",
                "r_shoulder_yaw": "r_shoulder_yaw",
                "r_elbow_pitch": "r_elbow_pitch",
                "r_wrist_yaw": "r_wrist_yaw",
                "r_wrist_roll": "r_wrist_roll",
                "r_wrist_pitch": "r_wrist_pitch",
                "head_yaw": "head_yaw",
                "head_roll": "head_roll",
                "head_pitch": "head_pitch", 
            },
            "actuator": {
                "joint_body":  "torque_body",
                "l_hip_roll": "l_hip_roll",
                "l_hip_yaw": "l_hip_yaw",
                "l_hip_pitch": "l_hip_pitch",
                "l_knee_pitch": "l_knee_pitch",
                "l_ankle_pitch": "l_ankle_pitch",
                "l_ankle_roll": "l_ankle_roll",
                "r_hip_roll": "r_hip_roll",
                "r_hip_yaw": "r_hip_yaw",
                "r_hip_pitch": "r_hip_pitch",
                "r_knee_pitch": "r_knee_pitch",
                "r_ankle_pitch": "r_ankle_pitch",
                "r_ankle_roll": "r_ankle_roll",
                "waist_yaw": "waist_yaw",
                "waist_pitch": "waist_pitch",
                "waist_roll": "waist_roll",
                "l_shoulder_pitch": "l_shoulder_pitch",
                "l_shoulder_roll": "l_shoulder_roll",
                "l_shoulder_yaw": "l_shoulder_yaw",
                "l_elbow_pitch": "l_elbow_pitch",
                "l_wrist_yaw": "l_wrist_yaw",
                "l_wrist_roll": "l_wrist_roll",
                "l_wrist_pitch": "l_wrist_pitch",
                "r_shoulder_pitch": "r_shoulder_pitch",
                "r_shoulder_roll": "r_shoulder_roll",
                "r_shoulder_yaw": "r_shoulder_yaw",
                "r_elbow_pitch": "r_elbow_pitch",
                "r_wrist_yaw": "r_wrist_yaw",
                "r_wrist_roll": "r_wrist_roll",
                "r_wrist_pitch": "r_wrist_pitch",
                "head_yaw": "head_yaw",
                "head_roll": "head_roll",
                "head_pitch": "head_pitch",
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
        "mobile_body": {},
        "right_arm_joint": {
            "r_shoulder_pitch": 0.0,
            "r_shoulder_roll": -0.16 * np.pi,
            "r_shoulder_yaw": 0.33 * np.pi,
            "r_elbow_pitch": -0.16 * np.pi,
            "r_wrist_pitch": -0.16 * np.pi,
            "r_wrist_roll": 0.0,
            "r_wrist_yaw": 0.0,
        },
        "left_arm_joint": {
            "l_shoulder_pitch": 0.0,
            "l_shoulder_roll": 0.16 * np.pi,
            "l_shoulder_yaw": -0.33 * np.pi,
            "l_elbow_pitch": -0.16 * np.pi,
            "l_wrist_pitch": -0.16 * np.pi,
            "l_wrist_roll": 0.0,
            "l_wrist_yaw": 0.0,
        },
        "right_leg_joint": {
            "r_hip_roll": 0.0,
            "r_hip_yaw": 0.0,
            "r_hip_pitch": 0.0,
            "r_knee_pitch": 0.0,
            "r_ankle_pitch": 0.0,
            "r_ankle_roll": 0.0,
        },
        "left_leg_joint": {
            "l_hip_roll": 0.0,
            "l_hip_yaw": 0.0,
            "l_hip_pitch": 0.0,
            "l_knee_pitch": 0.0,
            "l_ankle_pitch": 0.0,
            "l_ankle_roll": 0.0,
        },
        "waist_joint": {
            "waist_yaw": 0.0,
            "waist_pitch": 0.0,
            "waist_roll": 0.0,
        },
        "head_joint": {
            "head_yaw": 0.0,
            "head_pitch": 0.0,
            "head_roll": 0.0,
        },
        "mobile_joint": {
            'joint_body': {
                "pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                "quat": (0.0, 0.0, 0.0, 1),
            }
        },
        'right_gripper': {
                'joint_0': 0.0,
                },
        'left_gripper': {
                'joint_0': 0.0,
                }
    }

    CONTROL_CONFIG = {
        "mobile_body": {
            "robot_name": "humanoid",
            "type": "BodyInitializer",
            "init_state": {
                "body_pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                "body_quat": (0.0, 0.0, 0.0, 1.0),
            },
        },
        "mobile_joint": {
            "robot_name": "humanoid",
            "type": "LinkController",
            "init_state": {
                'joint_body': {
                    "pos": (DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2]+DEFAULT_HEIGHT),
                    "quat": (0.0, 0.0, 0.0, 1.0),
                }
            },
            "gains": {
                "joint_body": {
                    'pos': (2.0e5, 1000),
                    'quat': (1.0e5, 500),
                },
            },
        },
        "right_arm_joint": {
            "robot_name": "humanoid",
            "type": "JointController",
            "init_state": {
                "r_shoulder_pitch": 0.0,
                "r_shoulder_roll": -0.16 * np.pi,
                "r_shoulder_yaw": 0.33 * np.pi,
                "r_elbow_pitch": -0.16 * np.pi,
                "r_wrist_pitch": -0.16 * np.pi,
                "r_wrist_roll": 0.0,
                "r_wrist_yaw": 0.0,
            },
            "gains": {
                "r_shoulder_pitch": (1000, 50),
                "r_shoulder_roll": (1000, 50),
                "r_shoulder_yaw":(1000, 50),
                "r_elbow_pitch": (1000, 50),
                "r_wrist_pitch": (1000, 50),
                "r_wrist_roll": (1000, 50),
                "r_wrist_yaw": (1000, 50),
            },
        },
        "left_arm_joint": {
            "robot_name": "humanoid",
            "type": "JointController",
            "init_state": {
                "l_shoulder_pitch": 0.0,
                "l_shoulder_roll": 0.16 * np.pi,
                "l_shoulder_yaw": -0.33 * np.pi,
                "l_elbow_pitch": -0.16 * np.pi,
                "l_wrist_pitch": -0.16 * np.pi,
                "l_wrist_roll": 0.0,
                "l_wrist_yaw": 0.0,
            },
            "gains": {
                "l_shoulder_pitch": (1000, 50),
                "l_shoulder_roll": (1000, 50),
                "l_shoulder_yaw":(1000, 50),
                "l_elbow_pitch": (1000, 50),
                "l_wrist_pitch": (1000, 50),
                "l_wrist_roll": (1000, 50),
                "l_wrist_yaw": (1000, 50),
            },
        },
        "right_leg_joint": {
            "robot_name": "humanoid",
            "type": "JointController",
            "init_state": {
                "r_hip_roll": 0.0,
                "r_hip_yaw": 0.0,
                "r_hip_pitch": -0.08 * np.pi,
                "r_knee_pitch": 0.16 * np.pi,
                "r_ankle_pitch": -0.08 * np.pi,
                "r_ankle_roll": 0.0,
            },
            "gains": {
                "r_hip_roll": (0.0, 0.0),
                "r_hip_yaw": (0.0, 0.0),
                "r_hip_pitch": (0.0, 0.0),
                "r_knee_pitch": (0.0, 0.0),
                "r_ankle_pitch": (0.0, 0.0),
                "r_ankle_roll": (0.0, 0.0),
            },
        },
        "left_leg_joint": {
            "robot_name": "humanoid",
            "type": "JointController",
            "init_state": {
                "l_hip_roll": 0.0,
                "l_hip_yaw": 0.0,
                "l_hip_pitch": -0.08 * np.pi,
                "l_knee_pitch": 0.16 * np.pi,
                "l_ankle_pitch": -0.08 * np.pi,
                "l_ankle_roll": 0.0,
            },
            "gains": {
                "l_hip_roll": (0.0, 0.0),
                "l_hip_yaw": (0.0, 0.0),
                "l_hip_pitch": (0.0, 0.0),
                "l_knee_pitch": (0.0, 0.0),
                "l_ankle_pitch": (0.0, 0.0),
                "l_ankle_roll": (0.0, 0.0),
            },
        },
        "waist_joint": {
            "robot_name": "humanoid",
            "type": "JointController",
            "init_state": {
                "waist_yaw": 0.0,
                "waist_pitch": 0.0,
                "waist_roll": 0.0,
            },
            "gains": {
                "waist_yaw": (1000, 50),
                "waist_pitch": (1000, 50),
                "waist_roll": (1000, 50),
            },
        },
        "head_joint": {
            "robot_name": "humanoid",
            "type": "JointController",
            "init_state": {
                "head_yaw": 0.0,
                "head_pitch": 0.0,
                "head_roll": 0.0,
            },
            "gains": {
                "head_yaw": (1000, 50),
                "head_pitch": (1000, 50),
                "head_roll": (1000, 50),
            },
        },        
        'right_gripper': {
            'robot_name': 'right_gripper',
            'type': 'JointController',
            'init_state': {
                'joint_0': 0.0,
                },
            'gains': (100, 10)
        },
        'left_gripper': {
            'robot_name': 'left_gripper',
            'type': 'JointController',
            'init_state': {
                'joint_0': 0.0,
                },
            'gains': (100, 10)
        },
    }

    OBSERVER_CONFIG = {
        'right_arm_joint': {
            'robot_name': 'humanoid',
            'type': 'JointObserver',
            'joint_names': [
                "r_shoulder_pitch",
                "r_shoulder_roll",
                "r_shoulder_yaw",
                "r_elbow_pitch",
                "r_wrist_pitch",
                "r_wrist_roll",
                "r_wrist_yaw",
            ],
        },
        'left_arm_joint': {
            'robot_name': 'humanoid',
            'type': 'JointObserver',
            'joint_names': [
                "l_shoulder_pitch",
                "l_shoulder_roll",
                "l_shoulder_yaw",
                "l_elbow_pitch",
                "l_wrist_pitch",
                "l_wrist_roll",
                "l_wrist_yaw",
            ],
        },
        'waist_joint': {
            'robot_name': 'humanoid',
            'type': 'JointObserver',
            'joint_names': [
                "waist_yaw",
                "waist_pitch",
                "waist_roll",
            ],
        },
        'head_joint': {
            'robot_name': 'humanoid',
            'type': 'JointObserver',
            'joint_names': [
                "head_yaw",
                "head_pitch",
                "head_roll",
            ],
        },
        'hand_link': {
            'robot_name': 'humanoid',
            'type': 'LinkObserver',
            'frame_link': None,
            'link_names': [
                'right_eef_point',
                'right_tool_point',
                'left_eef_point',
                'left_tool_point',
                'head_pitch',
                'base'
            ],
        },
        'right_gripper': {
            'robot_name': 'right_gripper',
            'type': 'JointObserver',
            'joint_names': [
                'joint_0'
            ]
        },
        'left_gripper': {
            'robot_name': 'left_gripper',
            'type': 'JointObserver',
            'joint_names': [
                'joint_0'
            ]
        }
    }

    @property
    def all_joint_names(self):
        return self.JOINT_MAP[self.ROBOT_CONFIG["humanoid"]["type"]]["joint"]


class GR1IKConfig(BaseIKConfig):

    TIME_STEP = GR1EnvConfig.TELEOP_TIME

    CONTROLLABLE_JOINTS = list(GR1EnvConfig.CONTROL_CONFIG["right_arm_joint"]["init_state"].keys())\
        + list(GR1EnvConfig.CONTROL_CONFIG["left_arm_joint"]["init_state"].keys())\
        + list(GR1EnvConfig.CONTROL_CONFIG["waist_joint"]["init_state"].keys())\
        + list(GR1EnvConfig.CONTROL_CONFIG["head_joint"]["init_state"].keys())

    ARM_DOFS_JOINT = 20

    POS_LIMIT_MARGIN = 1e-8

    ACC_LIMIT_SCALING_FACTOR = 1.0  # Scaling factor for acceleration limits
    VEL_LIMIT_SCALING_FACTOR = 1.0  # Scaling factor for velocity limits
    JERK_LIMIT_SCALING_FACTOR = 0.2  # Scaling factor for jerk limits

    HEAD_TRACKING_GAIN = 2.0e1  # Gain applied for position tracking with the goal pose
    POS_TRACKING_GAIN = 1.0e2  # Gain applied for position tracking with the goal pose
    ROT_TRACKING_GAIN = 1.0e2  # Gain applied for rotational tracking with the goal pose

    NULL_SPACE_JOINT_INDICES = [-6, -5, -4]
    ARM_NULLSPACE_GAIN = 1e-1  # Gain applied for nullspace bias in the joints

    MAX_Q_ACC = 100.0  # Rad/sec^2 -- used only for smoothing joint saturation behavior
    MAX_Q_VEL = 2.5e1  # Rad/sec

    DEFAULT_BODY_POSE = None

    JOINT_OBS_LIST = ['right_arm_joint', 'left_arm_joint', 'waist_joint', 'head_joint']
