import numpy as np
from .base import BaseEnvConfig
from ..ik_solver import BaseIKConfig

class DemoEnvConfig(BaseEnvConfig):

    ROBOT_CONFIG = {
        'demo-bot': {'type': 'DemoBot',
                'root': None,
        },
        'right_gripper': {'type': 'Robotiq2F85',
                    'root': 'demo-bot',
                    'eef_name': 'right-eef',
        },
        'left_gripper': {'type': 'Robotiq2F85',
                    'root': 'demo-bot',
                    'eef_name': 'left-eef',
        }
    }
    JOINT_MAP = {
        'DemoBot': {
            'joint': {
                'joint_neck':       'joint_neck',
                'joint_left_hand':  'joint_left_hand',
                'joint_right_hand': 'joint_right_hand',
            },
            'actuator': {
                'joint_neck':       'torque_neck',
                'joint_left_hand':  'torque_left_hand',
                'joint_right_hand': 'torque_right_hand',
            }
        },
        'Robotiq2F85': {
            'joint': {
                'joint_0':  'joint_right_driver',
            },
            'actuator': {
                'joint_0':  'torque_drive',
            }
        }

    }
    angle = -0.25*np.pi

    INIT_ACTION = {
        'demo_link': {
            'joint_left_hand': {
                'pos': (0.0, 0.1, 0.0),
                'quat': (0.0, 0.0, 0.0, 1.0),
            },
            'joint_right_hand': {
                'pos': (0.0, -0.1, 0.0),
                'quat': (0.0, 0.0, 0.0, 1.0),
            },
            'joint_neck': {
                'pos': (0.0, 0.0, 0.0),
                'quat': (0.0, 0.0, 0.0, 1.0),
            },
        },
        'right_gripper': {
                'joint_0': 0.0,
                },
        'left_gripper': {
                'joint_0': 0.0,
                }
        }

    CONTROL_CONFIG = {
        'demo_link': {
            'robot_name': 'demo-bot',
            'type': 'LinkController',
            'init_state': {
                'joint_left_hand': {
                    'pos': (0.0,  0.1, 0.0),
                    'quat': (0, 0, 0, 1),
                },
                'joint_right_hand': {
                    'pos': (0.0, -0.1, 0.0),
                    'quat': (0, 0, 0, 1),
                },
                'joint_neck': {
                    'pos': (0.0, 0.0, 0.0),
                    'quat': (0.0, 0.0, 0.0, 1.0),
                }
            },
            'gains':{
                'joint_left_hand': {
                    'pos': (300, 1),
                    'quat': (50, 1),
                },
                'joint_right_hand': {
                    'pos': (300, 1),
                    'quat': (50, 1),
                },
                'joint_neck': {
                    'pos': (300, 1),
                    'quat': (50, 1),
                }
            }
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
 
    'hand_link': {
        'robot_name': 'demo-bot',
        'type': 'LinkObserver',
        'frame_link': None,
        'link_names': [
            'right_eef_point',
            'right_tool_point',
            'left_eef_point'
            'left_tool_point'
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
