import numpy as np

def tool_env_config_wrapper(env_config, gripper_name):

    env_config.JOINT_MAP.update({"LegatoGripper": {
                "joint": {
                    "joint_l": "joint_left_driver",
                    "joint_r": "joint_right_driver",
                },
                "actuator": {
                    "joint_l": "torque_left_drive",
                    "joint_r": "torque_right_drive",
                },
            }
        })

    if gripper_name in env_config.ROBOT_CONFIG.keys():

        env_config.ROBOT_CONFIG[gripper_name].update({'type': 'LegatoGripper',
                                                'eef_name': env_config.ROBOT_CONFIG[gripper_name]['eef_name'].replace("eef", "tool")
                                                })

        env_config.INIT_ACTION[gripper_name].update({
                        "joint_l": 0.25*np.pi,
                        "joint_r": -0.25*np.pi,
                })

        env_config.CONTROL_CONFIG[gripper_name] = {
                    "robot_name": gripper_name,
                    "type": "JointController",
                    "init_state": {
                        "joint_l": 0.25*np.pi,
                        "joint_r": -0.25*np.pi,
                    },
                    'gains': (100, 0.25),
                    'vel_max': 5.0,
                    'pos_max': 0.25*np.pi,
                    'pos_min': 0.0,
                }

        env_config.OBSERVER_CONFIG[gripper_name] = {
                    'robot_name': gripper_name,
                    'type': 'JointObserver',
                    'joint_names': [
                        'joint_l',
                        'joint_r'
                        ]
                }

    return env_config