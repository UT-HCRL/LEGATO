from simulator.envs import EmptyEnv, LidEnv, CupEnv, LadleEnv
from simulator.envs.google import GoogleEnvConfig, GoogleIKConfig
from simulator.envs.spot import SpotEnvConfig, SpotIKConfig
from simulator.envs.panda import PandaEnvConfig, PandaIKConfig
from simulator.envs.gr1 import GR1EnvConfig, GR1IKConfig
from simulator.envs.demo import DemoEnvConfig
from simulator.envs.tool_embodiment import tool_env_config_wrapper

from simulator.ik_solver import QuadrupedalArmIKSolver, ArmIKSolver, WholeBodyIKSolver, TorsoIKSolver, BaseIKConfig


ENVS = {
    'lid': LidEnv,
    'ladle': CupEnv,
    'cup': LadleEnv,
}


SETUPS = {
    'abstract': {
        'hand': 'right_eef_point',
        'env_config': DemoEnvConfig,
        'robot_type': 'demo-bot',
        'ik_config': BaseIKConfig,
    },
    'spot': {
        'hand': 'eef_point',
        'body': 'root',
        'head': '',
        'robot_type': 'legged-arm',
        'env_config': SpotEnvConfig,
        'ik_solver': QuadrupedalArmIKSolver,
        'ik_config': SpotIKConfig,
    },
    'panda': {
        'hand': 'eef_point',
        'body': '',
        'head': '',
        'robot_type': 'arm',
        'env_config': PandaEnvConfig,
        'ik_solver': ArmIKSolver,
        'ik_config': PandaIKConfig,
    },
    'google': {
        'hand': 'eef_point',
        'body': 'root',
        'head': '',
        'robot_type': 'mobile-arm',
        'env_config': GoogleEnvConfig,
        'ik_solver': WholeBodyIKSolver,
        'ik_config': GoogleIKConfig,
    },
    'gr1': {
        'hand': 'right_eef_point',
        'body': '',
        'head': 'head_pitch',
        'robot_type': 'humanoid',
        'env_config': GR1EnvConfig,
        'ik_solver': TorsoIKSolver,
        'ik_config': GR1IKConfig,
    }
}
