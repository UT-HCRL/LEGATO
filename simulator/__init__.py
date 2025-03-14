from .envs.panda import PandaEnvConfig, PandaMPConfig
from .mp_solver import ArmMPSolver

SETUPS = {
    'panda': {
        'hand': 'right_eef_point',
        'body': 'root',
        'head': '',
        'robot_type': 'arm',
        'env_config': PandaEnvConfig,
        'mp_solver': ArmMPSolver,
        'mp_config': PandaMPConfig,
    },
}
