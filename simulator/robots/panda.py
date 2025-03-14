import numpy as np
import os
import git

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel

git_repo = git.Repo(__file__, search_parent_directories=True)
git_root = git_repo.git.rev_parse("--show-toplevel")

PATH_TO_ROBOT_MODEL = os.path.join(git_root, "models/robots/panda")
PATH_TO_ROBOT_XML = os.path.join(PATH_TO_ROBOT_MODEL, "robot.xml")
PATH_TO_ROBOT_FIGURE_XML = os.path.join(PATH_TO_ROBOT_MODEL, "robot_figure.xml")
PATH_TO_ROBOT_COLLIDING_XML = os.path.join(PATH_TO_ROBOT_MODEL, "robot_colliding.xml")
PATH_TO_ROBOT_COLLISON_FREE_XML = os.path.join(PATH_TO_ROBOT_MODEL, "robot_collision_free.xml")


class Panda(ManipulatorModel):
    """
    Panda is a sensitive single-arm robot designed by Franka.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0, figure=False):
        if figure:
            super().__init__(fname=PATH_TO_ROBOT_FIGURE_XML, idn=idn)
        else:
            super().__init__(fname=PATH_TO_ROBOT_XML, idn=idn)

        # Set joint damping
        # self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01)))
        self._key_map = {"joint": {}, "actuator": {}}

    @property
    def default_mount(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return "PandaGripper"

    @property
    def default_controller_config(self):
        return "default_panda"

    @property
    def init_qpos(self):
        return np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, np.pi / 4])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"

    @property
    def key_map(self):
        return self._key_map

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return "gripper_mount"

    # -------------------------------------------------------------------------------------- #
    # Public Properties: In general, these are the name-adjusted versions from the private   #
    #                    attributes pulled from their respective raw xml files               #
    # -------------------------------------------------------------------------------------- #

    @property
    def eef_name(self):
        """
        Returns:
            str or dict of str: Prefix-adjusted eef name for this robot. If bimanual robot, returns {"left", "right"}
                keyword-mapped eef names
        """
        return self.correct_naming(self._eef_name)

