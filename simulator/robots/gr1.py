import os

from .base import LegatoRobotModel

cwd = os.getcwd()

PATH_TO_ROBOT_MODEL = os.path.expanduser(cwd + "/models/robots/gr1")
PATH_TO_ROBOT_XML = os.path.join(PATH_TO_ROBOT_MODEL, "robot.xml")


class GR1(LegatoRobotModel):
    """
    Panda is a sensitive single-arm robot designed by Franka.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):
        super().__init__(fname=PATH_TO_ROBOT_XML, idn=idn)


    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right-eef": "right_gripper_mount",
                "right-tool": "right_tool_mount",
                "left-eef": "left_gripper_mount",
                "left-tool": "left_tool_mount"}
