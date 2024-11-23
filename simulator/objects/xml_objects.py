import os

import numpy as np
from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import (
    # xml_path_completion,
    find_elements,
    array_to_string,
)

cwd = os.getcwd()
PATH_TO_OBJECT_MODELS = os.path.expanduser(cwd + "/models/objects")

def xml_path_completion(xml_path, root=None):
    """
    Takes in a local xml path and returns a full path.
        if @xml_path is absolute, do nothing
        if @xml_path is not absolute, load xml that is shipped by the package

    Args:
        xml_path (str): local xml path
        root (str): root location of assets (only applicable if path is not absolute)

    Returns:
        str: Full (absolute) xml path
    """
    if xml_path.startswith("/"):
        full_path = xml_path
    else:
        if root is None:
            root = PATH_TO_OBJECT_MODELS
        full_path = os.path.join(root, xml_path)
    return full_path


class PlateObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("dinnerware/plate.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

class TableObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("table.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class PotObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("pot.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class PotLidObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("pot_lid.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class UtensilCaseObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("utensil_case.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class LadleObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("ladle.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class StoveObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("stove.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class ShelfObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("shelf.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of door handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle"})
        return dic


class CabinetObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("cabinet.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of door handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle"})
        return dic


class NailObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("nail.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of door handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle"})
        return dic


class PegboardObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("pegboard.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[
                dict(
                    name="joint_x",
                    type="slide",
                    axis="1 0 0",
                    frictionloss="100",
                    damping="0.1",
                ),
                dict(
                    name="joint_y",
                    type="slide",
                    axis="0 1 0",
                    frictionloss="100",
                    damping="0.1",
                ),
                dict(
                    name="joint_yaw",
                    type="hinge",
                    axis="0 0 1",
                    frictionloss="100",
                    damping="0.1",
                ),
            ],
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class CartObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("cart.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[
                dict(
                    name="joint_x",
                    type="slide",
                    axis="1 0 0",
                    frictionloss="0.5",
                    damping="320.",
                ),
                dict(
                    name="joint_y",
                    type="slide",
                    axis="0 1 0",
                    frictionloss="0.5",
                    damping="320.",
                ),
                dict(
                    name="joint_yaw",
                    type="hinge",
                    axis="0 0 1",
                    frictionloss="0.5",
                    damping="640.",
                ),
            ],
            obj_type="all",
            duplicate_collision_geoms=False,
        )
