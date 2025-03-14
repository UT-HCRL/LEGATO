import os
import git
from robosuite.models.objects import MujocoXMLObject
import xml.etree.ElementTree as ET

legato_repo = git.Repo(__file__, search_parent_directories=True)
legato_root = legato_repo.git.rev_parse("--show-toplevel")
PATH_TO_OBJECT_MODELS = os.path.join(legato_root, "models", "objects")

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


def generate_shelf_xml(width, depth, height, num_vertical_slots, num_horizontal_slots):
    outer_thickness = 0.04
    inner_thickness = 0.02

    slot_height = (height - 2 * outer_thickness - (num_vertical_slots-1) * inner_thickness) / num_vertical_slots
    slot_width = (width - 2 * outer_thickness - (num_horizontal_slots-1) * inner_thickness) / num_horizontal_slots

    mujoco = ET.Element('mujoco', model='shelf')


    asset = ET.SubElement(mujoco, 'asset')
    ET.SubElement(asset, 'texture', file='../textures/light-wood.png', type='cube', name='light-wood')
    ET.SubElement(asset, 'material', name='MatLightWood', texture='light-wood', texrepeat='3 3', specular='0.4', shininess='0.1')

    worldbody = ET.SubElement(mujoco, 'worldbody')
    body = ET.SubElement(worldbody, 'body')

    object_base = ET.SubElement(body, 'body', name='object', pos=f'0 0 0')
    object_shelf = ET.SubElement(object_base, 'body', name='shelf', pos=f'0 0 {height / 2}')
    inertial = ET.SubElement(object_base, 'inertial', pos=f'0 0 0', mass='10', diaginertia='0.01 0.01 0.01')

    # Base
    ET.SubElement(object_shelf, 'geom', type='box', size=f'{width / 2} {depth / 2} {outer_thickness / 2}', pos=f'0 0 -{height / 2 - outer_thickness / 2}', material='MatLightWood', group='0')
    
    # Top
    ET.SubElement(object_shelf, 'geom', type='box', size=f'{width / 2} {depth / 2} {outer_thickness / 2}', pos=f'0 0 {height / 2 - outer_thickness / 2}', material='MatLightWood', group='0')
    
    # Sides
    ET.SubElement(object_shelf, 'geom', type='box', size=f'{outer_thickness / 2} {depth / 2} {height / 2}', pos=f'-{width / 2 - outer_thickness / 2} 0 0', material='MatLightWood', group='0')
    ET.SubElement(object_shelf, 'geom', type='box', size=f'{outer_thickness / 2} {depth / 2} {height / 2}', pos=f'{width / 2 - outer_thickness / 2} 0 0', material='MatLightWood', group='0')
    
    # Back
    ET.SubElement(object_shelf, 'geom', type='box', size=f'{width / 2 - outer_thickness} {outer_thickness / 2} {height / 2 - outer_thickness}', pos=f'0 -{depth / 2 - outer_thickness / 2} 0', material='MatLightWood', group='0')

    slot_centers = {}
    
    # Horizontal Slots
    for i in range(1, num_vertical_slots):
        pos_z = i * (slot_height +  inner_thickness) - (height / 2 - outer_thickness) - inner_thickness / 2
        ET.SubElement(object_shelf, 'geom', type='box', 
                      size=f'{width / 2 - outer_thickness} {depth / 2 - outer_thickness / 2} {inner_thickness/2}', 
                      pos=f'0 {outer_thickness/2} {pos_z}', 
                      material='MatLightWood', group='0')

    # Middle Shelves
    for i in range(1, num_horizontal_slots):
        pos_x = i * (slot_width +  inner_thickness) - (width / 2 - outer_thickness) - inner_thickness/2
        ET.SubElement(object_shelf, 'geom', type='box', 
                      size=f'{inner_thickness/2} {depth / 2 - outer_thickness / 2} {height / 2 - outer_thickness}', 
                      pos=f'{pos_x} {outer_thickness/2} 0', 
                      material='MatLightWood', group='0')

    for i in range(num_horizontal_slots):
        for j in range(num_vertical_slots):
            pos_x = i * (slot_width +  inner_thickness) - (width / 2 - outer_thickness) + slot_width / 2
            pos_y = outer_thickness/2
            pos_z = j * (slot_height +  inner_thickness) - (height / 2 - outer_thickness) + slot_height / 2 + height / 2
            slot_id = i*num_vertical_slots + j
            slot_centers[slot_id] = (pos_x, pos_y, pos_z)

    slot_half_size = (slot_width / 2, depth / 2 - outer_thickness / 2, slot_height / 2)
    tree = ET.ElementTree(mujoco)
    ET.indent(tree, space="\t", level=0)
    return ET.tostring(mujoco, encoding='unicode', method='xml'), {'slot_centers': slot_centers, 'slot_half_size': slot_half_size}


class ShelfObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name, width, depth, height, vertical_slots, horizontal_slots):

        xml_script, slot_info = generate_shelf_xml(width, depth, height, vertical_slots, horizontal_slots)
        with open(xml_path_completion("autogen_shelf.xml", PATH_TO_OBJECT_MODELS), "w") as file:
            file.write(xml_script)

        self._slot_info = slot_info

        super().__init__(
            xml_path_completion("autogen_shelf.xml", PATH_TO_OBJECT_MODELS),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def slot_info(self):
        return self._slot_info