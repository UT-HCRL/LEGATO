import numpy as np
import xml.etree.ElementTree as ET

from robosuite.models.objects import BoxObject, CylinderObject, BallObject
from robosuite.models.arenas.table_arena import TableArena

from ..base import BaseEnv
from ..objects import *
from ..objects.presto_shelf import ShelfObject as PrestoShelfObject
from ...utils import geom


def get_vertices(center_2d_pos, rect_dim, angle):
    
    corners = np.array([
        [ 1,  1],
        [-1,  1],
        [ 1, -1],
        [-1, -1]
    ]) * rect_dim
    
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
         [np.sin(angle),  np.cos(angle)]
    ])
    
    vertices = np.dot(corners, rotation_matrix.T) + center_2d_pos
    return vertices

def is_separating_axis(a, b, axis):
    a_proj = np.dot(a, axis)
    b_proj = np.dot(b, axis)
    return np.max(a_proj) < np.min(b_proj) or np.max(b_proj) < np.min(a_proj)

def are_rectangles_conflicting(rect1, rect2):
    vertices1 = get_vertices(**rect1)
    vertices2 = get_vertices(**rect2)
    edges1 = vertices1 - np.roll(vertices1, shift=-1, axis=0)
    edges2 = vertices2 - np.roll(vertices2, shift=-1, axis=0)
    axes = np.vstack([edges1, edges2])[:, ::-1] * np.array([-1, 1])
    for axis in axes:
        if is_separating_axis(vertices1, vertices2, axis):
            return False
    return True

def sample_rotated_rectangle_in_slot(slot_2d_half_range, min_rect_dim=0.02):

    while True:

        rect_dim = np.random.uniform(min_rect_dim, slot_2d_half_range.min(), size=2)
        # Sample a random angle between 0 and 2*pi
        angle = np.random.uniform(0, 0.5 * np.pi)
                
        # Calculate bounding box of the rotated rectangle
        mat_rot = np.array([[np.cos(angle), np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        # mat_rot = np.abs(mat_rot)
        bbox_dim = mat_rot @ rect_dim
        # Ensure the bounding box fits within the slot dimensions
        if np.all(bbox_dim < slot_2d_half_range):
            # Define the boundaries within which the rectangle's center can be placed
            max_center = slot_2d_half_range - bbox_dim
            center_2d_pos = np.random.uniform(-max_center, max_center)
            
            return {
                'center_2d_pos': center_2d_pos,
                'angle': angle,
                'rect_dim': rect_dim,
            }

def mp_object_wrapper(primitive_object_class, gravcomp=True):
    class MPObject(primitive_object_class):
        def _get_object_subtree_(self, **kwargs):
            obj = super()._get_object_subtree_(**kwargs)
            # adding the gravocomp argument on the root element
            obj.set("gravcomp", "1" if gravcomp else "0")
            ## find all the geom elements in the subtree and add contype and conaffinity to be 0
            for elem in obj.iter():
                if elem.tag == 'geom':
                    elem.set("contype", "2")
                    elem.set("conaffinity", "1")
                    elem.set("solimp", "0.9 0.9 0.001")
                    elem.set("solref", "0.02 1")
                # if elem.tag == 'joint':
                #     elem.set("frictionloss", "0.005")
                #     elem.set("damping", "0.0001")
            return obj
    return MPObject

def get_table_object(table_arena):
    
    table_object = table_arena.table_body

    class TableObject(object):
        def __init__(self, table_object):
            self.table_object = table_object
            self.naming_prefix = table_object.get("name")
        def get_obj(self):
            return self.table_object
    
    return TableObject(table_object)

def get_floor_object(table_arena):
        
    floor_object = table_arena.floor

    class FloorObject(object):
        def __init__(self, floor_object):
            self.floor_object = floor_object
            self.naming_prefix = floor_object.get("name")
        def get_obj(self):
            return self.floor_object
    
    return FloorObject(floor_object)


def dict_to_mujoco_xml(name, object_states, path_to_mesh_dir):
    
    # Create the asset element
    asset = ET.Element("asset")
    # Create the worldbody element
    root_body = ET.Element("body")
    root_body.set("gravcomp", "1")
    root_body.set("pos", "0.0 0.0 0.8")
    
    for objects_type, objects_info in object_states.items():

        for obj_id, obj_info in objects_info.items():
            
            if objects_type == 'mesh':
                # continue
                # Create the mesh element in the asset section
                mesh_name = f"mesh_{obj_id}"
                file_path = path_to_mesh_dir + '/' +obj_info['file_path'].split('/')[-1]
                ET.SubElement(asset, 'mesh', name=mesh_name, file=file_path)
                                
                # Prepare the geom attributes
                geom_attributes = {
                    'mesh': mesh_name
                }

            elif objects_type == 'cuboid':
                geom_attributes = {
                    'type': 'box',
                    'size': ' '.join(map(str, [d / 2 for d in obj_info['dims']]))  # MuJoCo uses half-sizes
                }

            elif objects_type == 'cylinder':
                geom_attributes = {
                    'type': 'cylinder',
                    'size': f"{obj_info['radius']} {obj_info['height'] / 2}"  # MuJoCo uses half-height
                }

            elif objects_type == 'capsule':
                geom_attributes = {
                    'type': 'capsule',
                    'fromto': ' '.join(map(str, obj_info['base'] + obj_info['tip'])),
                    'size': str(obj_info['radius'])
                }

            else:
                continue

            geom_attributes.update({
                'name': '{}_geom_{}'.format(name, obj_id),
                'contype': '2',
                'conaffinity': '1',
            })

            # Create the body element
            # quaternion xyzw to wxyz 
            pos = obj_info['pose'][:3]
            quat = obj_info['pose'][3:]
            # quat = quat[1:] + quat[:1]
            body = ET.SubElement(root_body, 'body', name='{}_body_{}'.format(name, obj_id), pos=' '.join(map(str, pos)), quat=' '.join(map(str, quat)))

            if 'rgba' in obj_info:
                geom_attributes['rgba'] = ' '.join(map(str, obj_info['rgba']))
            else:
                geom_attributes['rgba'] = '0.5 0.5 0.5 1'

            if 'group' in obj_info:
                geom_attributes['group'] = str(obj_info['group'])
            else:
                geom_attributes['group'] = '0'

            # Create the geom element
            geom = ET.SubElement(body, 'geom', **geom_attributes)
        
    return {'body': root_body, 'asset': asset}
    

class PrestoEnv(BaseEnv):
    def __init__(self, fixture_objects=None, gravcomp=True, vertical_slots=4, horizontal_slots=3, **kwargs):

        self._init_fixture_object_states = {
                                    'shelf': {'pos': np.array([0.75, 0.0, 0.8]),
                                              'quat': geom.euler_to_quat([0, 0, 0.5*np.pi])},
                                            #   'quat': geom.euler_to_quat([0.0*np.pi, 0, 0])},
                                    }

        self._hold_fixture_object_states = {}
        self._gravcomp = gravcomp
        self._vertical_slots = vertical_slots
        self._horizontal_slots = horizontal_slots

        if fixture_objects is not None:
            self._hold_fixture_object_states.update(fixture_objects)

        super().__init__(**kwargs)


    def reset(self, mode="forward", initial_qpos=None, **kwargs):
        if mode == "forward":
            self._reset_objects()
            if initial_qpos is not None:
                self.sim.data.qpos[:] = np.copy(initial_qpos)
            self.sim.forward()
        else:
            super().reset(initial_qpos=initial_qpos, **kwargs)

    def _load_model(self):

        super()._load_model()

        # Create an environment
        self.table_offset = np.array((0.5, 0, 0.8))
        self.table_size = np.array((1.0, 2.0, 0.05))
        self.arena = TableArena(table_full_size=self.table_size, table_offset=self.table_offset, has_legs=True)

        # initialize objects of interest
        shelf_class = mp_object_wrapper(PrestoShelfObject, gravcomp=self._gravcomp)
        self.fixture_objects = {
            'shelf': shelf_class(name="shelf", width=1.5, depth=0.8, height=1.0, vertical_slots=self._vertical_slots, horizontal_slots=self._horizontal_slots),
        }

        slot_info = self.fixture_objects['shelf'].slot_info

        # print(self.slot_info)
        self.slot_range_max = {}
        self.slot_range_min = {}
        slot_width, slot_depth, slot_height = slot_info['slot_half_size']
        for slot_id, slot_centers in slot_info['slot_centers'].items():
            center_x, center_y, center_z = slot_centers
            self.slot_range_max[slot_id] = np.array([center_x + slot_width, center_y + slot_depth, center_z + slot_height])
            self.slot_range_min[slot_id] = np.array([center_x - slot_width, center_y - slot_depth, center_z - slot_height])

        self.shelf_transform = np.eye(4)
        self.shelf_transform[:3, 3] = self._init_fixture_object_states['shelf']['pos']
        self.shelf_transform[:3, :3] = geom.quat_to_rot(self._init_fixture_object_states['shelf']['quat'])

        # locate the start and the goal pose
        pose_dim = np.array([0.15, 0.1, 0.05])

        self.target_poses = {}
        for waypoint in ['start', 'goal']:
            slot_id = np.random.choice(list(self.slot_range_min.keys()))
            center_pos = np.random.uniform(self.slot_range_min[slot_id] + pose_dim, self.slot_range_max[slot_id] -pose_dim)
            center_quat = geom.euler_to_quat([0, 0, -0.5*np.pi])         
            pos_in_slot = center_pos - (self.slot_range_min[slot_id] + self.slot_range_max[slot_id])/2

            center_mat = np.eye(4)
            center_mat[:3, 3] = center_pos
            center_mat[:3, :3] = geom.quat_to_rot(center_quat)
            center_mat = np.dot(self.shelf_transform, center_mat)
            center_pos = center_mat[:3, 3]
            center_quat = geom.rot_to_quat(center_mat[:3, :3])
            pose = {'pos': center_pos, 'quat': center_quat, 'slot_id': slot_id, 
                    'pos_in_slot': pos_in_slot,
                    'slot_dim': pose_dim}
            self.target_poses[waypoint] = pose

        self.world.merge(self.arena)
        for object in self.fixture_objects.values():
            self.world.merge_assets(object)
            self.world.worldbody.append(object.get_obj())

        self.table = get_table_object(self.arena)
        self.floor = get_floor_object(self.arena)

        self.objects = {'floor': self.floor, 
                        'table': self.table,
                        **self.fixture_objects
                        }


    def _reset_objects(self):
        
        for name, object in self.fixture_objects.items():
            joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
            joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
            if 'pos' in self._init_fixture_object_states[name]:
                self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                            = np.copy(self._init_fixture_object_states[name]['pos'])
            if 'quat' in self._init_fixture_object_states[name]:
                self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                            = np.copy(self._init_fixture_object_states[name]['quat'][[3, 0, 1, 2]])

    def _hold_objects(self):

        for name, state in self._hold_fixture_object_states.items():
            object = self.fixture_objects[name]
            joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
            joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
            if 'pos' in state:
                self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                            = np.copy(state['pos'])
            if 'quat' in state:
                self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                            = np.copy(state['quat'])

        self.sim.forward()


class PrestoPrimitiveEnv(PrestoEnv):

    def __init__(self, primitive_objects=None, **kwargs) -> None:

        self._init_primitive_object_states = {
            'cuboid': {},
            'sphere': {},
            'cylinder': {},
        }

        if primitive_objects is not None:
            for obj_type in ['cuboid', 'sphere', 'cylinder']:
                for obj_name, obj_infos in primitive_objects[obj_type].items():
                    self._init_primitive_object_states[obj_type][obj_name] = {'pos': np.array(obj_infos['pose'][0:3]), 'quat': geom.euler_to_quat(obj_infos['pose'][3:6])[0], 'dims': np.array(obj_infos['dims'])}
                    if 'rgba' in obj_infos:
                        self._init_primitive_object_states[obj_type][obj_name]['rgba'] = np.array(obj_infos['rgba'])

        super().__init__(**kwargs)


    def _load_model(self):

        super()._load_model()

        self.primitive_objects = {
            'cuboid': {},
            'sphere': {},
            'cylinder': {},
        }
        # rewrite primitive objects without gravity effects
        for obj_class, obj_type in zip([BoxObject, BallObject, CylinderObject], ['cuboid', 'sphere', 'cylinder']):
            primitive_class = mp_object_wrapper(obj_class)
            self.primitive_objects[obj_type]= {
                obj_name: primitive_class(name=obj_name, size=obj_infos['dims'],
                                    duplicate_collision_geoms=True, density=1.0e-2,
                                    # rgba=np.random.uniform([0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0, 1.0]),
                                    rgba=obj_infos['rgba'] if 'rgba' in obj_infos else [0.1, 0.1, 0.1, 1.0],
                                    )
                for obj_name, obj_infos in self._init_primitive_object_states[obj_type].items()
            }

        for obj_type in ['cuboid', 'sphere', 'cylinder']:
            for object in self.primitive_objects[obj_type].values():
                self.world.merge_assets(object)
                self.world.worldbody.append(object.get_obj())
        
        self.objects.update({
                        **self.primitive_objects['cuboid'], 
                        **self.primitive_objects['sphere'], 
                        **self.primitive_objects['cylinder'],
                        })


    def _reset_objects(self):

        for obj_type in ['cuboid', 'sphere', 'cylinder']:
            for name, object in self.primitive_objects[obj_type].items():
                joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
                joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
                if 'pos' in self._init_primitive_object_states[obj_type][name]:
                    self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                                = np.copy(self._init_primitive_object_states[obj_type][name]['pos'])
                if 'quat' in self._init_primitive_object_states[obj_type][name]:
                    self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                                = np.copy(self._init_primitive_object_states[obj_type][name]['quat'][[3, 0, 1, 2]])

        super()._reset_objects()


    def _hold_objects(self):

        for obj_type in ['cuboid', 'sphere', 'cylinder']:
            for name, object in self.primitive_objects[obj_type].items():
                joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
                joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
                if 'pos' in self._init_primitive_object_states[obj_type][name]:
                    self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                                = np.copy(self._init_primitive_object_states[obj_type][name]['pos'])
                if 'quat' in self._init_primitive_object_states[obj_type][name]:
                    self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                                = np.copy(self._init_primitive_object_states[obj_type][name]['quat'][[3, 0, 1, 2]])

        super()._hold_objects()

