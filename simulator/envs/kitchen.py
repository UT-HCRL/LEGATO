import numpy as np

from robosuite.models.arenas.table_arena import TableArena

from utils import geom
from .base import BaseEnv
from ..objects import PotObject, PotLidObject, StoveObject, LadleObject, CupObject, CabinetObject, UtensilCaseObject


class LidEnv(BaseEnv):

    def reset(self, **kwargs):
        
        direction_random = 1
        stove_pos_random = np.random.uniform([-0.31, -0.05, 0.0], [-0.33, 0.05, 0.0], size=3) + self.table_offset
        lid_pos_random = np.random.uniform(stove_pos_random+np.array([-0.07, -direction_random*0.25-0.03, 0.0]), 
                                            stove_pos_random+np.array([ -0.04, -direction_random*0.25+0.03, 0.0]), size=3)
        pot_pos_random = stove_pos_random

        self._init_object_states = {
                                    'stove': {'pos': stove_pos_random},
                                    'pot':   {'pos': pot_pos_random},
                                    'lid': {'pos': lid_pos_random, 'quat': np.array([0.707, 0.0, 0.0, 0.707])},
                                    }

        out = super().reset(**kwargs)
        return out


    def _load_model(self):
        
        super()._load_model()

        # Create an environment
        self.table_offset = np.array((1.0, 0, 0.7))
        self.table_size = np.array((1.0, 2.0, 0.05))
        self.arena = TableArena(table_full_size=self.table_size, table_offset=self.table_offset, has_legs=True)

        # initialize objects of interest
        self.objects = {
            'pot': PotObject(name="pot"),
            'lid': PotLidObject(name="lid"),
            'stove': StoveObject(name="stove"),
        }

        self.world.merge(self.arena)
        for object in self.objects.values():
            self.world.merge_assets(object)
            self.world.worldbody.append(object.get_obj())

    def _reset_objects(self):
        
        if '_init_object_states' in self.__dict__:

            for name, object in self.objects.items():
                joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
                joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
                if 'pos' in self._init_object_states[name]:
                    self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                                = np.copy(self._init_object_states[name]['pos'])
                if 'quat' in self._init_object_states[name]:
                    self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                                = np.copy(self._init_object_states[name]['quat'])


class CupEnv(BaseEnv):
    def reset(self, **kwargs):
        

        direction_random = 1
        shelf_pos_random = self.table_offset + np.random.uniform([0.15*self.table_size[0], -0.6, 0.26],
                                                                [0.25*self.table_size[0], -0.8, 0.26], size=3) # random distribution of toolbox (relative to hammer position)
        shelf_quat = geom.euler_to_quat([0.0, 0.0, np.random.uniform(0.3, 0.4)*np.pi])[[3, 0, 1, 2]]
        cup_pos_random = self.table_offset + np.random.uniform([-0.25*self.table_size[0] - 0.02, direction_random*0.1 - 0.02, 0.15], # random distribution of hammer
                                                              [-0.25*self.table_size[0] + 0.02,  direction_random*0.1 + 0.02, 0.15], size=3)
        cup_quat_random = geom.euler_to_quat([0.0, 0.0, 0.75*np.pi+np.random.uniform(-0.1*np.pi, 0.1*np.pi)])[[3, 0, 1, 2]]
        self._init_object_states = {
                                    'shelf': {'pos': shelf_pos_random,
                                              'quat': shelf_quat},
                                    'cup':   {'pos': cup_pos_random,
                                              'quat': cup_quat_random},
                                    }

        out = super().reset(**kwargs)
        return out


    def _load_model(self):
        
        super()._load_model()

        # Create an environment
        self.table_offset = np.array((0.75, 0, 0.625))
        self.table_size = np.array((0.75, 2.0, 0.05))
        self.arena = TableArena(table_full_size=self.table_size, table_offset=self.table_offset, has_legs=True)

        # initialize objects of interest
        self.objects = {
            'shelf': CabinetObject(name="cabinet"),
            'cup': CupObject(name="cup",
                            outer_cup_radius=0.08,
                            inner_cup_radius=0.07,
                            cup_height=0.08,
                            cup_ngeoms=8,
                            cup_base_height=0.01,
                            cup_base_offset=0.005,
                            add_handle=True,
                            handle_outer_radius=0.05,
                            handle_inner_radius=0.03,
                            handle_thickness=0.005,
                            handle_ngeoms=8,
                            density=100.,
                             ),
        }

        self.world.merge(self.arena)
        for object in self.objects.values():
            self.world.merge_assets(object)
            self.world.worldbody.append(object.get_obj())
        

    def _reset_objects(self):
        pass
        
        if '_init_object_states' in self.__dict__:

            for name, object in self.objects.items():
                joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
                joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
                if 'pos' in self._init_object_states[name]:
                    self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                                = np.copy(self._init_object_states[name]['pos'])
                if 'quat' in self._init_object_states[name]:
                    self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                                = np.copy(self._init_object_states[name]['quat'])


class LadleEnv(BaseEnv):


    def reset(self, **kwargs):
        
        direction_random = -1

        stove_pos_random = self.table_offset + np.random.uniform(np.array([-0.2*self.table_size[0], -direction_random*0.3-0.12, 0.01]), 
                                                                np.array([-0.2*self.table_size[0], -direction_random*0.3+0.12, 0.01]), size=3) # random distribution of toolbox (relative to hammer position)
        stove_quat_random = np.array([1.0, 0.0, 0.0, 0.0])

        pot_pos_random = stove_pos_random + np.array([0, 0, 0.01]) # random distribution of toolbox (relative to hammer position)
        pot_quat_random = np.array([1.0, 0.0, 0.0, 0.0])

        ladle_pos_random = stove_pos_random + np.random.uniform([-0.12,  -0.12, 0.2], # random distribution of hammer
                                                                [-0.12,  -0.12, 0.2], size=3)
        ladle_quat_random = geom.euler_to_quat([0.0, np.pi/4, np.random.uniform(np.pi/4, np.pi/4)])[[3, 0, 1, 2]]

        tray_pos_random = self.table_offset + np.random.uniform([-0.25*self.table_size[0], direction_random*0.4 - 0.02, 0.15], # random distribution of hammer
                                                            [-0.25*self.table_size[0],  direction_random*0.4 + 0.02, 0.15], size=3)
        tray_quat_random = geom.euler_to_quat([0.0, 0.0, 0.5*np.pi + np.random.uniform(-np.pi/8, np.pi/8)])[[3, 0, 1, 2]]


        self._init_object_states = {
                                    'stove': {'pos': stove_pos_random,
                                              'quat': stove_quat_random},
                                    'pot':   {'pos': pot_pos_random,
                                              'quat': pot_quat_random},
                                    'ladle': {'pos': ladle_pos_random,
                                              'quat': ladle_quat_random},
                                    'tray':   {'pos': tray_pos_random,
                                              'quat': tray_quat_random},
                                    }

        out = super().reset(**kwargs)
        return out

    def _load_model(self):

        super()._load_model()

        # Create an environment
        self.table_offset = np.array((0.75, 0, 0.625))
        self.table_size = np.array((0.75, 2.0, 0.05))

        self.arena = TableArena(
            table_full_size=self.table_size, table_offset=self.table_offset, has_legs=True)
        # initialize objects of interest
        self.objects = {
            'pot': PotObject(name="pot"),
            'ladle': LadleObject(name="ladle"), 
            'stove': StoveObject(name="stove"),
            'tray': UtensilCaseObject(name="tray"),
        }

        self.world.merge(self.arena)
        for object in self.objects.values():
            self.world.merge_assets(object)
            self.world.worldbody.append(object.get_obj())

    def _reset_objects(self):

        if '_init_object_states' in self.__dict__:

            for name, object in self.objects.items():
                joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
                joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
                if 'pos' in self._init_object_states[name]:
                    self.sim.data.qpos[joint_qposadr:joint_qposadr +3]\
                                = np.copy(self._init_object_states[name]['pos'])
                if 'quat' in self._init_object_states[name]:
                    self.sim.data.qpos[joint_qposadr+3:joint_qposadr +7]\
                                = np.copy(self._init_object_states[name]['quat'])
