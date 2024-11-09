from .base import BaseEnv
from ..objects import *
from robosuite.models.arenas.table_arena import TableArena
from utils import geom
import numpy as np


class EtudeEnv(BaseEnv):

    def reset(self, **kwargs):
        
        self._init_object_states = {
                                    'bread': {'pos': np.array([0.3, 0.3, 1.4])},
                                    # 'bread': {'pos': np.array([0.4,  0.0, 1.0]), 'quat': geom.euler_to_quat([0.5*np.pi, 0, 0])},
                                    'tray': {'pos': np.array([0.3, 0.3, 1.2])},
                                    # 'plate': {'pos': np.array([0.3, 0.3, 1.2])},
                                    'pan': {'pos': np.array([0.4,  -0.4, 1.6]), 'quat': geom.euler_to_quat([0.5*np.pi, 0, 0])},
                                    'shelf': {'pos': np.array([0.3, -0.4, 1.6])},
                                    'toaster': {'pos': np.array([0.4,  0.0, 1.0]), 'quat': geom.euler_to_quat([0.5*np.pi, 0, 0])},
                                    # 'pot':   {'pos': pot_pos_random},
                                    # 'lid': {'pos': lid_pos_random, 'quat': np.array([0.707, 0.0, 0.0, 0.707])},
                                    }

        out = super().reset(**kwargs)
        return out


    def _load_model(self):
        
        super()._load_model()

        # Create an environment
        self.table_offset = np.array((0.5, 0, 0.8))
        self.table_size = np.array((1.0, 2.0, 0.05))
        self.arena = TableArena(table_full_size=self.table_size, table_offset=self.table_offset, has_legs=True)

        # initialize objects of interest
        self.objects = {
            'bread': BreadObject(name="bread"),
            'tray': TrayObject(name="tray"),
            'pan': PanTefalObject(name="nut"),
            'shelf': ShelfObject(name="shelf"),
            'toaster': ToasterObject(name="toaster"),
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

