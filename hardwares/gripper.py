import os, sys
import copy
import numpy as np
import time

cwd = os.getcwd()
sys.path.append(cwd)
from devices.dynamixel.robotis_lib import Robotis_Servo
from devices.dynamixel.comms_handler import CommsHandler


def pos2cnt(pos, counter_per_range = 4096, range_deg = 360):
    '''
    convert desired angular position [rad] to control command values [count]
    arguments:
            float [angular position in rad]
    returns:
            float [anglular position in count]
    '''

    val =  180 * counter_per_range * pos / (range_deg * np.pi) + counter_per_range/2
    return int(val)


def cnt2pos(cnt, counter_per_range = 4096, range_deg = 360.0):
    '''
    convert received values [count] to current angular position [rad]
    arguments:
            float [anglular position in count]
    returns:
            float [angular position in rad]
    '''

    val =  (range_deg * np.pi) * (cnt - counter_per_range/2) / (180 * counter_per_range)
    return val


class Gripper():

    def __init__ (self, params, verbose_mode=False):
        # IPython.embed()
        self._verbose_mode = verbose_mode

        self.system_params = params['system']
        self.actuator_params = params['actuator']
        self.gripper_params = params['gripper']

        #This will keep track of our objects and our states
        #You can add joint state and encoders here if you want
        self.actuators = {}

        self._init_comms()
        self._init_actuators()

        self._state_init = self.get_current_angles()

        if self._verbose_mode:
            print(f'Initial state: {self._state_init}')

        self.init_gripper()

        #TODO: @Andy Park, go on from here and do what you would like
        # IPython.embed()

    def init_gripper(self):
        # Move to the initial pose
        init_angles = {'finger_left': 45,
                       'finger_right': -45}

        if self._verbose_mode:
            print('Moving to initial pose ... ')
        self.send_desired_angles(init_angles.keys(), init_angles.values())
        time.sleep(1)

        # Initialize the arm to a known state
        self.open_gripper()
        self._is_grasped = False

    def _init_comms(self):
        self.Comms = CommsHandler(params = self.system_params)

    def _init_actuators(self):
        #Launch all actuators and apply initial settings
        for motor_name in self.gripper_params.keys(): #name of motor for each part of the hand
            motor_params = self.gripper_params[motor_name]
            id = motor_params['id']
            series = motor_params['series']
            torque_limit = motor_params['torque_limit']
            velocity_limit = motor_params['velocity_limit']
            positive_direction = motor_params['positive_direction']

            actuator = Robotis_Servo(self.Comms, servo_id = id, positive_direction = positive_direction, max_torque = torque_limit, max_velocity = velocity_limit, actuator_name = motor_name , series = series )
            if self.system_params['RECALIBRATE']:
                actuator.reset_home_encoder_pos(offset = motor_params['calibration_enc'])

            self._append_actuator(motor_name, **motor_params)

        if self._verbose_mode:
            print ('Actuators intialized ... ')

    def _append_actuator(self, motor_name, **kwargs):
        m = copy.deepcopy(kwargs)
        self.actuators[motor_name] = m


    # Reads the present torque of the actuator and updates the self.actuators dict
    def read_actuator_torque_state(self):
        ids = [self.actuators[motor_name]['id'] for motor_name in self.actuators.keys()]
        v = self.Comms.sync_read_present_torque(ids)

        idx = 0
        ret = {}
        for motor_name in self.actuators.keys():
                ret[motor_name] = v[idx]
        idx +=1
        return ret

    # Reads the current state of the actuator encoders and updates the self.actuators dict
    def read_actuator_present_encoder(self):
        ids = [self.actuators[motor_name]['id'] for motor_name in self.actuators.keys()]
        v = self.Comms.sync_read_present_encoder(ids)

        idx = 0
        ret = {}
        for motor_name in self.actuators.keys():
            ret[motor_name] = v[idx]
            idx +=1

        return ret

    def get_current_angles(self, use_degrees=True):
        enc = self.read_actuator_present_encoder()
        ret = {}
        for motor_name in self.actuators.keys():
            if use_degrees:
                ret[motor_name] = np.rad2deg(cnt2pos(enc[motor_name]))
            else:
                ret[motor_name] = cnt2pos(enc[motor_name],
                                           **self.actuator_params[self.actuators[motor_name]['series']])

        return ret

    def send_desired_angles(self, motor_names, angles, use_degrees=True):
        # motor_names is list of names according to yaml file
        # angles is list of angles in radians or degrees based on use_degrees parameter

        ids = [self.actuators[motor_name]['id'] for motor_name in motor_names]
        if use_degrees:
            target_encoders = [pos2cnt(np.deg2rad(angle), 
                                       **self.actuator_params[self.actuators[motor_name]['series']]) 
                                       for motor_name, angle in zip(motor_names, angles)]
        else:
            target_encoders = [pos2cnt(angle, 
                                       **self.actuator_params[self.actuators[motor_name]['series']]) 
                                       for motor_name, angle in zip(motor_names, angles)]
        self.Comms.sync_write_target_encoder(ids, target_encoders)

    def write_actuator_target_encoder(self, motor_names, target_encoders):
        # motor_names is list of names according to yaml file
        # target_encoders is list of encoder positions

        ids = [self.actuators[motor_name]['id'] for motor_name in motor_names]
        self.Comms.sync_write_target_encoder(ids, target_encoders)

    ### high level control functions
    def open_gripper(self):
        if self._verbose_mode:
            print ('Opening gripper ... ')
        self.send_desired_angles(['finger_left', 'finger_right'], [45.0, -45.0])

    def close_gripper(self):
        if self._verbose_mode:
            print ('Closing gripper ... ')
        self.send_desired_angles(['finger_left', 'finger_right'], [0.0, 0.0])

    def toggle_gripper(self):
        if self._is_grasped:
            self.open_gripper()
        else:
            self.close_gripper()
        self._is_grasped = not self._is_grasped
