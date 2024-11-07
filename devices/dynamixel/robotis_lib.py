#!/usr/bin/python

import dynamixel_sdk as sdk
import time

from . import dictionary as dict
from .utils import twos_comp_forward, twos_comp_backward


##################################################################################################

# Class for Dynamixel Model X Servos

##################################################################################################

class Robotis_Servo():
    ''' Class to use a robotis XMservo.
    '''
    def __init__(self, Comms, servo_id, positive_direction = "CCW", max_torque = 0.5, max_velocity = 0.5,  actuator_name = "test_actuator", series = "XM430" , **kwargs):
        '''
        Describe this stuff
        '''
        print (f"[INFO] Initializing servo with ID {servo_id}")

        self.PortHandler = Comms.get_porthandler()
        self.PacketHandler = Comms.get_packethandler()

        positive_motor_direction = int(0 if positive_direction == "CCW" else 1)
        self.info = { "id": int(servo_id), #(int)
                      "name": actuator_name, #(str)
                      "series": series, #(str)
                      "max_torque": max_torque, #(float)
                      "max_velocity": max_velocity, #(float)
                      "direction": positive_motor_direction
                     }

        self.CONSTANTS = dict.Universal_Control_Constants

        if series == "XM430":
            self.ADDR_DICT = dict.X_Series_Address
            self.LEN_DICT = dict.X_Series_Address_Length
            self.settings = dict.XM430_Settings
            self.settings["mode"] = "position", #position, extended_position, current_position, velocity, torque

        elif series == "XM520":
            self.ADDR_DICT = dict.X_Series_Address
            self.LEN_DICT = dict.X_Series_Address_Length
            self.settings = dict.XM520_Settings
            self.settings["mode"] = "position", #position, extended_position, current_position, velocity, torque

        elif series == "XC330":
            self.ADDR_DICT = dict.X_Series_Address
            self.LEN_DICT = dict.X_Series_Address_Length
            self.settings = dict.XC330_Settings
            self.settings["mode"] = "position", #position, extended_position, current_position, velocity, torque
        else:
            raise RuntimeError('Error encountered. Motor not supported')

        try:
            #Read Servo ID to ensure you are connected
            data_read, _, error = self.PacketHandler.read1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_ID"])
            if data_read == 0: #we are not using 0 for any motor IDs
                raise RuntimeError('Ensure motors are powered on. If your motor ID is 0, change it')
        except Exception as inst:
            raise RuntimeError('Error encountered.  Could not find ID (%d) on bus.\n' %
                               ( self.info['id'] ))

        self.update_motor_direction()

        #This is just a long and confusing way to ensure that when motors start back up the encoder values
        #presented are between 0-4095 and there isn't some weird wrapping going on
        enc =self.read_present_encoder()
        if positive_motor_direction ==0:
            if enc<0:
                t = self.read_home_encoder() + int((int(enc/self.settings['max_encoder_position_mode'])+1)*self.settings['max_encoder_position_mode'])
                self.set_home_encoder(t)

            elif enc> self.settings['max_encoder_position_mode']:
                t = self.read_home_encoder() - int((int(enc/self.settings['max_encoder_position_mode']))*self.settings['max_encoder_position_mode'])
                self.set_home_encoder(t)
        else:
            if enc<0:
                t = self.read_home_encoder() - int((int(enc/self.settings['max_encoder_position_mode'])+1)*self.settings['max_encoder_position_mode'])
                self.set_home_encoder(t)

            elif enc> self.settings['max_encoder_position_mode']:
                t = self.read_home_encoder() + int((int(enc/self.settings['max_encoder_position_mode']))*self.settings['max_encoder_position_mode'])
                self.set_home_encoder(t)

        #This is generally the best mode to put all motors in upon startup
        self.set_shutdown_errors()
        self.enable_current_position_control_mode(max_torque, max_velocity)
        print (f"[INFO] Servo initialized with ID {servo_id}")

    ###############################################
    # Required Methods
    ###############################################

    '''Turns power to motors on/off as to switch operation
    '''
    def enable_torque(self):
        _, error = self.PacketHandler.write1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_TORQUE_ENABLE"], self.CONSTANTS["TORQUE_ENABLE"])
        return error

    def disable_torque(self):
        _ , error = self.PacketHandler.write1ByteTxRx(self.PortHandler,self.info['id'], self.ADDR_DICT["ADDR_TORQUE_ENABLE"], self.CONSTANTS["TORQUE_DISABLE"])
        return error

    #For more information, visit: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode10
    def update_motor_direction(self):
        direction = self.info['direction']
        self.disable_torque()
        _ , error = self.PacketHandler.write1ByteTxRx(self.PortHandler,self.info['id'], self.ADDR_DICT["ADDR_DRIVE_MODE"], direction)
        self.enable_torque()
        return error

    def read_motor_direction(self):
        data_read, _, error = self.PacketHandler.read1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_DRIVE_MODE"])
        return data_read


    def read_home_encoder(self):
        data_read, _, error = self.PacketHandler.read4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_HOMING_OFFSET"])
        enc = twos_comp_backward(data_read,32)
        return enc

    def set_home_encoder(self, enc):
        set_point = twos_comp_forward(enc,32)
        self.disable_torque()
        _ , error = self.PacketHandler.write4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_HOMING_OFFSET"], set_point)
        self.enable_torque()

    def reset_home_encoder_pos(self, offset = 0):
        ''' Resets the encoder so that the current location is now the 0 encoder
        '''
        enc = self.read_home_encoder()

        if self.info['direction'] ==0:
            set_point = enc - self.read_present_encoder() + offset
        else:
            set_point = enc + self.read_present_encoder() - offset

        self.set_home_encoder(set_point)


    def move_until_contact(self, positive_direction = False, contact_threshold = 130):
        ''' Moves the motor until contact is detected, then stops
            #TODO: This needs calibrated
        '''
        mult = 1 if positive_direction==True else -1
        setpoint = mult * 50000
        self.set_target_encoder(setpoint)
        time.sleep(0.04)
        while True:
            tor = self.read_present_torque()
            if abs(tor)> contact_threshold:
                enc = self.read_present_encoder()
                self.set_target_encoder(enc)
                return
            time.sleep(0.01)

    ###############################################
    # Setter/Getter(Readers) Methods
    ###############################################

    def set_shutdown_errors(self):
        ''' Sets which errors will allow the motor to shut down
        TODO: LET'S FIX THIS AT SOME POINT
        '''
        #CONSTANTS, DO NOT CHANGE THESE
        overload_error = 32
        electrical_shock_error = 16
        motor_encoder_error = 8
        overheating_error = 4
        input_voltage_error = 1

        #Set the error messages according to which ones you want
        error_code = overload_error + electrical_shock_error + motor_encoder_error + overheating_error + input_voltage_error
        self.disable_torque()
        _ , error = self.PacketHandler.write1ByteTxRx(self.PortHandler,self.info['id'], self.ADDR_DICT["ADDR_SHUTDOWN"], error_code)
        self.enable_torque()


    def set_max_torque(self, amnt = 0.5): #input is ratio of max motor torque to our own setpoint
        ''' Sets maximum torque the motor can emit, only applicable to torque contorl mode and current limited position control mode
        '''
        amnt = max(0.,min(abs(amnt),1.0))
        torque_val = int(amnt*self.settings["current_limit_range"])
        self.disable_torque()
        _ , error = self.PacketHandler.write2ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_CURRENT_LIMIT"], torque_val)
        return self.enable_torque()

    def set_torque(self,amnt=0.1):
        ''' Set torque works for torque control mode. Input is a ratio -1 to 1 of maximum motor torque (not the set torque)
        '''
        amnt = max(-1.,min(amnt,1.0))
        torque_val = int(amnt*self.settings["current_limit_range"])
        torque_val = twos_comp_forward(torque_val,16)
        self.disable_torque()
        _ , error = self.PacketHandler.write2ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_GOAL_CURRENT"], torque_val)
        return self.enable_torque()

    def read_present_torque(self):
        ''' rough estimate of torque
        '''
        data_read, _, error = self.PacketHandler.read2ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_PRESENT_CURRENT"])
        return twos_comp_backward(data_read,16)

    def read_present_encoder(self):
        ''' returns position in encoder ticks
        '''
        data_read, _, error = self.PacketHandler.read4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_PRESENT_POSITION"])
        return twos_comp_backward(data_read,32)

    def read_target_encoder(self):
        ''' returns target position in encoder ticks
        '''
        data_read, _, error = self.PacketHandler.read4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_GOAL_POSITION"])
        return twos_comp_backward(data_read,32)

    def set_target_encoder(self, enc):
        ''' move to encoder position enc
        '''
        if self.settings['mode'] == "position":
            enc = enc % self.settings["max_encoder"]

        enc = twos_comp_forward(enc, 32)
        _ , error = self.PacketHandler.write4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_GOAL_POSITION"], enc)

        return error

    def read_present_temperature(self):
        ''' returns the temperature (Celcius)
        '''
        data_read, _, error = self.PacketHandler.read1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_PRESENT_TEMPERATURE"])
        return twos_comp_backward(data_read,8)

    def read_goal_current(self):
        ''' returns the goal current
        '''
        data_read, _, error = self.PacketHandler.read2ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_GOAL_CURRENT"])
        return twos_comp_backward(data_read,16)

    def read_present_errors(self):
        ''' returns the errors
        '''
        data_read, _, error = self.PacketHandler.read1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_HARDWARE_ERROR_STATUS"])
        return twos_comp_backward(data_read,8)

    #set maximum velocity of the servo
    def set_max_velocity(self,amnt):
        amnt = min(abs(amnt),1.0)
        apply_speed = int(amnt*self.settings["velocity_limit_range"])
        apply_speed = twos_comp_forward(apply_speed,32)
        self.disable_torque()
        _ , error = self.PacketHandler.write4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_PROFILE_VELOCITY"], apply_speed)
        self.enable_torque()

    #read max velocity of the servo
    def read_max_velocity(self):
        data_read, _, error = self.PacketHandler.read4ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_PROFILE_VELOCITY"])
        return twos_comp_backward(data_read,32)


    ###############################################
    # Control types
    ###############################################
    #Sets position control mode of the actuator
    def enable_position_mode(self):
        self.disable_torque()
        data_read, error = self.PacketHandler.write1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_OPERATING_MODE"], self.CONSTANTS["POSITION_CONTROL_MODE"])
        self.settings['mode'] = "position"
        return self.enable_torque()

    def enable_torque_mode(self):
        self.disable_torque()
        data_read, error = self.PacketHandler.write1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_OPERATING_MODE"], self.CONSTANTS["CURRENT_CONTROL_MODE"])
        self.settings['mode'] = "torque"
        return self.enable_torque()

    def enable_extended_position_control_mode(self):
        self.disable_torque()
        data_read, error = self.PacketHandler.write1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_OPERATING_MODE"], self.CONSTANTS["EXTENDED_POSITION_CONTROL_MODE"])
        self.settings['mode'] = "extended_position"
        return self.enable_torque()

    def enable_current_position_control_mode(self, torque_val = 0.1, velocity_val = 0.1):
        self.disable_torque()
        data_read, error = self.PacketHandler.write1ByteTxRx(self.PortHandler, self.info['id'], self.ADDR_DICT["ADDR_OPERATING_MODE"], self.CONSTANTS["CURRENT_BASED_POSITION_CONTROL_MODE"])
        self.enable_torque()
        self.set_max_torque(1.0)
        self.set_max_velocity(velocity_val)
        self.set_torque(torque_val)
        self.settings['mode'] = "current_position"
