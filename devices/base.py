'''
Project:
	Data collection / Teleoprator

Title:
	VN100 Driver

Description
	-read asynchronous data streaming of sensors

* Copyrighted by Mingyo Seo
* Bonston Dynamics AI Institute, The University of Texas at Austin
* Created on June 7 2023
'''


from multiprocessing import Process, Value
import ctypes

class Sensor(object):
    '''
        a receiver's configuration for remote commands
    '''
    def __init__(self, name=""):

        self._name = name

        self._flag_proc = Value(ctypes.c_bool,)
        self._time_proc = Value(ctypes.c_float,)

        print("{} created".format(self._name))


    def _fn_init(self):

        raise NotImplementedError


    def start(self):

        # Start streaming with our callback
        self._flag_proc.value = True
        self._proc = Process(target=self._fn_proc, )
        self._proc.start()

        self._fn_init()

        print("{} start!".format(self._name))


    def stop(self):

        self._flag_proc.value = False
        self._proc.terminate()

        print("{} stop!".format(self._name))


    def _fn_proc(self):

        raise NotImplementedError


    @property
    def time(self):
        return self._time_proc.value
