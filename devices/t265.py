'''
Project:
	Data collection / Teleoprator

Title:
	T265 Driver

Description
	-read asynchronous data streaming of T265

* Copyrighted by Mingyo Seo
* Bonston Dynamics AI Institute, The University of Texas at Austin
* Created on June 7 2023
'''

# First import the library
import pyrealsense2 as rs

# Import OpenCV and numpy
import numpy as np
from multiprocessing import Array, Lock, Queue
import ctypes
import time

from .base import Sensor 

TIMEOUT = 0.5
HEIGHT=848
WIDTH=800
FPS = 10

class T265(Sensor):

    def __init__(self, img_stream = True, timeout = TIMEOUT, img_size = (HEIGHT, WIDTH)) -> None:

        self._left_buffer = Queue()
        self._right_buffer = Queue()
        self._left_lock = Lock()
        self._right_lock = Lock()
        self._pos_data = Array(ctypes.c_double, 3)
        self._rot_data = Array(ctypes.c_double, 4)
        self._angvel_data = Array(ctypes.c_double, 3)
        self._vel_data = Array(ctypes.c_double, 3)
        self._acc_data = Array(ctypes.c_double, 3)

        self._timeout = timeout
        self._img_size = img_size
        self._img_stream = img_stream

        # Build config object and stream everything
        self._cfg = rs.config()
        self._cfg.enable_stream(rs.stream.pose)
        if self._img_stream:
            self._cfg.enable_stream(rs.stream.fisheye, 1)
            self._cfg.enable_stream(rs.stream.fisheye, 2)

        super().__init__("T265")


    def _fn_init(self):

        # Wait until the first frameset is available
        while np.sum(self.rot**2) == 0:
            pass
        

    def _fn_proc(self):

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        pipe = rs.pipeline()
        pipe.start(self._cfg)
        self._time_proc.value = 0
        init_time = time.time()

        while self._flag_proc.value:

            cur_time = time.time() - init_time
            if cur_time - self._time_proc.value < 1./FPS:
                continue
            if cur_time - self._time_proc.value > self._timeout:
                break
            self._time_proc.value += 1./FPS

            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame().get_pose_data()

            if self._img_stream:

                f1 = frames.get_fisheye_frame(1).as_video_frame()
                f2 = frames.get_fisheye_frame(2).as_video_frame()

                self._left_lock.acquire()
                self._left_buffer.put(np.asanyarray(f1.get_data()))
                self._left_lock.release()

                self._right_lock.acquire()
                self._right_buffer.put(np.asanyarray(f2.get_data()))
                self._right_lock.release()

            self._pos_data[:] = (
                                pose.translation.x,
                                pose.translation.y,
                                pose.translation.z
                                )
            self._rot_data[:] = (
                                pose.rotation.x,
                                pose.rotation.y,
                                pose.rotation.z,
                                pose.rotation.w
                                )
            self._vel_data[:] = (
                                pose.velocity.x,
                                pose.velocity.y,
                                pose.velocity.z
                                )
            self._angvel_data[:] = (
                                pose.angular_velocity.x,
                                pose.angular_velocity.y,
                                pose.angular_velocity.z
                                )
            self._acc_data[:] = (
                                pose.acceleration.x,
                                pose.acceleration.y,
                                pose.acceleration.z
                                )
        pipe.stop()

    @property
    def left(self):
        if not self._left_buffer.empty():
            self._left_lock.acquire()
            self._left_img = self._left_buffer.get()
            self._left_lock.release()
        return self._left_img

    @property
    def right(self):
        if not self._right_buffer.empty():
            self._right_lock.acquire()
            self._right_img = self._right_buffer.get()
            self._right_lock.release()
        return self._right_img

    @property
    def pos(self):
        return np.array(self._pos_data)
    
    @property
    def rot(self):
        return np.array(self._rot_data)

    @property
    def vel(self):
        return np.array(self._vel_data)

    @property
    def angvel(self):
        return np.array(self._angvel_data)

    @property
    def acc(self):
        return np.array(self._acc_data)


if __name__ == "__main__":
    t265 = T265()
    t265.start()
    while True:
        pass
