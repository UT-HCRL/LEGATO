import time
import zmq
import pickle
import copy
import ctypes
import numpy as np
from multiprocessing import Process, Value, Manager


INIT_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


class ZMQServer(object):

    def __init__(self, ip, sub_port, pub_port, logger_fn=print, verbose=False):

        self.ip = ip
        self.pub_port = pub_port
        self.sub_port = sub_port
        self._running = Value(ctypes.c_bool,)
        self._init_time = 0.0
        self._send_proc = None
        self._recv_proc = None
        self._io_manager = Manager()
        self._cmd = self._io_manager.dict()
        self._obs = self._io_manager.dict()
        self._verbose = verbose
        self._logger_fn = logger_fn
        self._logger_fn("TESTING SPOT LOGGING...")

    def _publish(self):

        context = zmq.Context()
        pub_socket = context.socket(zmq.PUB)
        pub_socket.bind("tcp://{}:{}".format(self.ip, self.pub_port))
        self._logger_fn("Publishing Socket is running...")

        while self._running.value:

            if not self._cmd:
                continue
            cmd = {key: value for key, value in self._cmd.items()}
            msg = pickle.dumps(cmd)
            pub_socket.send(msg)
            if self._verbose:
                self._logger_fn(
                    "[{:.2f}] Publishing message...".format(
                        time.time() - self._init_time
                    )
                )
                for key, val in self._cmd.items():
                    self._logger_fn("{}: {}".format(key, val))
            self._cmd.clear()

        pub_socket.close()
        self._logger_fn("Publishing Socket is closed...")

    def _subscribe(self):

        context = zmq.Context()
        sub_socket = context.socket(zmq.SUB)
        sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
        sub_socket.setsockopt(zmq.RCVBUF, 0)
        sub_socket.bind("tcp://{}:{}".format(self.ip, self.sub_port))
        self._logger_fn("Subscription Socket is running...")

        while self._running.value:
            data = sub_socket.recv()
            obs = pickle.loads(data)
            self._obs.update(obs)
            if self._verbose:
                self._logger_fn(
                    "[{:.2f}] Recieved message...".format(time.time() - self._init_time)
                )
                for key, val in self._obs.items():
                    self._logger_fn("{}: {}".format(key, val))

        sub_socket.close()
        self._logger_fn("Subscription Socket is closed...")

    def start(self):

        assert self._send_proc is None and self._recv_proc is None

        self._cmd.clear()
        self._obs.clear()

        self._running.value = True
        self._init_time = time.time()

        self._send_proc = Process(target=self._publish,)
        self._recv_proc = Process(target=self._subscribe,)

        self._send_proc.start()
        self._recv_proc.start()

    def stop(self):
        self._running.value = False
        self._send_proc.terminate()
        self._recv_proc.terminate()
        del self._send_proc
        del self._recv_proc
        self._send_proc = None
        self._recv_proc = None

    def update(self, cmd):
        self._cmd.update(cmd)

    @property
    def obs(self):
        return copy.deepcopy(self._obs)


class ZMQClient(object):
    def __init__(self, ip, sub_port, pub_port, logger_fn=print):

        self.ip = ip
        self.pub_port = pub_port
        self.sub_port = sub_port

        self._running = Value(ctypes.c_bool,)
        self._init_time = 0.0
        self._send_proc = None
        self._recv_proc = None
        self._io_manager = Manager()
        self._cmd = self._io_manager.dict()
        self._obs = self._io_manager.dict()

        self._logger_fn = logger_fn
        self._logger_fn("TESTING SPOT LOGGING...")

    def _publish(self):

        context = zmq.Context()
        pub_socket = context.socket(zmq.PUB)
        pub_socket.connect("tcp://{}:{}".format(self.ip, self.pub_port))
        self._logger_fn("Publishing Socket is running...")

        while self._running.value:
            if not self._obs: # need to replace with a correct syntex
                continue
            obs = {key: value for key, value in self._obs.items()}
            msg = pickle.dumps(obs)
            pub_socket.send(msg)
            self._logger_fn("[{}] Publishing message...".format(time.time() - self._init_time))
            self._logger_fn(self._obs.items())
            self._obs.clear()

        pub_socket.close()
        self._logger_fn("Publishing Socket is closed...")

    def _subscribe(self):

        context = zmq.Context()
        sub_socket = context.socket(zmq.SUB)
        sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
        sub_socket.setsockopt(zmq.RCVBUF, 0)
        sub_socket.connect("tcp://{}:{}".format(self.ip, self.sub_port))
        self._logger_fn("Subscription Socket is running...")

        while self._running.value:
            data = sub_socket.recv()
            cmd = pickle.loads(data)
            self._cmd.update(cmd)
            self._logger_fn("[{}] Recieved message...".format(time.time() - self._init_time))
            self._logger_fn(self._cmd.items())

        sub_socket.close()
        self._logger_fn("Subscription Socket is closed...")

    def start(self):

        assert self._send_proc is None and self._recv_proc is None

        self._obs.clear()
        self._cmd.clear()
        self._cmd.update({"pose_ee_vector": np.array(INIT_POSE)})

        self._running.value = True
        self._init_time = time.time()

        self._send_proc = Process(target=self._publish,)
        self._recv_proc = Process(target=self._subscribe,)

        self._send_proc.start()
        self._recv_proc.start()

    def stop(self):
        self._running.value = False
        self._send_proc.terminate()
        self._recv_proc.terminate()
        del self._send_proc
        del self._recv_proc
        self._send_proc = None
        self._recv_proc = None

    def update(self, obs):
        self._obs.update(obs)

    @property
    def action(self):
        return copy.deepcopy(self._cmd["pose_ee_vector"])
