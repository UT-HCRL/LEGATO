import collections
import json
import os

import h5py
import numpy as np
import json
import copy

class HDF5Recorder:
    def __init__(self, config, sim, file_path="./data") -> None:
        self._config = config
        self._sim = sim
        self._path = file_path

        self._sim_buffer = None
        self._control_buffer = None
        self._demo_buffer = None

    def reset(self):
        self._sim_buffer = None
        self._control_buffer = None
        self._demo_buffer = None

    def record(self, time_stamp, label="sim", data=None):
        if label == "sim":
            if self._sim_buffer is None:
                self._sim_buffer = {
                    "time": collections.deque(),
                    "qpos": collections.deque(),
                    "qvel": collections.deque(),
                    "ctrl": collections.deque(),
                }

            self._sim_buffer["time"].append(time_stamp)
            self._sim_buffer["qpos"].append(np.copy(self._sim.data.qpos))
            self._sim_buffer["qvel"].append(np.copy(self._sim.data.qvel))
            self._sim_buffer["ctrl"].append(np.copy(self._sim.data.ctrl))

        elif label == "demo":
            if self._demo_buffer is None:
                self._demo_buffer = {"time": collections.deque()}
                self._demo_buffer.update(
                    {
                        "action": collections.deque(),
                        "observation/img": collections.deque(),
                        "observation/state": collections.deque()
                    }
                )
            self._demo_buffer["time"].append(time_stamp)
            if self._command is not None:
                concat_action = [self._command["pos"],
                                self._command["quat"],
                                np.array([self._command["gripper"]])]
            else:
                self._command = {"pos": np.array([4.53075822e-01, -9.55911781e-04,  9.23204757e-01]), 
                                 "quat": np.array([0, 0, 0, 1]), 
                                 "gripper": 0.0}
            self._demo_buffer["action"].append(np.concatenate(concat_action))


    def close(self):
        os.makedirs(self._path, exist_ok=True)

        self._config_file = open(os.path.join(self._path, "config.json"), "w")
        json.dump(self._config.to_dict(), self._config_file, indent=4)
        self._config_file.close()

        self._env_file = open(os.path.join(self._path, "env.xml"), "wb")
        self._env_file.write(self._sim.model.get_xml().encode())
        self._env_file.close()

        self._sim_file = h5py.File(os.path.join(self._path, "sim.hdf5"), "w")
        for label, buffer in self._sim_buffer.items():
            self._sim_file.create_dataset(
                label, data=np.array(buffer), compression="gzip", chunks=True, dtype="f"
            )
        self._sim_file.close()

        self._demo_file = h5py.File(os.path.join(self._path, "demo_raw.hdf5"), "w")
        for label, buffer in self._demo_buffer.items():
            self._demo_file.create_dataset(
                label, data=np.array(buffer), compression="gzip", chunks=True, dtype="f"
            )
        self._demo_file.close()

    def _update_commands(self, command):
        self._command = copy.deepcopy(command)


class Player:
    def __init__(self, env, mode, file_path="./data", post_fcn=None) -> None:
        self._env = env
        self._config = env.config
        # self._config.from_dict(json.load(open(os.path.join(file_path, "config.json"), "r")))

        self._renderer_right = None
        self._renderer_left = None

        self._post_fcn = post_fcn

        self._mode = mode
        self._load_data(file_path)


    def reset(self):
        self._demo_count = 0
        self._sim_count = 0

        if self._mode == "replay":
            self._env._reset_objects()
            self._env._reset_recorder()

            self._cur_sim_time = 0.0
            self._cur_render_time = 0.0
            self._cur_teleop_time = 0.0

            while self._cur_sim_time < self._config.INIT_TIME:
                self._env.sim.data.qpos[:] = self._read_qpos()
                self._env.sim.forward()
                self._cur_sim_time += self._config.SIM_TIME

            if self._post_fcn is not None:
                self._post_fcn(self, mode="reset")
        else:
            initial_qpos = self._read_qpos()
            self._env.reset(initial_qpos=initial_qpos)

    def step(self):
        if self._mode == "replay":
            if self._post_fcn is not None:
                self._post_fcn(self, mode="step")

            while (
                self._cur_sim_time - self._cur_teleop_time < self._config.TELEOP_TIME
            ) and not self.done:
                self._env.sim.data.qpos[:] = self._read_qpos()
                self._env.sim.forward()
                self._sim_count += 1

                if self._cur_sim_time - self._cur_render_time >= self._config.RENDER_TIME:
                    self._render()
                    self._cur_render_time += self._config.RENDER_TIME

                self._cur_sim_time += self._config.SIM_TIME

            self._cur_teleop_time += self._config.TELEOP_TIME
        else:
            action = self._read_action()
            self._env.step(action)
        self._demo_count += 1

    def set_stereo_renderer(self, renderer_right, renderer_left):
        self._renderer_right = renderer_right
        self._renderer_left = renderer_left

    def set_renderer(self, renderer):
        self._env.set_renderer(renderer)

    def close(self):
        if self._post_fcn is not None:
            self._post_fcn(self)

    def _load_data(self, file_path):
        raise NotImplementedError()

    def _read_qpos(self):
        raise NotImplementedError

    def _read_action(self):
        raise NotImplementedError

    def _read_obs(self):
        raise NotImplementedError

    def _render(self):
        if self._env.renderer == None:
            return
        else:
            return self._env.render()

    def _get_stereo(self):
        if self._renderer_right == None or self._renderer_left == None:
            return
        else:
            return {
                "right": self._renderer_right.render(),
                "left": self._renderer_left.render(),
            }

    @property
    def done(self):
        return (
            self._demo_count >= self._demo_length or self._sim_count >= self._sim_length
        )


class HDF5Player(Player):
    def _load_data(self, file_path):
        self._path = file_path

        self._sim_file = h5py.File(os.path.join(self._path, "sim.hdf5"), "r")
        self._qpos_data = self._sim_file["qpos"][:]
        self._qvel_data = self._sim_file["qvel"][:]
        self._sim_file.close()

        self._demo_file = h5py.File(os.path.join(self._path, "demo_raw.hdf5"), "r")
        self._demo_length = self._demo_file["action"].shape[0]
        self._sim_length = self._qpos_data.shape[0]

    def _read_qpos(self):
        qpos = np.copy(self._qpos_data[self._sim_count])
        return qpos

    def _read_action(self):
        action = {}
        return action

    def _read_obs(self):
        observation = {}
        return observation

    def close(self):
        super().close()

        if self._post_fcn is not None:
            self._post_fcn(self, mode="close")


if __name__ == "__main__":
    pass
