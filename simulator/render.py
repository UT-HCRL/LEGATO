import cv2
import numpy as np
from robosuite.utils.binding_utils import MjRenderContextOffscreen

RIGHTFORWARD_GRIPPER = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
RIGHTUP_GRIPPER = np.array([[0, 0, 1], [0, 1, 0], [1, 0, 0]])

WIDTH = 1096 * 2
HEIGHT = 1176


class PNGRenderer:
    def __init__(
        self,
        device_id,
        sim,
        cam_name,
        width=480,
        height=360,
        depth=False,
        rgb=True,
        segmentation=False,
        save_path=None,
        gui=True,
    ) -> None:
        self._render_context = MjRenderContextOffscreen(sim, device_id=device_id)
        sim.add_render_context(self._render_context)

        self._width = width
        self._height = height
        self._depth = depth
        self._segmentation = segmentation
        self._cam_id = sim.model.camera_name2id(cam_name)
        self._gui = gui
        self._rgb = rgb

        if save_path is not None:
            self._save_cnt = 0
            self._save_file = save_path


    def render(self, img_name=None):
        self._render_context.render(
            width=self._width,
            height=self._height,
            camera_id=self._cam_id,
            segmentation=self._segmentation,
        )
        img = self._render_context.read_pixels(
            self._width,
            self._height,
            depth=self._depth,
            segmentation=self._segmentation,
        )

        # convert rgb to grayscale
        if not self._rgb:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = np.expand_dims(img, axis=-1)

        if self._save_file is not None:
            if img_name is not None:
                cv2.imwrite(f"{self._save_file}/{img_name}.png", img[:, ::-1, ::-1])
            else:
                cv2.imwrite(f"{self._save_file}/{self._save_cnt:03d}.png", img[:, ::-1, ::-1])
                self._save_cnt += 1
        ###
        if self._gui:
            cv2.imshow("test", img[:, ::-1, ::-1])
            cv2.waitKey(1)
        return img

    def close(self):
        if self._save_file is not None:
            self._save_file = None
            self._save_cnt = 0

    def reset(self, save_path=None):
        self.close()

        if save_path is not None:
            self._save_file = cv2.VideoWriter(
                save_path, cv2.VideoWriter_fourcc(*"mp4v"), 30, (self._width, self._height)
            )
        else:
            self._save_file = None


class CV2Renderer:
    def __init__(
        self,
        device_id,
        sim,
        cam_name,
        width=480,
        height=360,
        depth=False,
        rgb=True,
        segmentation=False,
        save_path=None,
        gui=True,
    ) -> None:
        self._render_context = MjRenderContextOffscreen(sim, device_id=device_id)
        sim.add_render_context(self._render_context)

        self._width = width
        self._height = height
        self._depth = depth
        self._segmentation = segmentation
        self._cam_id = sim.model.camera_name2id(cam_name)
        self._gui = gui
        self._rgb = rgb

        if save_path is not None:
            self._save_file = cv2.VideoWriter(
                save_path, cv2.VideoWriter_fourcc(*"mp4v"), 30, (width, height)
            )
        else:
            self._save_file = None

    def render(self):
        self._render_context.render(
            width=self._width,
            height=self._height,
            camera_id=self._cam_id,
            segmentation=self._segmentation,
        )
        img = self._render_context.read_pixels(
            self._width,
            self._height,
            depth=self._depth,
            segmentation=self._segmentation,
        )

        # convert rgb to grayscale
        if not self._rgb:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = np.expand_dims(img, axis=-1)

        if self._save_file is not None:
            self._save_file.write(img[:, ::-1, ::-1])
        ###
        if self._gui:
            cv2.imshow("test", img[:, ::-1, ::-1])
            cv2.waitKey(1)
        return img

    def close(self):
        if self._save_file is not None:
            self._save_file.release()
            self._save_file = None

    def reset(self, save_path=None):
        self.close()

        if save_path is not None:
            self._save_file = cv2.VideoWriter(
                save_path, cv2.VideoWriter_fourcc(*"mp4v"), 30, (self._width, self._height)
            )
        else:
            self._save_file = None


def convert_orientation(VR_orientation):
    T = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]])
    B_inv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    return T @ B_inv @ VR_orientation @ T.T


if __name__ == "__main__":
    pass
