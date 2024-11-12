import os, sys
import time
import argparse

import yaml
import h5py

import numpy as np
import matplotlib.pyplot as plt
import cv2

cwd = os.getcwd()
sys.path.append(cwd)

from devices.t265 import T265
from hardwares.gripper import Gripper
from devices.button import DemoButtonTest as DemoButton
import utils.geom as geom


def record(path, key_interface=None, gripper=None, streaming=False):

    def quit():
        t265.stop()
        low_dim_file.close()
        if streaming:
            cv2.destroyAllWindows()

    # def wait_for_start():
    #     while not interface.enable and not interface.stop:
    #         if grasp != key_interface.click:
    #             grasp = key_interface.click
    #             if grasp:
    #                 gripper.close_gripper()
    #             else:
    #                 gripper.open_gripper()
    #         else:
    #             pass

    grasp = 0
    print("Press the fob button or 'b' key...")
    # wait_for_start()
    while not key_interface.enable and not key_interface.stop:
        if grasp != key_interface.click:
            grasp = key_interface.click
            if grasp:
                gripper.close_gripper()
            else:
                gripper.open_gripper()
        else:
            pass

    print("Recording started...")
    print("1. Triple click the fob or press 's' to save the recording")
    print("2. Double click the fob or press 'd' to discard the recording")

    done = False
    trk_init = True

    mat_trans = np.eye(4)
    mat_trans[:3, :3] = geom.quat_to_rot([0.5, -0.5, -0.5, 0.5])  # T265
    pos = np.zeros(3)
    quat = np.array([0, 0, 0, 0])
    read_time = 0

    t265 = T265()
    t265.start()

    dir_name = "{}".format(int(time.time()))
    dir_path = os.path.join(path, dir_name)
    left_img_path = os.path.join(dir_path, "left_img")
    right_img_path = os.path.join(dir_path, "right_img")

    os.makedirs(dir_path, exist_ok=True)
    os.makedirs(left_img_path, exist_ok=True)
    os.makedirs(right_img_path, exist_ok=True)

    low_dim_file = open(os.path.join(dir_path, "low_dim.csv"), "w")
    low_dim_file.write(
        "time, \
        pos_x, pos_y, pos_z, \
        quat_x, quat_y, quat_z, quat_w, \
        robot_pos_x, robot_pos_y, robot_pos_z, \
        robot_quat_x, robot_quat_y, robot_quat_z, robot_quat_w, \
        robot_grasp\
        \n"
    )

    while t265.left is None or t265.right is None:
        pass 

    while not done:

        if t265.time <= read_time + 0.1:
            continue

        trk_pos = t265.pos
        trk_rot = t265.rot
        left_img = np.copy(t265.left)
        right_img = np.copy(t265.right)
        read_time += 0.1

        mat_se3 = np.eye(4)
        mat_se3[:3, :3] = geom.quat_to_rot(trk_rot)
        mat_se3[:3, 3] = trk_pos

        if trk_init:
            mat_se3_base = np.eye(4)
            mat_se3_base = np.linalg.inv(mat_trans @ mat_se3) @ mat_se3_base
            trk_init = False
            print("Tracking initialized... Start now")

        trk_mat_se3 = mat_trans @ mat_se3 @ mat_se3_base
        pos = trk_mat_se3[:3, 3]
        quat = geom.rot_to_quat(trk_mat_se3[:3, :3])

        # These two values are dummy for the consistency with the robot-demo code
        robot_pos = np.zeros(3)
        robot_quat = np.array([0, 0, 0, 1])

        if grasp != key_interface.click:
            grasp = key_interface.click
            if grasp:
                gripper.close_gripper()
            else:
                gripper.open_gripper()


        if not trk_init:

            low_dim_file.write(
                "{}, \
                                {}, {}, {},\
                                {}, {}, {}, {},\
                                {}, {}, {},\
                                {}, {}, {}, {},\
                                {}\
                            \n".format(
                    t265.time,
                    pos[0],
                    pos[1],
                    pos[2],
                    quat[0],
                    quat[1],
                    quat[2],
                    quat[3],
                    robot_pos[0],
                    robot_pos[1],
                    robot_pos[2],
                    robot_quat[0],
                    robot_quat[1],
                    robot_quat[2],
                    robot_quat[3],
                    int(grasp),
                )
            )
            cv2.imwrite(
                os.path.join(left_img_path, "{}.png".format(int(t265.time * 100))),
                left_img,
            )
            cv2.imwrite(
                os.path.join(right_img_path, "{}.png".format(int(t265.time * 100))),
                right_img,
            )

            if streaming:
                stereo_image = np.concatenate((left_img, right_img), axis=1)
                cv2.imshow("image", stereo_image)
                cv2.waitKey(1)

        done = key_interface.stop or not key_interface.enable

    if not key_interface.save:
        os.system("rm -rf {}".format(dir_path))
    quit()


def visualize(data):

    time = np.array(data["time"][:])
    left_img = np.array(data["left_img"][:])
    right_img = np.array(data["right_img"][:])
    trk_pos = np.array(data["trk_pos"][:])
    trk_rot = np.array(data["trk_rot"][:])
    imu_acc = np.array(data["imu_acc"][:])
    imu_rot = np.array(data["imu_rot"][:])
    imu_gyro = np.array(data["imu_gyro"][:])

    fig = plt.figure()
    ax1 = fig.add_subplot(131, projection="3d")
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)

    length = time.shape[0]
    img_buffer = []

    for num in range(length):

        # Prepare the 2D plot for visual observation
        x_data = trk_pos[: num + 1, 0]
        y_data = trk_pos[: num + 1, 1]
        z_data = trk_pos[: num + 1, 2]
        left_data = left_img[num]
        right_data = right_img[num]

        fig.suptitle("Time: %.2f" % time[num])
        fig.set_size_inches(5, 2.5)
        if num == 0:
            # fig.set_size_inches(15, 7)
            left_image = ax2.imshow(left_img[0])
            right_image = ax3.imshow(right_img[0])
            (line,) = ax1.plot(
                trk_pos[0, 0], trk_pos[0, 1], trk_pos[0, 2], color="blue"
            )
            (point,) = ax1.plot([], [], [], "go")
            ax1.set_xlim(-0.3, 0.3)
            ax1.set_ylim(-0.3, 0.3)
            ax1.set_zlim(-0.3, 0.3)
            # ax1.set_xlim(-1, 1)
            # ax1.set_ylim(-1, 1)
            # ax1.set_zlim(-1, 1)
            ax1.xaxis.set_tick_params(labelsize=4, pad=0)
            ax1.yaxis.set_tick_params(labelsize=4, pad=0)
            ax1.zaxis.set_tick_params(labelsize=4, pad=0)
            ax1.set_title("Hand trajectory")
            ax2.set_title("Left image")
            ax3.set_title("Right image")
            ax1.grid(True)
            ax2.tick_params(
                left=False,
                right=False,
                labelleft=False,
                labelbottom=False,
                bottom=False,
            )
            ax3.tick_params(
                left=False,
                right=False,
                labelleft=False,
                labelbottom=False,
                bottom=False,
            )

        line.set_data(x_data, y_data)
        line.set_3d_properties(z_data)

        point.set_data(x_data[-1], y_data[-1])
        point.set_3d_properties(z_data[-1])
        left_image.set_array(left_data)
        right_image.set_array(right_data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str, default="./data/tool")
    parser.add_argument("--mode", type=str, default="record")
    parser.add_argument("--param_file", type=str, default="configs/gripper.yaml")
    parser.add_argument("--button_mode", type=str, default="toggle")
    parser.add_argument(
        "--streaming", type=int, default=1, help="streaming stereo images"
    )
    args = parser.parse_args()

    path = args.path
    mode = args.mode

    if mode == "record":
        param_file = args.param_file
        button_mode = args.button_mode
        streaming = args.streaming

        with open (param_file, 'r') as file:
            params = yaml.safe_load(file)

        gripper = Gripper(params)
        key_interface = DemoButton(mode=button_mode)

        cnt = 1

        while not key_interface.stop:
            print("Recording {}th demo...".format(cnt))
            record(path, key_interface=key_interface, gripper=gripper, streaming=streaming)
            cnt += 1

    elif mode == "visualize":
        visualize(path)