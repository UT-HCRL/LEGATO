import numpy as np
import h5py
import torch
import torchvision
import imageio
import math
import argparse
import cv2
from multiprocessing import Process, Manager
import matplotlib.pyplot as plt
import io
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from simulator.wrapper import unwrap_delta_action

def generate_video_stream(imgs, fps=30, video_name="./video.mp4"):
    try:
        if "mp4" not in video_name:
            video_name = f"{video_name}.mp4"
        video_writer = imageio.get_writer(video_name, fps=fps)
        for img in imgs:
            video_writer.append_data(img)
        video_writer.close()
        return True
    except:
        return False


def get_img_from_fig(fig, dpi=360):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi)
    buf.seek(0)
    img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
    buf.close()
    img = cv2.imdecode(img_arr, 1)
    return img


def obs_visualizer(demo_file, demo_name, result_queue):
    tag = demo_file.attrs["tag"]
    if 'right_rgb' in demo_file["obs"].keys():
        images = demo_file["right_rgb"][()]
    elif 'right_gray' in demo_file["obs"].keys():
        images = demo_file["obs/right_gray"][()]
        if images.shape[-1] == 1:
            images = np.repeat(images, 3, axis=-1)
    ra_img = []
    for idx in range(images.shape[0]):
        img = np.array(images[idx][:, :, ::-1], dtype=np.uint8)
        img = cv2.putText(img, tag, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        ra_img.append(img)
    result_queue.put((demo_name, ra_img))


def act_visualizer(demo_file, demo_name, result_queue):
    tag = demo_file.attrs["tag"]
    actions = np.array(demo_file["actions"][()])
    command_buffer = {
        'pose_ee_vector': [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1])],
        'gripper': [0.0]
    }
    for idx in range(actions.shape[0]):
        action = actions[idx]
        new_command = unwrap_delta_action(action, command_buffer, mode='rpy')
        for key in new_command.keys():
            command_buffer[key].append(new_command[key])
    trajectories = np.array(command_buffer['pose_ee_vector'])
    grasping_states = np.array(command_buffer['gripper'])

    fig = plt.figure()
    fig.set_size_inches(2, 2)
    ax = fig.add_subplot(111, projection='3d')

    ra_img = []

    for num in range(trajectories.shape[0]):
        x_data = trajectories[: num + 1, 0]
        y_data = trajectories[: num + 1, 1]
        z_data = trajectories[: num + 1, 2]

        if num == 0:
            subtitle_text = fig.text(0.5, 0.02, "Step: {}".format(num), ha='center', fontsize=12)
            point, = ax.plot(x_data, y_data, z_data, color='green', marker='o')
            ax.set_xlim(-0.3, 0.3)
            ax.set_ylim(-0.3, 0.3)
            ax.set_zlim(-0.3, 0.3)
            ax.xaxis.set_tick_params(labelsize=4, pad=0)
            ax.yaxis.set_tick_params(labelsize=4, pad=0)
            ax.zaxis.set_tick_params(labelsize=4, pad=0)
            # ax.set_title("Hand trajectory")

        subtitle_text.set_text("Step: {}".format(num))            
            
        if grasping_states[num] > 0.5:
            ax.plot(x_data[-2:], y_data[-2:], z_data[-2:], color='red')
        else:
            ax.plot(x_data[-2:], y_data[-2:], z_data[-2:], color='blue')
                        
        point.set_data(x_data[-1], y_data[-1])
        point.set_3d_properties(z_data[-1])        

        img = get_img_from_fig(fig, dpi=120)
        ra_img.append(img)

    for img in ra_img:
        img = cv2.putText(img, tag, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    result_queue.put((demo_name, ra_img))


def main(dataset_path,  visualizer_fcn, dimensions=None, out_path="./video.mp4", num_workers=48, **kwargs):
    demo_file_name = dataset_path
    demo_file = h5py.File(demo_file_name, "r")

    keys = list(demo_file["data"].keys())

    num_videos = len(keys) 
    # total number of demo videos to display
    # if dimensions are not given, we display everything in one page. 
    # Otherwise, fit as many as we can in each page

    page_size = num_videos if dimensions is None else dimensions[0] * dimensions[1]
    num_pages = math.ceil(num_videos / page_size)
    final_images = []

    for page in range(num_pages):

        video_source = {}
        jobs = []
        manager = Manager()
        q = manager.Queue()
        for demo in keys[page * page_size : page * page_size + page_size]:

            # Process data with a limited worker
            while len(jobs) >= num_workers:
                for job in jobs:
                    if not job.is_alive():
                        job.join()
                        jobs.remove(job)
                        break
            job = Process(target=visualizer_fcn, args=(demo_file[f"data/{demo}"], demo, q), kwargs=kwargs)
            jobs.append(job)
            job.start()
            print("Processing demo {}...".format(demo))

        for job in jobs:
            job.join()
        
        video_source = {}
        while not q.empty():
            demo, imgs = q.get()
            video_source[demo] = imgs

        print("Generating video for page {}...".format(page))
        
        max_len = 0
        for images in video_source.values():
            if max_len < len(images):
                max_len = len(images)
        for run_idx in video_source.keys():
            for _ in range(max_len + 5 - len(video_source[run_idx])):
                video_source[run_idx].append(np.copy(video_source[run_idx][-1]))
        for i in range(max_len + 4):
            imgs = []
            for run_idx in video_source.keys():
                imgs.append(video_source[run_idx][i])
            imgs = np.stack(imgs, axis=0)
            img_tensor = torch.tensor(imgs)
            nrow = 10 if dimensions is None else dimensions[0]
            grid_img = torchvision.utils.make_grid(img_tensor.permute(0, 3, 1, 2), nrow=nrow)
            final_images.append(grid_img.permute(1, 2, 0).detach().numpy()[:, :, 2::-1].astype(np.uint8))
        
    generate_video_stream(final_images, video_name=out_path)        
    demo_file.close()

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=str)
    parser.add_argument("--mode", default="obs", type=str)
    parser.add_argument("--out", default="./video.mp4", type=str)
    parser.add_argument("--width", type=int, default=5)
    parser.add_argument("--height", type=int, default=5)
    args = parser.parse_args()

    if args.width == 0 or args.height == 0:
        dimensions = None
    else:
        dimensions = (args.width, args.height)

    if args.mode == "obs":
        visualizer_fcn = obs_visualizer
    else:
        visualizer_fcn = act_visualizer

    main(args.dataset, visualizer_fcn, dimensions, out_path=args.out)