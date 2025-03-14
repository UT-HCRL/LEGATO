import cv2
import os
import sys
import argparse

cwd = os.getcwd()
sys.path.append(cwd)

def save_video_from_images(images, video_path, fps=30):
    """Save a video from a list of images."""
    height, width, _ = images[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for image in images:
        video.write(image)

    video.release()
    cv2.destroyAllWindows()

def save_video_from_image_folder(image_folder, video_path, fps=30):
    """Save a video from a folder of images."""
    # sort by name in the order of index int values
    images = {}

    for img in sorted(os.listdir(image_folder)):
        index = int(img.split('.')[0])
        images[index] = cv2.imread(os.path.join(image_folder, img))
    images = [images[key] for key in sorted(images.keys())]
    save_video_from_images(images, video_path, fps)

def main():
    parser = argparse.ArgumentParser(description='Save a video from a folder of images.')
    parser.add_argument('--image_folder', type=str, help='Path to the folder containing images.')
    parser.add_argument('--video_path', type=str, help='Path to save the video.')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second.')
    args = parser.parse_args()

    print(args)
    image_folder = args.image_folder
    video_path = args.video_path
    fps = args.fps

    save_video_from_image_folder(image_folder, video_path, fps)

if __name__ == '__main__':
    main()