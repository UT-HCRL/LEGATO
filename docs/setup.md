# Setup Guide

## Install Python Requirements
_**We strongly advise using a virtual envrionment manager such as Conda or pyenv**_

Install the environments and dependencies by running the following commands.
```
pip install -r requirement.txt
```
Then, set up the colors of collision meshes transparent in [these lines](https://github.com/ARISE-Initiative/robosuite/blob/eb01e1ffa46f1af0a3aa3ac363d5e63097a6cbcc/robosuite/utils/mjcf_utils.py#L18C39-L18C39) at `<robosuite-home>/utils/mjcf_utils.py`.

## RealSense Driver Installation for Demo Collection
Your machine must have the [Intel Realsense driver](https://github.com/IntelRealSense/librealsense) installed to use a RealSense T265 camera as a demo collection device. From 2.54.X, the codes for T265 are removed, so we need to use the version release v2.53.1 or lower. You can use the below commands for installing the Realsense driver.
```bash
# Dependencies for the Realsense driver
apt-get install libusb-1.0-0-dev xorg-dev libglu1-mesa-dev libglfw3 libglfw3-dev
# Cloning the source code of the Realsense driver
git clone https://github.com/IntelRealSense/librealsense.git DIR_TO_LIBREALSENSE
cd DIR_TO_LIBREALSENSE
# Specifying the driver version
git checkout v2.53.1
# Setting up authorizing USB devices (You may need "sudo" commands.) 
cp config/99-realsense-libusb.rules /etc/udev/rules.d/
# Building the driver
mkdir build && cd build
cmake -D BUILD_EXAMPLES=true -D BUILD_GRAPHICAL_EXAMPLES=false -D BUILD_PYTHON_BINDINGS=true -D PYTHON_EXECUTABLE=FILEPATH_TO_PYTHON ../
make -j NUMBER_OF_PROCESSES
# Installation (You may need "sudo" commands.)
make install
```

Prior to running your devices, make sure your machine allow a local user to access the device. You can use the below command at the Realsense driver source code.
```
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
```

## Julia Installation for IK acceleration
The IK optimizer implemented in this project can be accelerated using Julia. If you need IK acceleration, install Julia with the following command.
```
curl -fsSL https://install.julialang.org | sh
```

## CUDA Installation
You should install a version of CUDA corresponding to the PyTorch installation. This repo currently uses PyTorch 2.1
with a CUDA 12.1 backend. You can [install CUDA 12.1 here](https://developer.nvidia.com/cuda-12-1-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network).

## Weights & Biases Logging
This repository uses [Weights & Biases](https://wandb.ai/) for logging and data monitoring during training. To setup,
sign up for a free account on the website and authorize your account using `wandb login`. This will require an API key
you can acquire from the [authorization page](https://wandb.ai/authorize). You should only need to do this setup step once.