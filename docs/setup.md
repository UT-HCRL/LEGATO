# Setup Guide

## Install Python Requirements
_**We strongly advise using a virtual envrionment manager such as Conda or pyenv**_

Install the environments and dependencies by running the following commands.
```
pip install -r requirement.txt
```
Then, set up the colors of collision meshes transparent in [these lines](https://github.com/ARISE-Initiative/robosuite/blob/eb01e1ffa46f1af0a3aa3ac363d5e63097a6cbcc/robosuite/utils/mjcf_utils.py#L18C39-L18C39) at `<robosuite-home>/utils/mjcf_utils.py`.

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