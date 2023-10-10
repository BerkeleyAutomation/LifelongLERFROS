#!/bin/bash
SCRIPT_PATH="/home/lerf/lifelong_lerf_ws/src/env_setup.bash"
FOLDER_PATH="$(dirname "$SCRIPT_PATH")"
cd ~/
python3 -m venv droid_slam_env
source ~/droid_slam_env/bin/activate
cd $FOLDER_PATH
pip uninstall torch torchvision functorch tinycudann
pip install -r requirements.txt
pip install torch==2.0.1+cu118 torchvision==0.15.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
pip install nerfstudio
cd droid_slam_ros
python setup.py install
cd ..