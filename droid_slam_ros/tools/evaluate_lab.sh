#!/bin/bash


ETH_PATH=datasets/ETH3D-SLAM/training

# all "non-dark" training scenes
evalset=(
    sfm_lab_room_1
)

for seq in ${evalset[@]}; do
    python evaluation_scripts/test_eth3d.py --datapath=$ETH_PATH/$seq --weights=droid.pth --depth $@
done




