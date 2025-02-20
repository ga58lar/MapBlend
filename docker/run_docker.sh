#!/bin/bash

# Check correct call of script
if [ $# -ne 12 ]; then
    echo "Usage: $0 <model_path> <config_path> <map_path> <scan_path> <output_path> <x> <y> <z> <qx> <qy> <qz> <qw>"
    echo "Example: $0 models/ config/ map.pcd scans/ output/ 0 0 0 0 0 0 1"
    exit 1
fi

MODEL_PATH=$1
CONFIG_PATH=$2
MAP_PATH=$3
PCD_PATH=$4
OUTPUT_PATH=$5
X=$6
Y=$7
Z=$8
QX=$9
QY=$10
QZ=$11
QW=${12}

tag=0.0.1

xhost +
docker run --rm -it \
    --network=host \
    -v /dev/shm:/dev/shm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --privileged \
    --gpus=all \
	-e QT_X11_NO_MITSHM=1 \
	--runtime=nvidia \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
    -v $CONFIG_PATH:/config_path \
    -v $MAP_PATH:/map_path.pcd \
    -v $PCD_PATH:/pcd_path \
    -v $OUTPUT_PATH:/output_path \
    -v $MODEL_PATH:/tum_models \
    ga58lar/mapblend:$tag \
    bash -c "cd /dev_ws/build && ./mapblend /config_path /map_path.pcd /pcd_path /output_path $X $Y $Z $QX $QY $QZ $QW"
xhost -
