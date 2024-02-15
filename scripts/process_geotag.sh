#!/bin/bash

export DATASET_PATH=$1
export OUTPUT_PATH=$2

###TODO: Check if the dataset directory exists

##TODO: Extract geotag and convert it to ecef
source ~/catkin_ws/devel/setup.bash

roslaunch photogrammetry_evaluations process_geotag.launch dataset_path:=$DATASET_PATH output_path:=$OUTPUT_PATH
