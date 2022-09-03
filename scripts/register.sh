#!/bin/bash

export DATASET_PATH=$1
export OUTPUT_PATH=$2

###TODO: Check if the dataset directory exists

##TODO: Extract geotag and convert it to ecef

colmap model_aligner \
    --input_path $DATASET_PATH/sparse/0/ \
    --output_path $DATASET_PATH/sparse/0/ \
    --ref_images_path $DATASET_PATH/camera.txt \
    --alignment_type enu \
    --robust_alignment 1 \
    --robust_alignment_max_error 3 \
    --transform_path $DATASET_PATH/transform.txt
