#!/bin/bash

export DATASET_PATH=$1
export OUTPUT_PATH=$2

###TODO: Check if the dataset directory exists

colmap feature_extractor \
   --database_path $DATASET_PATH/database.db \
   --image_path $DATASET_PATH/images

colmap exhaustive_matcher \
   --database_path $DATASET_PATH/database.db

mkdir -p $DATASET_PATH/sparse

colmap mapper \
    --database_path $DATASET_PATH/database.db \
    --image_path $DATASET_PATH/images \
    --output_path $DATASET_PATH/sparse

mkdir -p $DATASET_PATH/sparse/1

colmap model_aligner \
    --input_path $DATASET_PATH/sparse/0/ \
    --output_path $DATASET_PATH/sparse/1 \
    --ref_images_path $DATASET_PATH/camera.txt \
    --alignment_type enu \
    --robust_alignment 1 \
    --ref_is_gps 0 \
    --robust_alignment_max_error 3 \
    --transform_path $DATASET_PATH/transform.txt

mkdir $DATASET_PATH/dense

colmap image_undistorter \
    --image_path $DATASET_PATH/images \
    --input_path $DATASET_PATH/sparse/1 \
    --output_path $DATASET_PATH/dense \
    --output_type COLMAP \
    --max_image_size 2000

colmap patch_match_stereo \
    --workspace_path $DATASET_PATH/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.geom_consistency true

colmap stereo_fusion \
    --workspace_path $DATASET_PATH/dense \
    --workspace_format COLMAP \
    --input_type geometric \
    --output_path $DATASET_PATH/dense/fused.ply

colmap poisson_mesher \
    --input_path $DATASET_PATH/dense/fused.ply \
    --output_path $DATASET_PATH/dense/meshed-poisson.ply

colmap delaunay_mesher \
    --input_path $DATASET_PATH/dense \
    --output_path $DATASET_PATH/dense/meshed-delaunay.ply
