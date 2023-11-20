#!/bin/bash

export DATASET_PATH=$1
export OUTPUT_PATH=$2

###TODO: Check if the dataset directory exists

start_time=$(date +%s.%3N)

touch $OUTPUT_PATH/timing.txt

process_start_time=$(date +%s.%3N)

colmap feature_extractor \
   --database_path $DATASET_PATH/database.db \
   --image_path $DATASET_PATH/images

current_time=$(date +%s.%3N)
echo "  - Feature extractor [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

process_start_time=$(date +%s.%3N)

colmap exhaustive_matcher \
   --database_path $DATASET_PATH/database.db

current_time=$(date +%s.%3N)
echo "  - Exhaustive Matcher [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

mkdir -p $DATASET_PATH/sparse

process_start_time=$(date +%s.%3N)

colmap mapper \
    --database_path $DATASET_PATH/database.db \
    --image_path $DATASET_PATH/images \
    --output_path $DATASET_PATH/sparse

current_time=$(date +%s.%3N)
echo "  - Mapper        [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

mkdir -p $DATASET_PATH/sparse/1

echo "============================================"
echo "COLMAP Model Aligner"
echo "  - input path          : " $DATASET_PATH/sparse/0/
echo "  - output path         : " $DATASET_PATH/sparse/1/
echo "  - reference image path: " $DATASET_PATH/camera.txt

process_start_time=$(date +%s.%3N)

colmap model_aligner \
    --input_path $DATASET_PATH/sparse/0/ \
    --output_path $DATASET_PATH/sparse/1 \
    --ref_images_path $DATASET_PATH/camera.txt \
    --alignment_type enu \
    --alignment_max_error 3 \
    --ref_is_gps 0 \
    --transform_path $DATASET_PATH/transform.txt
    # --robust_alignment 1 \

current_time=$(date +%s.%3N)
echo "  - Model Aligner  elapsed time [seconds]: " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

echo "============================================"
echo "COLMAP Image_undistorter"
echo "  - input path          : " $DATASET_PATH/sparse/1/
echo "  - output path         : " $DATASET_PATH/dense

mkdir -p $DATASET_PATH/dense

process_start_time=$(date +%s.%3N)

colmap image_undistorter \
    --image_path $DATASET_PATH/images \
    --input_path $DATASET_PATH/sparse/1 \
    --output_path $DATASET_PATH/dense \
    --output_type COLMAP \
    --max_image_size 2000

current_time=$(date +%s.%3N)
echo "  - Image Undistorter [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

process_start_time=$(date +%s.%3N)

colmap patch_match_stereo \
    --workspace_path $DATASET_PATH/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.geom_consistency true

current_time=$(date +%s.%3N)
echo "  - Patchmatch Stereo [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

process_start_time=$(date +%s.%3N)

colmap stereo_fusion \
    --workspace_path $DATASET_PATH/dense \
    --workspace_format COLMAP \
    --input_type geometric \
    --output_path $DATASET_PATH/dense/fused.ply

current_time=$(date +%s.%3N)
echo "  - Stereo Fusion [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt


process_start_time=$(date +%s.%3N)

colmap poisson_mesher \
    --input_path $DATASET_PATH/dense/fused.ply \
    --output_path $DATASET_PATH/dense/meshed-poisson.ply

current_time=$(date +%s.%3N)
echo "  - Poission Mesher [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

process_start_time=$(date +%s.%3N)

colmap delaunay_mesher \
    --input_path $DATASET_PATH/dense \
    --output_path $DATASET_PATH/dense/meshed-delaunay.ply

current_time=$(date +%s.%3N)
echo "  - Delaunay Mesher [seconds]:  " $(echo "scale=3; $current_time - $process_start_time" | bc) >> $OUTPUT_PATH/timing.txt

current_time=$(date +%s.%3N)
echo "  - Total elapsed Time [seconds]:  " $(echo "scale=3; $current_time - $start_time" | bc) >> $OUTPUT_PATH/timing.txt
