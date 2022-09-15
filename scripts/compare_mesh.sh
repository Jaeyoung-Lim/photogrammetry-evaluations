#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

usage() { echo "Usage: $0 [-i <increment>] [-n <num_images>] [-s <start_index>]" 1>&2; exit 1; }

num_benchmarks=1
max_images=200
increment=20
groundtruth_path="/home/jaeyoung/dev/mesh/groundtruth_roi_meshlab.obj"

while getopts "g:i:d:n:o:p:g:m:" arg; do
    case ${arg} in
        n)
            num_benchmarks=${OPTARG}
            ;;
        i)
            increment=${OPTARG}
            ;;
        d)
            dataset_path=${OPTARG}
            ;;
        g)
            groundtruth_path=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

if [ -z ${dataset_path} ]; then
    echo "Error: No dataset path provided"
    exit 1
fi

instance=0
OUTPUT_PATH=${dataset_path}

mesh_instance=0

echo "====================================================="
echo "Starting Benchmark"
echo "  - Dataset Path:" ${dataset_path}


echo "====================================================="
echo "Processing Instance:"
if [ ! -d "${dataset_path}/${instance}" ]; then
    echo "Cannot find dataset path: " ${dataset_path}/${instance}
    exit 1
fi

while [ ${mesh_instance}  -lt 10 ]; do
    INSTANCE_DATASET_PATH=${OUTPUT_PATH}/${mesh_instance}
    echo "====================================================="
    groundtruth_path=/home/jaeyoung/dev/mesh/groundtruth_roi_meshlab.obj
    estimatedmesh_path=${INSTANCE_DATASET_PATH}/dense/meshed-poisson.ply
    output_path=${OUTPUT_PATH}/poisson/poisson_map_${mesh_instance}.csv
    echo "Mesh Instance:" ${mesh_instance}
    echo "Output Path: " ${output_path}
    echo "Mesh Path: " ${estimatedmesh_path}
    echo "Groundtruth Path: " ${groundtruth_path}
    mkdir -p ${OUTPUT_PATH}/poisson
    # roslaunch photogrammetry_evaluations compare_mesh.launch visualization:=false path:=${OUTPUT_PATH}/poisson/map_data_${mesh_instance}.csv \
    # groundtruth_mesh:=${groundtruth_path} estimated_mesh:=${INSTANCE_DATASET_PATH}/dense/meshed-poisson.ply > ${OUTPUT_PATH}/poisson/poission_${mesh_instance}.log 2>&1
    roslaunch photogrammetry_evaluations compare_mesh.launch visualization:=false path:=${output_path} \
    groundtruth_mesh:=${groundtruth_path} estimated_mesh:=${estimatedmesh_path} > ${OUTPUT_PATH}/poisson/poission_${mesh_instance}.log 2>&1

    # ${SCRIPT_DIR}/evaluate_model.sh -g ${groundtruth_path} \
    # -m ${INSTANCE_DATASET_PATH}/dense/meshed-poisson.ply -p ${OUTPUT_PATH}/poisson/map_data_${mesh_instance}.csv > ${OUTPUT_PATH}/poisson/poission_${mesh_instance}.log 2>&1
    echo " - Model evaluation done log: " ${OUTPUT_PATH}/poisson/poission.log
    mesh_instance=$(($mesh_instance + 1))
done
