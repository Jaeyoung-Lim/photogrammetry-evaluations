#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

# set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

usage() { echo "Usage: $0 [-i <increment>] [-n <num_images>] [-s <start_index>]" 1>&2; exit 1; }

min_images=10
increment=10
max_images=200

while getopts ":i:d:o:p:g:n:" arg; do
    case ${arg} in
        s)
            min_images=${OPTARG}
            ;;
        n)
            max_images=${OPTARG}
            ;;
        i)
            increment=${OPTARG}
            min_images=${increment}
            ;;
        d)
            DATASET_PATH=${OPTARG}
            ;;
        o)
            OUTPUT_PATH=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

instance=0
num_images=$(ls ${DATASET_PATH}/images/*.JPG | wc -l)


target_num_images=$((${num_images}>${max_images} ? ${max_images} : ${num_images}))
echo "====================================================="
echo "Dataset path           : " ${DATASET_PATH}
echo "Output  path           : " ${OUTPUT_PATH}
echo "Total number of images : " ${num_images}
echo "Target number of images: " ${target_num_images}
echo "Increment              : " ${increment}



while [ $(( $((${increment}*${instance})) + ${min_images})) -le $target_num_images ]; do
    INSTANCE_DATASET_PATH=${OUTPUT_PATH}/${instance}
    echo "====================================================="
    echo "Instance:" ${instance}
    echo "  - number of images:" $(( $((${increment}*${instance})) + ${min_images}))
    echo "  - instance dataset path:" ${INSTANCE_DATASET_PATH}
    mkdir -p ${INSTANCE_DATASET_PATH}/images
    num_subset_images=0
    cp ${DATASET_PATH}/camera.txt ${INSTANCE_DATASET_PATH}/camera.txt
    for FILE in ${DATASET_PATH}/images/* ; do
        echo "Copy file ${FILE}";
        cp $FILE ${INSTANCE_DATASET_PATH}/images/
        num_subset_images=$(($num_subset_images + 1))
        if [ $num_subset_images -ge $(( $((${increment}*${instance})) + ${min_images})) ]
        then
            echo "Number of subset images: $num_subset_images instance: $instance"
            break;
        fi
    done

    echo " - Running reconstruction..."
    ${SCRIPT_DIR}/reconstruction.sh ${INSTANCE_DATASET_PATH} ${INSTANCE_DATASET_PATH} > ${INSTANCE_DATASET_PATH}/reconstruction.log 2>&1
    echo " - Reconstruction done log: " 
    echo " - Running model evaluation..." ${INSTANCE_DATASET_PATH}/reconstruction.log
    ${SCRIPT_DIR}/evaluate_model.sh -g "/home/jaeyoung/dev/mesh/groundtruth_roi_meshlab.obj" \
    -m ${INSTANCE_DATASET_PATH}/dense/meshed-delaunay.ply -p ${OUTPUT_PATH}/map_data_${instance}.csv > ${INSTANCE_DATASET_PATH}/evaluate.log 2>&1
    echo " - Model evaluation done log: " ${INSTANCE_DATASET_PATH}/evaluate.log

    instance=$(($instance + 1))
done
