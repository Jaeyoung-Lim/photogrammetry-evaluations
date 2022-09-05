#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

export DATASET_PATH=$1
export OUTPUT_PATH=$2

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

instance=0
num_images=$(ls ${DATASET_PATH}/images | wc -l)
min_images=5
increment=5
max_images=20

target_num_images=$((${num_images}>${max_images} ? ${num_images} : ${max_images}))
echo "Total number of images: " ${num_images}
echo "Target number of images: " ${target_num_images}

while [ $instance -lt $target_num_images ]; do
    INSTANCE_DATASET_PATH=${OUTPUT_PATH}/output/${instance}
    mkdir -p ${INSTANCE_DATASET_PATH}/images
    num_subset_images=0
    for FILE in ${DATASET_PATH}/images/* ; do
        echo "Copy file ${FILE}";
        cp $FILE ${INSTANCE_DATASET_PATH}/images/
        num_subset_images=$(($num_subset_images + 1))
        min_subset_images=
        if [ $num_subset_images -ge $(( $((${increment}*${instance})) + ${min_images})) ]
        then
            echo "number of subset images: $num_subset_images instance: $instance"
            break;
        fi
    done

    ${SCRIPT_DIR}/reconstruction.sh ${INSTANCE_DATASET_PATH} ${INSTANCE_DATASET_PATH}

    ${SCRIPT_DIR}/evaluate_model.sh -g "/dev/mesh/groundtruth_roi_meshlab.obj" \
    -m ${INSTANCE_DATASET_PATH}/dense/meshed-delaunay.ply -o ${INSTANCE_DATASET_PATH}

    instance=$(($instance + 1))
done
