#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

set -e

# TODO: Run airsim client to generate data multiple times

num_simulations=1
num_samples=50

while getopts "s:n:o:" arg; do
    case ${arg} in
        o)
            dataset_path=${OPTARG}
            ;;
        n)
            num_simulations=${OPTARG}
            ;;
        s)
            num_samples=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

if [ -z $dataset_path ]; then
    echo "Error: No dataset path specified"
    exit 1
fi

echo "Datset path: " ${dataset_path}


instance=0

while [ ${instance} -lt ${num_simulations} ]; do
    echo "Generate dataset instance: ${instance}"
    instance_path=${dataset_path}/${instance}
    image_path=${instance_path}/images
    mkdir -p ${instance_path}
    mkdir -p ${image_path}
    roslaunch airsim_client test_random_airsim.launch output_path:=${instance_path}\
        visualization:=false num_samples:=${num_samples} > ${instance_path}/dataset.log 2>&1

    instance=$(($instance + 1))
done

echo "Ended Dataset Generation"
