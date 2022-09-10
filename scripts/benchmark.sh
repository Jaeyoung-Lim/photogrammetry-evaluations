#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

usage() { echo "Usage: $0 [-i <increment>] [-n <num_images>] [-s <start_index>]" 1>&2; exit 1; }

num_benchmarks=1
max_images=200
increment=20

while getopts "i:d:n:o:p:g:m:" arg; do
    case ${arg} in
        s)
            min_images=${OPTARG}
            ;;
        m)
            max_images=${OPTARG}
            ;;
        n)
            num_benchmarks=${OPTARG}
            ;;
        i)
            increment=${OPTARG}
            ;;
        d)
            dataset_path=${OPTARG}
            ;;
        o)
            output_path=${OPTARG}
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

if [ -z ${output_path} ]; then
    echo "Error: No dataset path provided"
    exit 1
fi

instance=0

echo "====================================================="
echo "Starting Benchmark"
echo "  - Dataset Path:" ${dataset_path}
echo "  - Output Path:" ${output_path}
echo "  - Increment: " ${increment}


while [ ${instance} -le ${num_benchmarks} ]; do
    echo "====================================================="
    echo "Benchmark Instance:" ${instance}
    if [ ! -d "${dataset_path}/${instance}" ]; then
        echo "Cannot find dataset path: " ${dataset_path}/${instance} 
        exit 1
    fi
    instance_output_path=${output_path}/benchmark_${instance}
    echo "  Benchmark result path:" ${instance_output_path}
    mkdir -p ${instance_output_path}
    ${SCRIPT_DIR}/incremental_reconstruction.sh -d ${dataset_path}/${instance} -o ${instance_output_path} -i ${increment} -n ${max_images}
    instance=$(($instance + 1))
done
