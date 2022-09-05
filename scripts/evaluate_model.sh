#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

export DATASET_PATH=$1
export OUTPUT_PATH=$2

usage() { echo "Usage: $0 [-o <output-path>] [-g <groundtruth-mesh>] [-m <mesh>]" 1>&2; exit 1; }

while getopts "p:g:m:" o; do
    case "${o}" in
        g)
            groundtruth_mesh_path=${OPTARG}
            ;;
        m)
            estimated_mesh_path=${OPTARG}
            ;;
        p)
            output_path=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

shift $((OPTIND-1))
echo ${groundtruth_mesh_path}
echo ${estimated_mesh_path}
if [ -z "${groundtruth_mesh_path}" ] || [ -z "${estimated_mesh_path}" || [ -z "${output_path}"]; then
    echo "Mandatory variables missing"
    usage
else
    roslaunch photogrammetry_evaluations compare_mesh.launch visualization:=false path:=${output_path}
fi
