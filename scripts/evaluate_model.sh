#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

visualization=false

usage() { echo "Usage: $0 [-o <output-path>] [-g <groundtruth-mesh>] [-m <mesh>]" 1>&2; exit 1; }

while getopts "v:p:g:m:" o; do
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
        v)
            visualization=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

shift $((OPTIND-1))
echo ${groundtruth_mesh_path}
echo ${estimated_mesh_path}
if [ -z "${groundtruth_mesh_path}" ] || [ -z "${estimated_mesh_path}" ]; then
    echo "Mandatory variables missing"
    usage
else
    echo "  - Groundtruth mesh path: " ${groundtruth_mesh_path}
    echo "  - Estimated mesh path: " ${estimated_mesh_path}
    roslaunch photogrammetry_evaluations compare_mesh.launch visualization:=${visualization} path:=${output_path} \
    groundtruth_mesh:=${groundtruth_mesh_path} estimated_mesh:=${estimated_mesh_path}
fi
