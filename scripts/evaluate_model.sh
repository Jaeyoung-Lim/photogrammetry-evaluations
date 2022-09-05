#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

export DATASET_PATH=$1
export OUTPUT_PATH=$2

roslaunch photogrammetry_evaluations compare_mesh.launch visualization:=false path:=$2
