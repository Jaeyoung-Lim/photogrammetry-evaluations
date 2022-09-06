#!/usr/bin/env python3
# This python script evaluates mission results

import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import yaml
import argparse
import os


# Planner benchmark

def getCompleteness(data_df, threshold):
    elevation_diff =  np.array(data_df['elevation_difference'])
    total = np.count_nonzero(~np.isnan(elevation_diff))
    precision = (np.abs(elevation_diff) < threshold).sum() / total
    return precision

def getPrecision(data_df, threshold):
    elevation_diff =  np.array(data_df['elevation_difference'])
    total = np.prod(elevation_diff.shape)
    completeness = (np.abs(elevation_diff) < threshold).sum() / total
    return completeness

def model_evaluation(path, threshold):
    #TODO: Iterate on csv files to plot
    precision=np.zeros(10)
    completeness=np.zeros(10)
    num_images = np.zeros(10)
    for filename in os.listdir(path):
        if filename.endswith('.csv'):
            index = int(filename.split('_')[2].split('.')[0])
            num_images[index+1] = 5 * (index+1)
            f = os.path.join(path, filename)
            if os.path.isfile(f):
                data_df = pd.read_csv(f)
                precision[index+1] = getPrecision(data_df, threshold)
                completeness[index+1] = getCompleteness(data_df, threshold)
    return num_images, completeness, precision

def accumulate_evaluation(path, threshold):
    figure1 = plt.figure("Map Data")

    output_path = path
    num_images, completeness, precision = model_evaluation(output_path, threshold)

    axis1 = figure1.add_subplot(2, 1, 1)
    axis1.set_title('Completeness')
    axis1.set_xlabel('Number of Images')
    axis1.plot(num_images, completeness, '-o')
    axis1.grid(True)
    axis2 = figure1.add_subplot(2, 1, 2)
    axis2.set_title('Precision')
    axis2.set_xlabel('Number of Images')
    axis2.plot(num_images, precision, '-o')
    axis2.grid(True)
    plt.tight_layout()
    plt.show()

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('path', metavar='p', type=str, help='path to datafile')
parser.add_argument('--threshold', metavar='t', type=float, default=1.0, help='error threshold')
arg_list = parser.parse_args()

accumulate_evaluation(**vars(arg_list))
