#!/usr/bin/env python3
# This python script evaluates mission results

import numpy as np
import pandas as pd
import os


# Planner benchmark

def getPrecision(data_df, threshold):
    elevation_diff =  np.array(data_df['elevation_difference'])
    total = np.count_nonzero(~np.isnan(elevation_diff))
    precision = (np.abs(elevation_diff) < threshold).sum() / total
    return precision

def getCompleteness(data_df, threshold):
    elevation_diff =  np.array(data_df['elevation_difference'])
    total = np.prod(elevation_diff.shape)
    completeness = (np.abs(elevation_diff) < threshold).sum() / total
    return completeness

def model_evaluation(path, threshold, increment):
    num_data = 10
    precision=np.zeros(num_data+1)
    completeness=np.zeros(num_data+1)
    num_images = np.arange(0, (num_data+1) * increment, increment)

    for filename in os.listdir(path):
        if filename.endswith('.csv'):
            index = int(filename.split('_')[2].split('.')[0])
            f = os.path.join(path, filename)
            if os.path.isfile(f):
                data_df = pd.read_csv(f)
                precision[index+1] = getPrecision(data_df, threshold)
                completeness[index+1] = getCompleteness(data_df, threshold)
    print(" - Images: ", num_images)
    print(precision)
    return num_images, completeness, precision
