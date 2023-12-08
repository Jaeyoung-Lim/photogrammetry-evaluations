#!/usr/bin/env python3
# This python script visualizes the geometric priors
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import yaml

def loadDataFrames(path):
    data_df = pd.read_csv(path)
    return data_df

def analyzeErrorStatistics(fig, data_df, name):
    elevation_dem = np.array(data_df["elevation_dem"])
    elevation_colmap = np.array(data_df["elevation_colmap"])
    error = elevation_dem - elevation_colmap
    n, bins, patches = fig.hist(error, bins=100, alpha=0.75, label=name)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('Number of Points')
    fig.grid(True)
    fig.legend(loc="upper right")

def analyzeUtilityStatistics(fig, data_df, name):
    elevation_dem = np.array(data_df["elevation_dem"])
    elevation_colmap = np.array(data_df["elevation_colmap"])
    error = elevation_dem - elevation_colmap
    cr_bound = np.array(data_df["cr_bound"])
    # n, bins, patches = fig.hist(error, bins=100, alpha=0.75, label=name)
    fig.scatter(cr_bound, np.abs(error), s=2, lw = 0, alpha=0.01)
    fig.set_xlabel('Cramer-Rao Bounds')
    fig.set_ylabel('Error')
    fig.grid(True)
    fig.legend(loc="upper right")

def analyzePrecision(fig, data_df, name):
    error = np.array(data_df["error"])

    # total = np.prod(error.shape)
    total = np.count_nonzero(~np.isnan(error))
    precision = np.array([])
    length = np.arange(0.0, 5.0, 0.05)
    for threshold in length:
        rate = (error < threshold).sum() / total
        precision = np.append(precision, rate)
        if threshold == 2.0:
            print("precision: ", rate, " [Error: ", threshold, "]")

    fig.plot(length, precision, '-', label=name)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('Precision')
    fig.set_title('Precision')
    fig.grid(True)
    fig.legend(loc="lower right")

def analyzeCompleteness(fig, data_df, name):
    error = np.array(data_df["error"])

    total = np.prod(error.shape)
    # total = np.count_nonzero(~np.isnan(error))
    completeness = np.array([])
    length = np.arange(0.0, 5.0, 0.05)
    for threshold in length:
        rate = (error < threshold).sum() / total
        completeness = np.append(completeness, rate)
        if threshold == 2.0:
            print("completeness: ", rate, " [Error: ", threshold, "]")
    fig.plot(length, completeness, '-', label=name)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('Completeness')
    fig.set_title('Completeness')
    fig.grid(True)
    fig.legend(loc="lower right")

def analyzeF1score(fig, data_df, name):
    error = np.array(data_df["error"])

    total_completness = np.prod(error.shape)
    total_precision = np.count_nonzero(~np.isnan(error))

    # total = np.count_nonzero(~np.isnan(error))
    completeness = np.array([])
    precision = np.array([])

    length = np.arange(0.0, 5.0, 0.05)
    for threshold in length:
        completeness_rate = (error < threshold).sum() / total_completness
        completeness = np.append(completeness, completeness_rate)
        precision_rate = (error < threshold).sum() / total_precision
        precision = np.append(precision, precision_rate)

    f1_score = 2 * np.divide(np.multiply(precision, completeness), np.add(precision, completeness))
    fig.plot(length, f1_score, '-', label=name)
    fig.set_xlabel('Error [m]')
    fig.set_ylabel('F1 Score')
    fig.set_title('F1 Score')
    fig.grid(True)
    fig.legend(loc="lower right")


fig1 = plt.figure("Error Statistics")
#Error Histogram
ax11 = fig1.add_subplot(1, 1, 1)

fig2 = plt.figure("Utility Statistics")
ax21 = fig2.add_subplot(1, 1, 1)


with open(sys.argv[1]) as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    print("  Reading: ", file)
    map_data_df = pd.read_csv(file)
    print(map_data_df)
    analyzeErrorStatistics(ax11, map_data_df, "")
    analyzeUtilityStatistics(ax21, map_data_df, "")
    # analyzePrecision(ax31, map_data_df, "")
    # analyzeCompleteness(ax32, map_data_df, "")
    # analyzeF1score(ax33, map_data_df, "")
plt.tight_layout()
plt.legend()
plt.show()
