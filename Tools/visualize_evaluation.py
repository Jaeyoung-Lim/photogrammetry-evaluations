import evaluate_map

import yaml
import argparse

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_increment_evaluation(ax, name, path, threshold, data_increment):
    num_images, completeness, precision = evaluate_map.model_evaluation(path, threshold, data_increment)

    ax[0].set_xlabel('Number of Images')
    ax[0].set_ylabel('Precision')
    ax[0].plot(2*num_images, precision, '-o', label=name)
    ax[0].set_xlim([0.0, max(num_images)])
    ax[0].grid(True)
    ax[0].legend(loc='lower right')

    ax[1].set_xlabel('Number of Images')
    ax[1].set_ylabel('Completeness')
    ax[1].plot(2*num_images, completeness, '-o', label=name)
    ax[1].set_xlim([0.0, max(num_images)])
    ax[1].grid(True)
    ax[1].legend()
    ax[1].legend(loc='lower right')

def plot_timed_evaluation(ax, name, path, threshold, data_increment, timestamped_path):
    num_images, completeness, precision = evaluate_map.model_evaluation(path, threshold, data_increment)

    print("Number of images", num_images)
    data_df = pd.read_csv(timestamped_path)

    #TODO: Acquire time stamps by comparing timestamp tags
    timestamp_raw = np.array(data_df['timestamp'])
    image_count_raw = np.array(data_df['image_count'])
    timestamp = np.array([])
    for images in num_images:
        idx = [ n for n,i in enumerate(image_count_raw) if i>=images ][0]
        print("images", images, ' idx ', idx, ' timestamp', timestamp_raw[idx])
        timestamp = np.append(timestamp, timestamp_raw[idx]) 

    ax[0].set_xlabel('Time[s]')
    ax[0].set_ylabel('Precision')
    ax[0].plot(timestamp, precision, '-o', label=name)
    ax[0].set_xlim([0.0, max(timestamp)])
    ax[0].grid(True)
    ax[0].legend(loc='lower right')

    ax[1].set_xlabel('Time[s]')
    ax[1].set_ylabel('Completeness')
    ax[1].plot(timestamp, completeness, '-o', label=name)
    ax[1].set_xlim([0.0, max(timestamp)])
    ax[1].grid(True)
    ax[1].legend()
    ax[1].legend(loc='lower right')

def accumulate_evaluation(path, threshold):
    figure1, ax = plt.subplots(2, 1)
    with open(path) as file:
        list = yaml.load(file, Loader=yaml.FullLoader)
        print(list)
        for key, value in list.items():
            data_path = value['path']
            data_label = value['name']
            data_increment = value['increment']
            if 'timestamp_path' in value:
                timestamped_path = value['timestamp_path']
                plot_timed_evaluation(ax, data_label, data_path, threshold, data_increment, timestamped_path)

            else:
                plot_increment_evaluation(ax, data_label, data_path, threshold, data_increment)

        plt.tight_layout()
        plt.show()

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('path', metavar='p', type=str, help='path to datafile')
parser.add_argument('--threshold', metavar='t', type=float, default=1.0, help='error threshold')
arg_list = parser.parse_args()

accumulate_evaluation(**vars(arg_list))
