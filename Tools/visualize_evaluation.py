import evaluate_map

import yaml
import argparse
import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_increment_evaluation(ax, name, linestyle, path, threshold, data_increment):
    num_images, completeness, precision = evaluate_map.model_evaluation(path, threshold, data_increment)

    ax[0].set_xlabel('Number of Images')
    ax[0].set_ylabel('Precision')
    ax[0].plot(num_images, precision, linestyle, label=name, markersize=4)
    ax[0].fill_between(num_images, precision, precision, alpha=0.2)
    ax[0].set_xlim([0.0, max(num_images)])
    ax[0].set_xticks(np.arange(0, max(num_images), step=data_increment))
    ax[0].set_yticks(np.arange(0, 1.1, step=0.5))
    ax[0].set_ylim([0.0, 1.1])
    ax[0].grid(True)
    # ax[0].legend(loc='lower right')

    ax[1].set_xlabel('Number of Images')
    ax[1].set_ylabel('Completeness')
    ax[1].plot(num_images, completeness, linestyle, label=name, markersize=4)
    ax[1].fill_between(num_images,completeness, completeness, alpha=0.2)
    ax[1].set_xlim([0.0, max(num_images)])
    ax[1].set_xticks(np.arange(0, max(num_images), step=data_increment))
    ax[1].set_yticks(np.arange(0, 1.1, step=0.5))
    ax[1].set_ylim([0.0, 1.1])
    ax[1].grid(True)
    # ax[1].legend(loc='lower right')

def plot_benchmark(ax, name, linestyle, path, threshold, data_increment):

    image_count = np.array([])
    accumulate_completeness=np.array([])
    accumulate_precision=np.array([])
    for dirname in os.listdir(path):
        print(dirname)
        instance_path = os.path.join(path, dirname)
        num_images, completeness, precision = evaluate_map.model_evaluation(instance_path, threshold, data_increment)
        if (accumulate_completeness.size == 0):
            accumulate_completeness = completeness
        else:
            accumulate_completeness = np.vstack([accumulate_completeness, completeness])
        if (accumulate_precision.size == 0):
            accumulate_precision = precision
        else:
            accumulate_precision = np.vstack([accumulate_precision, precision])
    
    mean_completeness = np.mean(accumulate_completeness, axis=0)
    std_completeness = np.std(accumulate_completeness, axis=0)

    mean_precision = np.mean(accumulate_precision, axis=0)
    std_precision = np.std(accumulate_completeness, axis=0)

    ax[0].set_xlabel('Number of Images')
    ax[0].set_ylabel('Precision')
    ax[0].plot(num_images, mean_precision, linestyle, label=name, markersize=4)
    ax[0].fill_between(num_images, mean_precision-std_precision, mean_precision+std_precision, alpha=0.2)
    ax[0].set_xlim([0.0, max(num_images)])
    ax[0].set_xticks(np.arange(0, max(num_images), step=data_increment))
    ax[0].set_yticks(np.arange(0, 1.1, step=0.5))
    ax[0].set_ylim([0.0, 1.1])
    ax[0].grid(True)
    # ax[0].legend(loc='lower right')

    ax[1].set_xlabel('Number of Images')
    ax[1].set_ylabel('Completeness')
    ax[1].plot(num_images, mean_completeness, linestyle, label=name, markersize=4)
    ax[1].fill_between(num_images, mean_completeness-std_completeness, mean_completeness+std_completeness, alpha=0.2)
    ax[1].set_xlim([0.0, max(num_images)])
    ax[1].set_xticks(np.arange(0, max(num_images), step=data_increment))
    ax[1].set_yticks(np.arange(0, 1.1, step=0.5))
    ax[1].set_ylim([0.0, 1.1])
    ax[1].grid(True)
    # ax[1].legend(loc='lower right')


def plot_timed_evaluation(ax, name, linestyle, path, threshold, data_increment, timestamped_path):
    num_images, completeness, precision = evaluate_map.model_evaluation(path, threshold, data_increment)
    print(num_images)
    print("Number of images", num_images)
    data_df = pd.read_csv(timestamped_path)

    #TODO: Acquire time stamps by comparing timestamp tags
    timestamp_raw = np.array(data_df['timestamp'])
    image_count_raw = np.array(data_df['image_count'])
    timestamp = np.array([])
    for images in num_images:
        idx = [ n for n,i in enumerate(image_count_raw) if i>=images ][0]
        print("images", images, ' idx ', idx, ' timestamp', timestamp_raw[idx])
        print("idx: ", idx)
        timestamp = np.append(timestamp, timestamp_raw[idx]) 

    ax[0].set_xlabel('Time[s]')
    ax[0].set_ylabel('Precision')
    ax[0].plot(timestamp, precision, '-o', label=name, markersize=4)
    ax[0].set_xlim([0.0, max(timestamp)])
    ax[0].grid(True)
    ax[0].legend(loc='lower right')
    ax[0].set_xticks(np.arange(0, max(timestamp), step=40))

    ax[1].set_xlabel('Time[s]')
    ax[1].set_ylabel('Completeness')
    ax[1].plot(timestamp, completeness, '-o', label=name, markersize=4)
    ax[1].set_xlim([0.0, max(timestamp)])
    ax[1].set_xticks(np.arange(0, max(timestamp), step=40))
    ax[1].set_yticks(np.arange(0, 1.1, step=0.5))
    ax[1].grid(True)
    ax[1].legend(loc='lower right')

def accumulate_evaluation(path, threshold):
    figure1, ax = plt.subplots(2, 1)
    figure1.set_size_inches((6, 4))
    with open(path) as file:
        list = yaml.load(file, Loader=yaml.FullLoader)
        print(list)
        for key, value in list.items():
            data_label = value['name']
            data_increment = value['increment']
            linestyle = value['linestyle']
            if 'timestamp_path' in value:
                data_path = value['path']
                timestamped_path = value['timestamp_path']
                plot_timed_evaluation(ax, data_label, linestyle, data_path, threshold, data_increment, timestamped_path)
            elif 'benchmark_path' in value:
                benchmark_path = value['benchmark_path']
                plot_benchmark(ax, data_label, linestyle, benchmark_path, threshold, data_increment)
            else:
                data_path = value['path']
                plot_increment_evaluation(ax, data_label, linestyle, data_path, threshold, data_increment)
        handles, labels = ax[1].get_legend_handles_labels()
        # plt.legend(bbox_to_anchor=(1, -0.1), loc="lower right", bbox_transform=figure1.transFigure, ncol=3)
        leg = ax[1].legend(handles, labels, bbox_to_anchor=(1, -0.2), loc="lower right", bbox_transform=figure1.transFigure, ncol=3, )
        figure1.savefig('samplefigure.pdf', bbox_extra_artists=(leg,), bbox_inches='tight')
        plt.tight_layout()
        plt.show()

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('path', metavar='p', type=str, help='path to datafile')
parser.add_argument('--threshold', metavar='t', type=float, default=1.0, help='error threshold')
arg_list = parser.parse_args()

accumulate_evaluation(**vars(arg_list))
