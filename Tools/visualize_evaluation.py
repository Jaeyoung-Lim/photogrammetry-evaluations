import evaluate_map

import yaml
import argparse
import matplotlib.pyplot as plt

def plot_evaluation(ax, name, path, threshold):
    num_images, completeness, precision = evaluate_map.model_evaluation(path, threshold)

    ax[0].set_xlabel('Number of Images')
    ax[0].set_ylabel('Precision')
    ax[0].plot(num_images, precision, '-o', label=name)
    ax[0].set_xlim([0.0, 50.0])
    ax[0].grid(True)
    ax[0].legend()

    ax[1].set_xlabel('Number of Images')
    ax[1].set_ylabel('Completeness')
    ax[1].plot(num_images, completeness, '-o', label=name)
    ax[1].set_xlim([0.0, 50.0])
    ax[1].grid(True)
    ax[1].legend()

def accumulate_evaluation(path, threshold):
    figure1, ax = plt.subplots(2, 1)
    with open(path) as file:
        list = yaml.load(file, Loader=yaml.FullLoader)
        print(list)
        for key, value in list.items():
            data_path = value['path']
            data_label = value['name']
            print(data_path)
            plot_evaluation(ax, data_label, data_path, threshold)
        plt.tight_layout()
        plt.show()

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('path', metavar='p', type=str, help='path to datafile')
parser.add_argument('--threshold', metavar='t', type=float, default=1.0, help='error threshold')
arg_list = parser.parse_args()

accumulate_evaluation(**vars(arg_list))
