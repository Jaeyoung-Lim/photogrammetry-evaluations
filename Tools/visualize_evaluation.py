import evaluate_map
import argparse
import matplotlib.pyplot as plt

def plot_evaluation(fig, name, path, threshold):
    num_images, completeness, precision = evaluate_map.model_evaluation(path, threshold)

    axis1 = fig.add_subplot(2, 1, 1)
    axis1.set_xlabel('Number of Images')
    axis1.set_ylabel('Completeness')
    axis1.plot(num_images, completeness, '-o', label=name)
    axis1.set_xlim([0.0, 50.0])
    axis1.grid(True)
    axis1.legend()

    axis2 = fig.add_subplot(2, 1, 2)
    axis2.set_xlabel('Number of Images')
    axis2.set_ylabel('Precision')
    axis2.plot(num_images, precision, '-o', label=name)
    axis2.set_xlim([0.0, 50.0])
    axis2.grid(True)
    axis2.legend()

def accumulate_evaluation(path, threshold):
    figure1 = plt.figure("Map Data")

    #TODO: Iterate over multiple datasets
    plot_evaluation(figure1, 'Ours', path, threshold)
    plt.tight_layout()
    plt.show()

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('path', metavar='p', type=str, help='path to datafile')
parser.add_argument('--threshold', metavar='t', type=float, default=1.0, help='error threshold')
arg_list = parser.parse_args()

accumulate_evaluation(**vars(arg_list))
