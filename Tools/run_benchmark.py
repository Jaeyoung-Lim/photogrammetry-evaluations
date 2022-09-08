import evaluate_map
import argparse
import matplotlib.pyplot as plt
import os

def benchmark(path, threshold):
    figure1 = plt.figure("Map Data")

    #TODO: Iterate over multiple datasets
    for filename in os.listdir(path):
        benchmark_dir = os.path.join(path, filename)
        print(benchmark_dir)
        num_images, completeness, precision = evaluate_map.model_evaluation(benchmark_dir, threshold)
        print(completeness)

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

benchmark(**vars(arg_list))
