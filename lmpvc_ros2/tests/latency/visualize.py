#!/usr/bin/env python3
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt

def process_data(data_path):
    
    data = None
    mean = None
    std = None

    with open(data_path, 'r') as datafile:
        data = np.asarray(list(csv.reader(datafile)), dtype=float)

    if data is not None:
        mean = np.zeros(data.shape[0], dtype=float)
        std = np.zeros(data.shape[0], dtype=float)

        for i in range(data.shape[0]):
            mean[i] = np.mean(data[i])
            std[i] = np.std(data[i])
    
    return mean, std

def plot_data(mean, std):
    
    fig, axs = plt.subplots(2)

    mean_index = np.linspace(1, mean.shape[0], num=mean.shape[0], dtype=int)
    std_index = np.linspace(1, std.shape[0], num=std.shape[0], dtype=int)

    axs[0].set_xticks(range(1, mean.shape[0]+1))
    axs[0].bar(mean_index, mean)

    axs[1].set_xticks(range(1, std.shape[0]+1))
    axs[1].bar(std_index, std, color='red')

    plt.savefig('res.svg')

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('data_path')
    args = parser.parse_args()

    data_path = args.data_path

    (mean, std) = process_data(data_path)

    if mean is not None and std is not None:
        plot_data(mean, std)
    else:
        print("Failed to process data!")

if __name__ == '__main__':
    main()