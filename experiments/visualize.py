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
    axs[0].grid(axis='y')
    axs[0].bar(mean_index, mean, width=0.5, label="seconds")
    axs[0].legend(["Mean (seconds)"])

    axs[1].set_xticks(range(1, std.shape[0]+1))
    axs[1].grid(axis='y')
    axs[1].bar(std_index, std, width=0.5, color='red')
    axs[1].legend(["Standard deviation (seconds)"])

    fig.suptitle("Latency")
    fig.savefig('res.svg')

def plot_data_compact(mean, std):
    std_max = np.round(np.max(std), decimals=4)
    std_mean = np.round(np.mean(std), decimals=4)
    mean_index = np.linspace(1, mean.shape[0], num=mean.shape[0], dtype=int)
    plt.bar(mean_index, mean, color = "#4CAF50", width=0.8)
    plt.ylabel("latency (seconds)", fontsize = 13)
    plt.text(29, 3.8, "std: mean = " + str(std_mean) + ", max = " + str(std_max), fontsize=13)
    plt.gcf().set_size_inches(10, 2)
    ax = plt.gca()
    ax.get_xaxis().set_visible(False)
    ax.set_xlim([0,51])
    plt.savefig('res.svg', bbox_inches='tight')

def plot_data_compact_overlay(mean, std):
    std_max = np.round(np.max(std), decimals=4)
    std_mean = np.round(np.mean(std), decimals=4)
    mean_index = np.linspace(1, mean.shape[0], num=mean.shape[0], dtype=int)
    plt.bar(mean_index, mean, color = "#4CAF50", width=0.8)
    plt.bar(mean_index, std, width=0.8)
    plt.ylabel("latency (seconds)", fontsize = 13)
    plt.text(1, 9, "std: mean = " + str(std_mean) + ", max = " + str(std_max), fontsize=13)
    plt.gcf().set_size_inches(10, 2)
    ax = plt.gca()
    ax.get_xaxis().set_visible(False)
    ax.set_xlim([0,51])
    plt.savefig('res.svg', bbox_inches='tight')

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('data_path')
    args = parser.parse_args()

    data_path = args.data_path

    (mean, std) = process_data(data_path)

    if mean is not None and std is not None:
        plot_data_compact_overlay(mean, std)
    else:
        print("Failed to process data!")

if __name__ == '__main__':
    main()