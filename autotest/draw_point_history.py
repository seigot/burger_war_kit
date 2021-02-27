#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
import sys

def draw_graph(csv_file, png_file):
    df = pd.read_csv(csv_file,
                     encoding="UTF8",
                     names=('sec', 'you', 'enemy', 'act_mode', 'event',
                            'index', 'point', 'before', 'after'),
                     usecols=['sec', 'you', 'enemy'],
                     index_col='sec')
    df.plot(color=['r','b'], drawstyle='steps-post', fontsize=15)

    plt.title("Point history")
    plt.xlabel("timestamp [sec]")
    plt.ylabel("point")

    plt.savefig(png_file)

if __name__ == "__main__":
    if (len(sys.argv) != 3):
        print("[usage]" + sys.argv[0] + " CSV_FILE PNG_FILE")
        exit(1)
    draw_graph(sys.argv[1], sys.argv[2])
