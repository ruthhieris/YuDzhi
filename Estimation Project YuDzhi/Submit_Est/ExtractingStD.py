#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  9 23:42:09 2018

@author: yudzhi
"""

import pandas as pd
from pandas import DataFrame
from matplotlib import pyplot as plt

def find_std(csv_name):
    df = pd.read_csv(csv_name, delimiter = ',', header = 0, usecols = [1], parse_dates=True)
    #print(df.head())
    data_std = df.std()
    return data_std

if __name__ == "__main__":
    csv_name1 = "Graph1.txt"
    csv_name2 = "Graph2.txt"
    std1 = find_std(csv_name1)
    std2 = find_std(csv_name2)
    print(std1, std2)