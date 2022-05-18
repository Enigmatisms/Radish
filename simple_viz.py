import numpy as np
import matplotlib.pyplot as plt
from sys import argv
from time import time

def readFromFile(path):
    with open(argv[1], 'r') as file:
        raw = file.readlines()
        lines = [x[:-1].split(" ") for x in raw]
        result = [[float(x) for x in line] for line in lines]
    return np.array(result)

if __name__ == "__main__":
   values = readFromFile(argv[1])
   print(values)
   plt.plot(values[:, 1], values[:, 2], c = 'b')
   plt.scatter(values[:, 1], values[:, 2], c = 'r', s = 4)
   plt.grid(axis = 'both')
   plt.show()