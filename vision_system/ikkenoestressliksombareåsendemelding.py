import numpy as np
import math
import matplotlib.pyplot as plt



k_rep = 1



for distance in range(20,60,1):
    repulsive_force = k_rep*((1/(distance))-(1/distance**3))*(1/(distance**3))
    plt.plot(distance, repulsive_force)

plt.show()