import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

# define pts from the question

pts = np.array([[0, 0], [1, 1], [2, 0], [3, 1], [4, 0], [5, 1]])

tck, u = splprep(pts.T, u=None, s=0.0) 
u_new = np.linspace(u.min(), u.max(), 50)
x_new, y_new = splev(u_new, tck, der=0)

print(u_new)
plt.plot(pts[:,0], pts[:,1], 'ro')
plt.plot(x_new, y_new, 'b--')
plt.show()