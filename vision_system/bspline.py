import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

# define pts from the question
pts = np.array([
 [ 58.  , 281.  ],
 [ 77.52 ,289.31],
 [ 96.53 ,299.37],
 [103.91 ,330.22],
 [124.37 ,333.58],
 [146.35 ,333.31],
 [165.1  ,341.48],
 [180.32 ,356.87],
 [200.79 ,360.25],
 [213.96 ,379.63],
 [232.3  ,387.46],
 [254.23 ,387.08],
 [277.16 ,385.12],
 [298.87 ,389.44],
 [319.64 ,400.19],
 [338.22 ,408.45],
 [356.84 ,416.76],
 [377.45 ,426.22],
 [398.31 ,434.7 ]])

tck, u = splprep(pts.T, u=None, s=0.0) 
u_new = np.linspace(u.min(), u.max(), 100)
x_new, y_new = splev(u_new, tck, der=0)

plt.plot(pts[:,0], pts[:,1], 'ro')
plt.plot(x_new, y_new, 'b--')
plt.show()