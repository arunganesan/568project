import numpy as np
import matplotlib.pyplot as plt

xs = np.arange(0, 2, 0.01)
ys = [x**2 - 2*x for x in xs]
def y(x):
    return 8*x - 12.75
plt.plot(xs, ys)
plt.plot([1.25, 1.75], [y(1.25), y(1.75)])
plt.xlim(1, 2)
plt.ylim([-1.5, 1]);
