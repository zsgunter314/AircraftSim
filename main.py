import numpy as np
from matplotlib import pyplot as plt

from airplane_sim.first_airplane import airplane_zsg

Air = airplane_zsg()

x = np.arange(0,20,1)

y = 2 * x + 5
plt.title("Matplotlib demo") 
plt.xlabel("x axis caption") 
plt.ylabel("y axis caption") 
plt.plot(x,y)
plt.show()


print("Hello World")
