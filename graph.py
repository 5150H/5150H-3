import numpy as np
import matplotlib.pyplot as plt



data = np.genfromtxt("vs.csv", delimiter=",")
#define data

y = data[:, 0]
x = data[:, 1]

#find line of best fit
a, b = np.polyfit(x, y, 1)

#add points to plot
plt.scatter(x, y)

#add line of best fit to plot
plt.plot(x, a*x+b)

plt.text(2000, 0, 'y = '+ '{:.2f}'.format(a) + 'x + ' + '{:.2f}'.format(b) , size=14)
plt.show()