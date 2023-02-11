
from matplotlib import pyplot as plt
plt.rcParams["figure.figsize"] = [7.00, 7.00]
plt.rcParams["figure.autolayout"] = True

plt.grid()
plt.plot([data[i][0] for i in range(len(data))], [data[i][1] for i in range(len(data))])
plt.show()