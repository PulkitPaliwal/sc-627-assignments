import matplotlib.pyplot as plt
x_coordinates = [coordinates[i][0] for i in range(len(coordinates))]
y_coordinates = [coordinates[i][1] for i in range(len(coordinates))]
plt.plot(x_coordinates, y_coordinates, color = "black")
plt.plot(-2.149935112985904e-05, 8.475973582964367e-05, "s", color="black")
plt.plot(2.4974476784164144, -2.501287794745341, "s", color="black")
plt.xlabel("X")
plt.ylabel("Y")
plt.axes()
rectangle = plt.Rectangle((0.5,-1.5), 1, 1, fc='red',ec="red")
circle = plt.Circle((2,-2),0.1, fc='red',ec="red")
plt.gca().add_patch(circle)
plt.gca().add_patch(rectangle)
plt.show()