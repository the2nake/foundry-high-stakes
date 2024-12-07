import io, matplotlib.pyplot as plt

file = open("test-output/tuning.txt", "r")
lines = file.readlines()
split_lines = []

for line in lines:
    split_lines.append(line.split(" "))

t = []
l = []
r = []

for splitted in split_lines:
    t.append(int(splitted[0]) - int(split_lines[0][0]))
    l.append(float(splitted[1]))
    r.append(float(splitted[2]))

axis = plt.axes();
axis.xaxis.set_major_locator(plt.MaxNLocator(15))
axis.yaxis.set_major_locator(plt.MaxNLocator(30))

# Plotting both the curves simultaneously
plt.plot(t, l, label="left")
plt.plot(t, r, label="right")

# Adding legend, which helps us recognize the curve according to it's color
plt.legend()
plt.axis()
plt.grid()

# To load the display window
plt.show()
