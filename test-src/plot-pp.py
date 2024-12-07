import io, matplotlib.pyplot as plt, matplotlib.animation as anim

file = open("test-output/test-pp.txt", "r")


def split_data(parsed):
    xs = []
    ys = []
    for item in parsed:
        xs.append(item[0])
        ys.append(item[1])
    return xs, ys


def plot_list(ax, parsed_list, style="-"):
    xs = []
    ys = []
    for item in parsed_list:
        xs.append(item[0])
        ys.append(item[1])
    (l,) = ax.plot(xs, ys, linestyle=style)
    return l


def scatter_list(ax, parsed_list, style="o"):
    xs = []
    ys = []
    for item in parsed_list:
        xs.append(item[0])
        ys.append(item[1])
    ax.scatter(xs, ys, marker=style)

def gen_parsed(lines, str):
    parsed = []
    for line in lines:
        split = line.split(" ")
        if len(split) >= 3:
            if split[0] == str:
                parsed.append([float(i) for i in split[1:]])
    return parsed


def plot(lines, str, ax, style="-"):
    return plot_list(ax, gen_parsed(lines, str), style)


def scatter(lines, str, ax, style="-"):
    return scatter_list(ax, gen_parsed(lines, str), style)


lines = file.readlines()

fig, ax = plt.subplots()

pursuit_data = gen_parsed(lines, "pursuit")
pursuit_x, pursuit_y = split_data(pursuit_data)[0], split_data(pursuit_data)[1]

plot(lines, "spline", ax, "-.")
pursuit = ax.plot(pursuit_x[0], pursuit_y[0])[0]
scatter(lines, "ctrl", ax, ".")


def anim_update(frame):
    global pursuit_circle
    global pursuit_carrot
    x = pursuit_x[: frame + 1]
    y = pursuit_y[: frame + 1]
    pursuit.set_xdata(x)
    pursuit.set_ydata(y)
    try:
        pursuit_circle.remove()
        pursuit_carrot.remove()
    except NameError:
        pass
    if len(x) > 0:
        pursuit_circle = plt.Circle(
            (x[len(x) - 1], y[len(y) - 1]),
            pursuit_data[len(pursuit_data) - 1][2],
            color="orange",
            lw=2,
            fill=False,
            clip_on=False,
        )
        pursuit_carrot = plt.Circle(
            (pursuit_data[frame][3], pursuit_data[frame][4]),
            0.01,
            color="green",
            lw=2,
            fill=False,
            clip_on=False,
        )
        ax.add_patch(pursuit_circle)

        ax.add_patch(pursuit_carrot)
    return pursuit


ani = anim.FuncAnimation(
    fig=fig, func=anim_update, frames=len(pursuit_data), interval=10
)

# ax.plot([0.0, 1], [0.0, 1])
# ax.plot([0.5, 1], [0.25, 2])
ax.grid()
ax.set_xlim([-1.8, 1.8])
ax.set_ylim([-1.8, 1.8])
plt.show()

ani.save("test-output/pursuit-anim.mp4")
