import io, matplotlib.pyplot as plt

file = open("test-output/test-mp.txt", "r")

lines = file.readlines()

def generate(lines, key: str):
    output_t = []
    output_k = []
    for line in lines:
        split = line.split();
        if len(split) >= 3 and split[0] == key:
            output_t.append(float(split[1]))
            output_k.append(float(split[2]))

    return (output_t, output_k)

def plot(ax, xs, ys, style="-"):
    (l,) = ax.plot(xs, ys, linestyle=style)
    return l

fig, ((ax_pos, ax_h), (ax_vx, ax_vh), (ax_vy, b)) = plt.subplots(3, 2)
ax_pos.axis('equal')

x_list_t, x_list = generate(lines, "x")
y_list_t, y_list = generate(lines, "y")
h_list_t, h_list = generate(lines, "h")
vx_list_t, vx_list = generate(lines, "vx")
vy_list_t, vy_list = generate(lines, "vy")
vh_list_t, vh_list = generate(lines, "vh")

# plot(ax, x_list, y_list)

plot(ax_pos, x_list, y_list)
plot(ax_h, h_list_t, h_list)

plot(ax_vx, vx_list_t, vx_list)
plot(ax_vy, vy_list_t, vy_list)
plot(ax_vh, vh_list_t, vh_list)

ax_pos.grid()
ax_h.grid()
ax_vx.grid()
ax_vy.grid()
ax_vh.grid()
# ax_x.set_xlim([0, x_list_t[len(x_list_t)-1]])
# ax_x.set_ylim([-1.8, 1.8])
plt.show()
