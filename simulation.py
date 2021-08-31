import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import dynamics

x0 = np.array([0, .01, -.01, 0, 0, 0])
cart = dynamics.CartSystem(x0)

fig, axes = plt.subplots(1, 1)
axes.set_ylim(-2, 4)
axes.set_xlim(-3, 3)
cart_line, = axes.plot([], [], 'k')
pendulum_line, = axes.plot([], [], '.k-', markersize=12)
plt.style.use("ggplot")


def animation_function(frame, simulator):
    simulator.update(0)
    state = simulator.state

    width = .6
    height = .4
    length = 1
    x_left, x_right = state[0] - width/2, state[0] + width/2

    cart_x = np.array([x_left, x_right, x_right, x_left, x_left])
    cart_y = np.array([0, 0, height, height, 0])

    pendulum_x = state[0] + np.array([0, -length*np.sin(state[1]), -length*(np.sin(state[1]) + np.sin(state[2]))])
    pendulum_y = height/2 + np.array([0, length*np.cos(state[1]), length*(np.cos(state[1]) + np.cos(state[2]))])

    cart_line.set_xdata(cart_x)
    cart_line.set_ydata(cart_y)

    pendulum_line.set_xdata(pendulum_x)
    pendulum_line.set_ydata(pendulum_y)


anim = FuncAnimation(fig, animation_function, frames=100, fargs=(cart,), interval=10)
plt.show()
