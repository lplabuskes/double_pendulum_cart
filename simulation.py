import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import dynamics

x0mean = np.array([0, 0, 0, 0, 0, 0])
x0cov = .01*np.identity(6)

cart = dynamics.CartSystem(x0mean, x0cov)

fig = plt.figure()
gs = fig.add_gridspec(3, 2)

cart_axes = fig.add_subplot(gs[:, 0])
cart_axes.axis('equal')
cart_axes.set(xlim=(-3, 3), ylim=(-2, 4))
cart_line, = cart_axes.plot([], [], 'k')
pendulum_line, = cart_axes.plot([], [], '.k-', markersize=12)


max_len = 100
time_data = []
x_data = []
x_est_data = []
theta1_data = []
theta1_est_data = []
theta2_data = []
theta2_est_data = []

x_axes = fig.add_subplot(gs[0, 1])
x_actual, = x_axes.plot([], [], 'k')
x_est, = x_axes.plot([], [], 'g--')

theta1_axes = fig.add_subplot(gs[1, 1], sharex=x_axes)
theta1_actual, = theta1_axes.plot([], [], 'k')
theta1_est, = theta1_axes.plot([], [], 'g--')

theta2_axes = fig.add_subplot(gs[2, 1], sharex=theta1_axes)
theta2_actual, = theta2_axes.plot([], [], 'k')
theta2_est, = theta2_axes.plot([], [], 'g--')

plt.style.use("ggplot")


def animation_function(frame, simulator):
    simulator.update()
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

    estimate = simulator.est_state
    time_data.append(simulator.time)
    x_data.append(state[0])
    x_est_data.append(estimate[0])
    theta1_data.append(state[1])
    theta1_est_data.append(estimate[1])
    theta2_data.append(state[2])
    theta2_est_data.append(estimate[2])

    if len(time_data) > max_len:  # All data arrays should be same length
        time_data.pop(0)
        x_data.pop(0)
        x_est_data.pop(0)
        theta1_data.pop(0)
        theta1_est_data.pop(0)
        theta2_data.pop(0)
        theta2_est_data.pop(0)

    x_actual.set_xdata(time_data)
    x_actual.set_ydata(x_data)
    x_est.set_xdata(time_data)
    x_est.set_ydata(x_est_data)

    theta1_actual.set_xdata(time_data)
    theta1_actual.set_ydata(theta1_data)
    theta1_est.set_xdata(time_data)
    theta1_est.set_ydata(theta1_est_data)

    theta2_actual.set_xdata(time_data)
    theta2_actual.set_ydata(theta2_data)
    theta2_est.set_xdata(time_data)
    theta2_est.set_ydata(theta2_est_data)

    if len(time_data) >= 2:
        x_axes.set(xlim=(time_data[0], time_data[-1]),
                   ylim=(min(min(x_data), min(x_est_data)), max(max(x_data), max(x_est_data))))
        theta1_axes.set(xlim=(time_data[0], time_data[-1]),
                        ylim=(min(min(theta1_data), min(theta1_est_data)), max(max(theta1_data), max(theta1_est_data))))
        theta2_axes.set(xlim=(time_data[0], time_data[-1]),
                        ylim=(min(min(theta2_data), min(theta2_est_data)), max(max(theta2_data), max(theta2_est_data))))


anim = FuncAnimation(fig, animation_function, frames=100, fargs=(cart,), interval=10)
plt.show()
