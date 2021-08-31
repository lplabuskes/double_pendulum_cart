import numpy as np


class CartSystem:
    dt = .01

    def __init__(self, initial_conditions):
        self.state = initial_conditions

    def system_dynamics(self, u):
        # Constants
        m_c = 3  # cart mass
        m_p = 1  # pendulum mass
        l_p = 1  # pendulum length
        g = 9.81
        c_c = .2  # cart damping coefficient
        c_p = .1  # pendulum damping coefficient

        # State Equations
        theta1, theta2 = self.state[1], self.state[2]
        theta1dot, theta2dot = self.state[4], self.state[5]

        # Equations derived from Lagrangian mechanics
        mass_matrix = np.array([[m_c+2*m_p, -2*m_p*l_p*np.cos(theta1), -m_p*l_p*np.cos(theta2)],
                                [-2*m_p*l_p*np.cos(theta1), 2*m_p*l_p*l_p, m_p*l_p*l_p*np.cos(theta1-theta2)],
                                [-m_p*l_p*np.cos(theta2), m_p*l_p*l_p*np.cos(theta1-theta2), m_p*l_p*l_p]])

        rhs = np.array([[u - 2*m_p*l_p*(theta1dot**2)*np.sin(theta1) - m_p*l_p*(theta2dot**2)*np.sin(theta2)],
                        [2*m_p*g*l_p*np.sin(theta1) - m_p*l_p*l_p*(theta2dot**2)*np.sin(theta1-theta2)],
                        [m_p*g*l_p*np.sin(theta2) + m_p*l_p*l_p*(theta1dot**2)*np.sin(theta1-theta2)]])

        second_derivs = np.linalg.solve(mass_matrix, rhs)

        state_derivs = np.array([self.state[3], self.state[4], self.state[5], second_derivs[0][0], second_derivs[1][0], second_derivs[2][0]])

        # Include damping
        state_derivs[3:] -= np.array([c_c, c_p, c_p]).T * self.state[3:]

        return state_derivs

    def update(self, u):
        self.state += self.dt * self.system_dynamics(u)
