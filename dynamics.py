import numpy as np


class CartSystem:
    dt = .01
    time = 0
    process_noise = .1*np.identity(6)
    measurement_noise = .03*np.identity(3)

    def __init__(self, x0mean, x0cov):
        self.est_state = x0mean
        self.est_cov = x0cov
        self.state = np.random.multivariate_normal(x0mean, x0cov)

    def system_dynamics(self, state, u, mod=False):
        # Constants
        m_c = 3  # cart mass
        m_p = 1  # pendulum mass
        l_p = 1  # pendulum length
        g = 9.81
        c_c = .2  # cart damping coefficient
        c_p = .15  # pendulum damping coefficient

        # State Equations
        theta1, theta2 = state[1], state[2]
        theta1dot, theta2dot = state[4], state[5]

        # Equations derived from Lagrangian mechanics
        mass_matrix = np.array([[m_c+2*m_p, -2*m_p*l_p*np.cos(theta1), -m_p*l_p*np.cos(theta2)],
                                [-2*m_p*l_p*np.cos(theta1), 2*m_p*l_p*l_p, m_p*l_p*l_p*np.cos(theta1-theta2)],
                                [-m_p*l_p*np.cos(theta2), m_p*l_p*l_p*np.cos(theta1-theta2), m_p*l_p*l_p]])

        rhs = np.array([[u - 2*m_p*l_p*(theta1dot**2)*np.sin(theta1) - m_p*l_p*(theta2dot**2)*np.sin(theta2)],
                        [2*m_p*g*l_p*np.sin(theta1) - m_p*l_p*l_p*(theta2dot**2)*np.sin(theta1-theta2)],
                        [m_p*g*l_p*np.sin(theta2) + m_p*l_p*l_p*(theta1dot**2)*np.sin(theta1-theta2)]])

        second_derivs = np.linalg.solve(mass_matrix, rhs)

        state_derivs = np.array([state[3], state[4], state[5], second_derivs[0][0], second_derivs[1][0], second_derivs[2][0]])

        # Include damping
        state_derivs[3:] -= np.array([c_c, c_p, c_p]) * state[3:]
        state_derivs += np.random.multivariate_normal(np.zeros(6), self.process_noise)
        next_state = state + self.dt*state_derivs

        if mod:
            next_state[1] = np.mod(next_state[1], (2*np.pi))
            next_state[2] = np.mod(next_state[2], (2*np.pi))

        return next_state

    # An unscented Kalman filter implementation
    def estimator_update(self, u_prev):
        measurement = self.state[:3]+np.random.multivariate_normal(np.zeros(3), self.measurement_noise)

        # Generate sigma points of state. Since measurements give state variables, no further processing needed
        sigma_spacing = np.linalg.cholesky(6*self.est_cov)
        x_sigma_points = []
        for i in range(6):
            temp_state = self.est_state + sigma_spacing[:, i]
            x_sigma_points.append(self.system_dynamics(temp_state, u_prev))
            temp_state = self.est_state - sigma_spacing[:, i]
            x_sigma_points.append(self.system_dynamics(temp_state, u_prev))
        for i in range(6):
            x_sigma_points.append(self.system_dynamics(self.est_state, u_prev))

        # Generate statistics of state prediction sigma points
        predicted_state = sum(x_sigma_points)/len(x_sigma_points)
        predicted_measurement = np.copy(predicted_state[:3])

        predicted_covariance = np.zeros((6, 6))
        for i in range(len(x_sigma_points)):
            predicted_covariance += (x_sigma_points[i]-predicted_state) @ (x_sigma_points[i]-predicted_state).T
        predicted_covariance /= len(x_sigma_points)
        measurement_covariance = np.copy(predicted_covariance[:3, :3])  # a little hacky, possible due to direct measurement
        cross_covariance = np.copy(predicted_covariance[:, :3])  # see above

        predicted_covariance += self.process_noise
        measurement_covariance += self.measurement_noise

        kalman_gain = cross_covariance @ np.linalg.inv(measurement_covariance)
        self.est_state = predicted_state + kalman_gain @ (measurement - predicted_measurement)
        self.est_cov = predicted_covariance - kalman_gain @ measurement_covariance @ kalman_gain.T

    def update(self):
        self.time += self.dt
        # compute control
        # update dynamics
        # estimate state
        self.state = self.system_dynamics(self.state, 0)
        self.estimator_update(0)
