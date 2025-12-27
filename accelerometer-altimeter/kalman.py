from ulab import numpy as np


class KalmanFilter:
    # Adapted from https://www.geeksforgeeks.org/python/kalman-filter-in-python/
    def __init__(self, delta_t, accel_err_std_dev, alti_err_std_dev):
        F = np.array([[1, delta_t], [0, 1]])
        G = np.array([[0.5 * delta_t ** 2], [delta_t]])
        H = np.array([[1, 0]])
        Q = np.array([[(delta_t ** 4)/4, (delta_t ** 3)/2],[(delta_t ** 3)/2, delta_t **2]]) * accel_err_std_dev ** 2
        R = np.array([[alti_err_std_dev ** 2]])

        x0 = np.array([[0], [0]])
        P0 = np.array([[500, 0], [0, 500]])

        self.F = F
        self.G = G
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0
    
    @micropython.native
    def predict(self, u: float) -> None:
        u -= 9.8
        u = np.array([[u]])
        self.x = np.dot(self.F, self.x) + np.dot(self.G, u)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    @micropython.native
    def update(self, z : float) -> None:
        z = np.array([[z]])
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

class StateEstimator:

    def __init__(self, period: float, accel_err: float, alti_err: float):
        """Initialize the estimator

        This initializes a Kalman filter using the supplied timestep and errors (as standard deviations)

        :param float period: The time between readings in milliseconds
        :param float alti_err: The standard deviation of altimeter readings in meters
        :param float accel_err: The standard deviation of accelerometer readings in meters per second per second
        """
        self.KF = KalmanFilter(period / 1000, accel_err, alti_err)
        # Change this if you're launching somewhere other than earth 😄
        self.acceleration = 9.80665 # https://en.wikipedia.org/wiki/Standard_gravity 
        

    @property
    def altitude(self) -> float:
        return self.KF.x[0][0]

    @property
    def velocity(self) -> float:
        return self.KF.x[1][0]

    @altitude.setter
    def altitude(self, value) -> None:
        self.KF.update(value)

    @altitude.setter
    def acceleration(self, value) -> None:
        self.KF.predict(value)